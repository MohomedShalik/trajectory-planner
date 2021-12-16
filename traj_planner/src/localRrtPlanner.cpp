

#include "localRrtPlanner.h"
#include "nav_msgs/Odometry.h"
#include "kdtree.h"
#include <iostream>
#include <pcl/io/ply_io.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <math.h>


#define PI 3.141592653589793238463

#define DEG(X)\
    X*(180/PI)
#define RAD(X)\
    X*(PI/180)


#define RESOLUTION 0.05
#define VERT_FOV 22.5
#define VERT_FOV_5 10
#define to_pcl_type(x) (pcl::PointXYZ(x(0) , x(1) , x(2)))
#define inf 999999999.0

#define V_VAL NAN
#define A_VAL NAN

#define VEL_x V_VAL
#define VEL_y V_VAL
#define VEL_z V_VAL
#define ACCL_x A_VAL
#define ACCL_y A_VAL
#define ACCL_z A_VAL


void delete_node(void* node)
{
      delete static_cast<NodePtr>(node);
}

inline double sq_dist(Eigen::Vector3d pt1 , Eigen::Vector3d pt2)
{

    //double i = (pt2 - pt1).squaredNorm();
    
    double i = sqrt(pow(pt2(0) - pt1(0),2)  + pow(pt2(1) - pt1(1),2) + pow(pt2(2) - pt1(2),2));
    return i;

}

LocalRrtPlanner::LocalRrtPlanner(ros::NodeHandle &nh):cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
                                                    cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>),
                                                    local_octree(RESOLUTION),step_dist(0.7) , safe_radius(0.6),
                                                    listener_(buffer_),
                                                    fov(30.5 * M_PI/180 , VERT_FOV_5 * M_PI/180 , 0.1 , 8.0),
                                                    rand_z(fov.near_plane_depth , fov.far_plane_depth),
                                                    rand_x(-1,1),rand_y(-1,1)
{
    kd_tree = kd_create(3);
    kd_data_destructor(kd_tree , delete_node);
    iris_pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom" , 10 , &LocalRrtPlanner::pose_callback, this);
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/realsense_camera/camera/depth_registered/points" , 10 , &LocalRrtPlanner::cloud_callback ,
                this);  
    point_pub = nh.advertise<geometry_msgs::PointStamped>("/sample/point", 1000);
    rrtVis = nh.advertise<visualization_msgs::Marker>("/tree/generated_paths" , 1000);
    pVis = nh.advertise<visualization_msgs::Marker>("/tree/selected_path" , 1000);
    adaptedPathPub  = nh.advertise<mavros_msgs::Trajectory>("traj_planner/trajectory/generated" , 20 );
    waypoint_sub  = nh.subscribe("mavros/trajectory/desired" , 10, &LocalRrtPlanner::desiredTraj , this);

    localPoseSub  = nh.subscribe<geometry_msgs::PoseStamped>
                          ("mavros/local_position/pose" , 10 , &LocalRrtPlanner::poseCallb , this );

    edgs.clear();
    while(!buffer_.canTransform("color" , "map" , ros::Time::now() , ros::Duration(1,0))){
        ROS_WARN("transform unavailable");   
        continue;
    }
    ROS_INFO("color coordinate to map coordinate transform available");
    pass_through_filter.setFilterFieldName ("z");
    pass_through_filter.setFilterLimits (0.0, 8.5);
    voxel_filter.setLeafSize (0.03f, 0.03f, 0.03f);
    

}

void LocalRrtPlanner::poseCallb(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{

  this->localPosition = *_msg;
  
}

void LocalRrtPlanner::pose_callback(const nav_msgs::Odometry::ConstPtr &pose_in)
{

    iris_pose.header = pose_in->header;
    iris_pose.child_frame_id = pose_in->child_frame_id;
    iris_pose.pose = pose_in->pose;
    iris_pose.twist = pose_in->twist;
}

void LocalRrtPlanner::desiredTraj(const mavros_msgs::Trajectory::ConstPtr& _msg)
{

    if ( _msg == NULL ) {
        return;
    }
   // desiredPath = *_msg;
    if (_msg->point_valid[0] != true) return;

    goal_point(0) = _msg->point_1.position.x;
    goal_point(1) = _msg->point_1.position.y;
    goal_point(2) = _msg->point_1.position.z;
    
    this->desiredPath.point_valid[0] = true;
    this->desiredPath.point_1.position.x   = _msg->point_1.position.x;
    this->desiredPath.point_1.position.y   = _msg->point_1.position.y;
    this->desiredPath.point_1.position.z   = _msg->point_1.position.z;
    this->desiredPath.point_1.velocity.x   = _msg->point_1.velocity.x;
    this->desiredPath.point_1.velocity.y   = _msg->point_1.velocity.y;
    this->desiredPath.point_1.velocity.z   = _msg->point_1.velocity.z;
    this->desiredPath.point_1.yaw          = _msg->point_2.yaw;
    this->desiredPath.point_1.yaw_rate     = _msg->point_1.yaw_rate;

    this->desiredPath.point_2.yaw          = _msg->point_2.yaw;

    ROS_INFO("goal is %f %f %f " , goal_point(0),goal_point(1),goal_point(2));

}


void LocalRrtPlanner::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
{
    ros::Time time_bef_expand = ros::Time::now();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_in,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    
    // pass_through_filter.setInputCloud (cloud);
    // pass_through_filter.filter (*cloud_filtered);
    // voxel_filter.setInputCloud (cloud_filtered);
    // voxel_filter.filter (*cloud_voxel_filtered);
    
    local_octree.deleteTree();
    local_octree.setInputCloud(cloud);
    local_octree.addPointsFromInputCloud();
    // double elapsed_time = (ros::Time::now() - time_bef_expand).toSec();
    // ROS_WARN("eELAPSED time %f seconds " , elapsed_time);
}

Eigen::Vector3d LocalRrtPlanner::sample_rand_point()
{

    double r_z = rand_z(generator);
    double width_x =  r_z * tan(30   * M_PI /180.0);
    double width_y =  r_z * tan(VERT_FOV_5 * M_PI/180.0);
    if (width_y > 0) width_y = -width_y;
    double r_x = rand_x(generator) * width_x;
    double r_y = rand_y(generator) * width_y;
    Eigen::Vector3d rand_point(r_x , r_y , r_z);
    return rand_point;
}

Eigen::Vector3d LocalRrtPlanner::apply_point_transform(Eigen::Vector3d pt, const std::string& target_frame , const std::string& source_frame)
{
    Eigen::Vector3d rand_point;

    tf2::Vector3 point(pt(0), pt(1) , pt(2));
    /*while(!buffer_.canTransform("color" , "map" , ros::Time::now() , ros::Duration(1,0))){
        ROS_ERROR("transform unavailable");   
        continue;
    }*/

    geometry_msgs::TransformStamped trmsg = buffer_.lookupTransform(
      target_frame, source_frame,ros::Time(0)
    );
    tf2::Stamped< tf2::Transform > transform;
    tf2::fromMsg(trmsg , transform);
    point = transform * point;
    rand_point(0) = point.x();
    rand_point(1) = point.y();
    rand_point(2) = point.z();
    return Eigen::Vector3d(point.x() , point.y() , point.z());
}

double LocalRrtPlanner::radius_search(Eigen::Vector3d pt)
{

    pcl::PointXYZ searchPoint;

    pointIdxRadSearch.clear();
    pointRadSquaredDistance.clear();

    searchPoint.x = pt(0);
    searchPoint.y = pt(1);
    searchPoint.z = pt(2);
    int K = 1;
    if ( (local_octree.radiusSearch (searchPoint,safe_radius ,pointIdxRadSearch, pointRadSquaredDistance))> 0){
        // double max = *std::max_element(pointRadSquaredDistance.begin(), pointRadSquaredDistance.end());
        // std::cout << "vectorsize is " << pointRadSquaredDistance.size() << std::endl;
        return 0.3;
    }   
    else{
        ROS_INFO("no points within safe distance");
        return safe_radius;
    }
}

float LocalRrtPlanner::radius_free_search(Eigen::Vector3d pt)
{

    pcl::PointXYZ searchPoint;
    searchPoint.x = pt(0);
    searchPoint.y = pt(1);
    searchPoint.z = pt(2);
    pointIdxRadSearch.clear();
    pointRadSquaredDistance.clear();
    
    local_octree.nearestKSearch(searchPoint , 1 ,pointIdxRadSearch, pointRadSquaredDistance);
    return sqrt(pointRadSquaredDistance[0]) - 0.2;

}

void LocalRrtPlanner::add_free_space(Eigen::Vector3d pt  , double radius)
{
    Sphere safe_sphere(pt , radius);
    free_space.push_back(safe_sphere);
}

void LocalRrtPlanner::publish_point(Eigen::Vector3d pt)
{
    geometry_msgs::PointStamped pub_point;
    pub_point.header.stamp = ros::Time::now();
    pub_point.header.frame_id = "color";
    pub_point.point.x = pt(0);
    pub_point.point.y = pt(1);
    pub_point.point.z = pt(2);
    point_pub.publish(pub_point);
}


void LocalRrtPlanner::insertNode(NodePtr node ,Eigen::Vector3d pos)
{
    double arr[3] = {pos(0) ,pos(1) ,pos(2)};
    kd_insert(kd_tree , arr ,(void*)node);
}


void LocalRrtPlanner::create_root_node()
{
    
    Eigen::Vector3d cam_frame_pose(0 , 0 , 0.1);
    double arr[3] = {0 , 0 , 0.1};
    double current_rad = 0.5;
    float g = sq_dist(cam_frame_pose , cam_frame_pose);
    float f = sq_dist(cam_frame_pose , goal_point);
    root_node = new Node(cam_frame_pose , current_rad , g , f);
    nodes.push_back(root_node);
    kd_insert(kd_tree , arr ,(void*)root_node);
}


bool LocalRrtPlanner::checkIntersect(Eigen::Vector3d pt_cam_frame)
{
    traced_pts.clear();
    bool valid = true;
    Eigen::Vector3d origin(0 , 0, 0.1);
    Eigen::Vector3d direction = (pt_cam_frame - origin);
    direction.normalize();
    double dist = sq_dist(origin , pt_cam_frame);
    double ray_dist = 0;
    Eigen::Vector3d move_pt(0 , 0 , 0.1);
    traced_pts.push_back(to_pcl_type(move_pt));
    while(ray_dist <= dist)
    {
        if (radius_free_search(move_pt) == false)return false;    
        move_pt += step_dist * direction;
        traced_pts.push_back(to_pcl_type(move_pt));
        ray_dist = sq_dist(origin , move_pt);
    }
    return true;
}




NodePtr LocalRrtPlanner::nearestNeighbour(Eigen::Vector3d pt , Eigen::Vector3d& nearest_pt)
{

    double arr[3] = {pt(0) , pt(1) , pt(2)};
    double arr_n[3];
    struct kdres* nearest_node = kd_nearest(kd_tree , arr);
    NodePtr near((Node*)(kd_res_item(nearest_node, arr_n)));
    nearest_pt(0) = arr_n[0];
    nearest_pt(1) = arr_n[1];
    nearest_pt(2) = arr_n[2];
    kd_res_free(nearest_node);
    return near;
    
}

void LocalRrtPlanner::extendRRT(Eigen::Vector3d sampled_pt)
{

    Eigen::Vector3d n_pt;
    NodePtr nn = nearestNeighbour(sampled_pt , n_pt);
    ROS_INFO("nearest neighbour %f %f %f" , n_pt(0) ,n_pt(1),n_pt(2) );
    float g = nn->g + sq_dist(nn->pos, sampled_pt);
    if ( g <= step_dist)
    {
        traced_pts.clear();
        if (radius_free_search(sampled_pt) == false)
                return;
        float f = sq_dist(sampled_pt , goal_point);
        NodePtr new_node(new Node(sampled_pt , safe_radius ,g, f));
        new_node->parent_node = nn;
        nn->child_nodes.push_back(new_node);
        traced_pts.push_back(to_pcl_type(sampled_pt));
        edgs.push_back(std::make_pair(nn->pos , sampled_pt));
        nodes.push_back(new_node);
        
    }
    else
    {
        NodePtr nn = nearestNeighbour(sampled_pt , n_pt);
        Eigen::Vector3d direction = (sampled_pt - nn->pos);
        direction.normalize();
        Eigen::Vector3d st_pt = nn->pos;
        double dist = sq_dist(st_pt , sampled_pt);
        double ray_dist = 0;
        Eigen::Vector3d move_pt = nn->pos;
        float accum_g = nn->g;
        traced_pts.clear();
        while(ray_dist <= dist)
        {
            move_pt += step_dist * direction;

            if (radius_free_search(move_pt) == false)
                return;
            accum_g += sq_dist(nn->pos , move_pt);
            float f = sq_dist(move_pt, goal_point);
            NodePtr new_node(new Node(move_pt , safe_radius ,accum_g, f));
            new_node->valid = true;
            new_node->parent_node = nn;
            nn->child_nodes.push_back(new_node);
            insertNode(new_node , new_node->pos );
            edgs.push_back(std::make_pair(nn->pos , move_pt));
            nodes.push_back(new_node);
            nn = new_node;
            ray_dist = sq_dist(st_pt , move_pt);
        }

    }
}



//extend function for RRT
void LocalRrtPlanner::extendRRTStar(Eigen::Vector3d sampled_pt , Eigen::Vector3d n_pt ,  NodePtr nn , NodePtr& new_node)
{

    double arr[3] = {sampled_pt(0) , sampled_pt(1) , sampled_pt(2)};
    struct kdres* nearest_node_range = kd_nearest_range(kd_tree , arr , 1.2); //get neighbours around the sampled point of the RRT within a radius of 0.5
    //std::cout << "size checking" << std::endl;

    //if there are no neighbours around simply add the node to its nearest neigbour
    if (kd_res_size(nearest_node_range) == 0)
    {
        return;
    }
    
    else
    {
        ROS_INFO("size not 0");
        float min_dist , curr_val = 0;

        NodePtr best_nn;
        std::vector<NodePtr> ptrs;
        std::vector<Eigen::Vector3d> pts;
        // NodePtr nearPtr = (NodePtr)kd_res_item_data(nearest_node_range);
        NodePtr nearPtr = nn;
        curr_val = nearPtr->g + sq_dist(n_pt, sampled_pt) /*+ sq_dist(sampled_pt , goal_point)*/;
        best_nn  = nearPtr;
        Eigen::Vector3d near_pt ;
        ptrs.push_back(nearPtr);
        pts.push_back(n_pt);
        while (kd_res_next(nearest_node_range))
        {
            double pos[3];

            min_dist = curr_val;
            NodePtr nearPtr = (NodePtr)kd_res_item(nearest_node_range , pos);
            near_pt(0) = pos[0];
            near_pt(1) = pos[1];
            near_pt(2)=  pos[2];
            ptrs.push_back(nearPtr);
            pts.push_back(near_pt);
            curr_val = nearPtr->g + sq_dist(near_pt , sampled_pt) /*+ sq_dist(sampled_pt , goal_point)*/;
            if (curr_val < min_dist){
                min_dist = curr_val;
                best_nn = nearPtr;
            }
            continue;
            
        }
        float g = best_nn->g + sq_dist(near_pt, sampled_pt);
        float f = sq_dist(best_nn->pos , goal_point);
        NodePtr new_node(new Node(sampled_pt , safe_radius ,g, f));
       // insertNode(new_node);
        // edgs.push_back(std::make_pair(best_nn->pos , sampled_pt));
        

        ROS_INFO("tree rewiring");
        //tree rewiring procedure
        for (auto node : ptrs)
        {

            if (node->g > (best_nn->g + sq_dist(best_nn->pos , node->pos)))
            {
                // erase node from node->parent_node->child_nodes
                // std::vector<NodePtr>::iterator position = std::find(node->parent_node->child_nodes.begin(), 
                //                                                 node->parent_node->child_nodes.end(), node);
                // if (position != node->parent_node->child_nodes.end()) // 
                //         node->parent_node->child_nodes.erase(position);
                ROS_INFO("tree rewiring node");
                node->parent_node = best_nn;
                node->g = sq_dist(best_nn->pos , node->pos);
                best_nn->child_nodes.push_back(node);
                
            }
            continue;
        }
        kd_res_free(nearest_node_range);
    }
}



inline void toPointmsg(Eigen::Vector3d pt , geometry_msgs::Point& p)
{
    p.x = pt(0);
    p.y = pt(1);
    p.z = pt(2);

}


void LocalRrtPlanner::extendRRT(Eigen::Vector3d sampled_pt , Eigen::Vector3d& near_pt , NodePtr nn)
{

    ROS_INFO("nearest neighbour %f %f %f" , near_pt(0) ,near_pt(1),near_pt(2));

    float g = nn->g + sq_dist(near_pt, sampled_pt);
    Eigen::Vector3d goal_pt_col = apply_point_transform(goal_point , "color" , "map");
    if ( g <= step_dist)
    {

        

        Eigen::Vector3d sampled_pt_map  = apply_point_transform(sampled_pt , "map" , "color");
        float f = sq_dist(sampled_pt_map , goal_point);
        NodePtr new_node(new Node(sampled_pt_map , safe_radius ,g, f));
        new_node->parent_node = nn;
        nn->child_nodes.push_back(new_node);
        nodes.push_back(new_node);
        
    }

    else
    {
        //NodePtr nn = nearestNeighbour(sampled_pt , n_pt);
        Eigen::Vector3d step_pt_map;
        Eigen::Vector3d direction = (sampled_pt - near_pt);
        direction.normalize();
        Eigen::Vector3d st_pt = near_pt;
        double dist = sq_dist(st_pt , sampled_pt);
        double ray_dist = 0;
        Eigen::Vector3d move_pt = near_pt;
        float accum_g = nn->g;
        
        while(ray_dist <= dist)
        {
            move_pt += step_dist * direction;
            
            if (radius_free_search(move_pt) == false)
                return;
            accum_g += sq_dist(near_pt , move_pt);
            float f = sq_dist(move_pt, goal_pt_col);
            step_pt_map = apply_point_transform(move_pt , "map" , "color");
            NodePtr new_node(new Node(step_pt_map , safe_radius ,accum_g, f));
            new_node->valid = true;
            new_node->parent_node = nn;
            nn->child_nodes.push_back(new_node);
            insertNode(new_node , move_pt);
            // edgs.push_back(std::make_pair(near_pt , move_pt));
            nodes.push_back(new_node);
            nn = new_node;
            ray_dist = sq_dist(st_pt , move_pt);
        }

    }

}


void LocalRrtPlanner::build_local_rrt(double time_limit)
{
    
    ROS_INFO("build local rrt");
    edgs.clear();
    
    create_root_node();
    ros::Time time_bef_expand = ros::Time::now();
    ros::Time time_in_expand;
    double elapsed_time;
    for (int i = 0 ; i < 200 ; i++)
    {
        Eigen::Vector3d pt = sample_rand_point();
        bool valid = radius_free_search(pt);
        if (valid == false) {
            ROS_INFO("sample %u invvalid" , i);
            continue;}
        ROS_INFO("adding %u th sample" , i);
        pcl::PointXYZ searchPoint(pt(0) , pt(1) , pt(2));
        extendRRT(pt);
        time_in_expand = ros::Time::now();
        if( (elapsed_time = (time_in_expand - time_bef_expand).toSec()) > time_limit ) {
            ROS_WARN("TIME OVERDUE %f seconds %u th sample" , elapsed_time , i);
            break;
        }   
    }
    elapsed_time = (time_in_expand - time_bef_expand).toSec();
    ROS_WARN("elapsed time %f seconds " , elapsed_time);
    visualizeRRT();
    std::vector<Eigen::Vector3d> s_path = extract_shortest_path();
    //publish_path();
    elapsed_time = (time_in_expand - time_bef_expand).toSec();
    ROS_WARN("elapsed time after visualization %f seconds " , elapsed_time);
    ROS_INFO("point published");
    ROS_INFO("clearing kdtree"); 
    kd_clear(kd_tree);   
}

void LocalRrtPlanner::addNode(Eigen::Vector3d s_pt ,Eigen::Vector3d n_pt, NodePtr nearest_neighbour , NodePtr& new_node)
{
    
    double dist = sq_dist(s_pt , n_pt);
    float accum_g = nearest_neighbour->g;
    bool valid = true;
    Eigen::Vector3d goal_pt_col = apply_point_transform(goal_point , "color" , "map");

    float radius_p = radius_free_search(s_pt);

    float radius_sum = nearest_neighbour->radius + radius_p;

    if (dist < nearest_neighbour->radius ) return;

    if(radius_sum < dist)
    {
        Eigen::Vector3d direction = (s_pt - n_pt);
        direction.normalize();
        Eigen::Vector3d move_pt = n_pt;
        move_pt += nearest_neighbour->radius * direction;

        float radius = radius_free_search(move_pt);

        if (radius < 0.7) return;

        //ROS_INFO("radius of %f" ,  radius);
        accum_g += sq_dist(n_pt , move_pt);
        Eigen::Vector3d step_pt_map = apply_point_transform(move_pt , "map" , "color");
        float f = sq_dist(step_pt_map, goal_point);
        new_node = new Node(step_pt_map , radius ,accum_g, f);        
        new_node->valid = valid;
        new_node->parent_node = nearest_neighbour;
        nearest_neighbour->child_nodes.push_back(new_node);
        insertNode(new_node , move_pt);
        nodes.push_back(new_node);
        return;
    }

    else
    {
        float radius = radius_free_search(s_pt);
        if (radius < 0.7) return;
        Eigen::Vector3d sampled_pt_map  = apply_point_transform(s_pt , "map" , "color");
        float f = sq_dist(sampled_pt_map , goal_point);
        new_node = new Node(sampled_pt_map , safe_radius ,accum_g + dist, f);
        new_node->parent_node = nearest_neighbour;
        nearest_neighbour->child_nodes.push_back(new_node);
        insertNode(new_node , s_pt);
        nodes.push_back(new_node);

    }
}

std::vector<Eigen::Vector3d> LocalRrtPlanner::rrt_expansion(double time_limit)
{

    
    ROS_INFO("RRT expansion");
    Eigen::Vector3d n_pt;
    ros::Time time_bef_expand = ros::Time::now();
    ros::Time time_in_expand;
    
    double elapsed_time;
    Eigen::Vector3d cam_frame_pose(0 , 0 , 0.1);
    double arr[3] = {0 , 0 , 0.1};
    double current_rad = 0.25;
    root_pos = apply_point_transform(cam_frame_pose , "map" , "color");
    float f = sq_dist(root_pos , goal_point);
    root_node = new Node(root_pos , current_rad , 0.0 , f);
    nodes.push_back(root_node);
    kd_insert(kd_tree , arr ,(void*)root_node);
    best_path = root_node;
    int counter;

    for (counter = 0 ; counter < 600 ; counter++)
    {
        time_in_expand = ros::Time::now();

        if( (elapsed_time = (time_in_expand - time_bef_expand).toSec()) > time_limit ) {
            ROS_WARN("TIME OVERDUE %f seconds %u th sample" , elapsed_time , counter);
            break;
        }
        Eigen::Vector3d pt = sample_rand_point();

        publish_point(pt);
    
        NodePtr nn = nearestNeighbour(pt , n_pt);

        NodePtr new_node = NULL;
        addNode(pt , n_pt, nn , new_node);
        if (new_node == NULL) continue;
        if (new_node->parent_node == NULL)
        {
             ROS_WARN("null parent node %u" , counter );

        }
        
        
        
    }

    elapsed_time = (time_in_expand - time_bef_expand).toSec();
    
    ROS_WARN("elapsed time %f seconds " , elapsed_time);
    
    visualizeRRT();
    std::vector<Eigen::Vector3d> s_path = extract_shortest_path();
    elapsed_time = (time_in_expand - time_bef_expand).toSec();
    visualizePAth(s_path);
    ROS_WARN("elapsed time after visualization %f seconds %u path list size" , elapsed_time , s_path.size());
    ROS_INFO("point published");
    ROS_INFO("clearing kdtree"); 
    
    kd_clear(kd_tree); 

    return s_path;

}



std::vector<Eigen::Vector3d> LocalRrtPlanner::extract_shortest_path()
{

   std::vector<NodePtr> open_list, closed_list;
   open_list.push_back(root_node);
   double min_cost = inf, current;
   NodePtr record;
   while (open_list.size() > 0)
   {
    
    for (auto node : open_list)
    {

        current = node->g + node->f;

        if (min_cost > current)
        {
            min_cost = current;
            record = node;
        }
    }
    closed_list.push_back(record);
    open_list.clear();
    for (auto node : record->child_nodes)
    {

        open_list.push_back(node);
    }
    min_cost = inf;
    }
    std::vector<Eigen::Vector3d> path_list;
    for (auto node : closed_list)
    {

        path_list.push_back(node->pos);
    }
    return path_list;
}


std::vector<Eigen::Vector3d> LocalRrtPlanner::trace_path()
{

    std::vector<Eigen::Vector3d> path_list;

    NodePtr currnt = best_path;

    while(best_path != NULL)
    {

        path_list.insert(path_list.begin() , best_path->pos);


        best_path = best_path->parent_node;

        if (best_path == NULL)
        {

            ROS_INFO("null pointer");

        }

        ROS_INFO("path tracing"); 
    }

    return path_list;
}




inline double setYaw(Eigen::Vector3d pt1 , Eigen::Vector3d pt2)
{

    double tan_val = (pt2(1) - pt1(1))/(pt2(0) - pt1(0)) ;

    // return atan(tan_val);

    return atan2(pt2(1) - pt1(1) , pt2(0) - pt1(0));
}





void LocalRrtPlanner::publish_path(std::vector<Eigen::Vector3d> sh_path)
{

   

    mavros_msgs::Trajectory s_path;
    Eigen::Vector3d pt;
    pt(0) = iris_pose.pose.pose.position.x;
    pt(1) = iris_pose.pose.pose.position.y;
    pt(2) = iris_pose.pose.pose.position.z;

    double yaw = setYaw(pt , goal_point);

    if (sh_path.size() == 1 ||sh_path.size() == 2)
    {


        s_path.point_valid[0] = true;
        s_path.point_1.position.x   = pt(0);
        s_path.point_1.position.y   = pt(1);
        s_path.point_1.position.z   = 5.5;
        s_path.point_1.velocity.x   = VEL_x;
        s_path.point_1.velocity.y   = VEL_y;
        s_path.point_1.velocity.z   = VEL_z;
        s_path.point_1.acceleration_or_force.x = ACCL_x;
        s_path.point_1.acceleration_or_force.y = ACCL_y;
        s_path.point_1.acceleration_or_force.z = ACCL_z;
        s_path.point_1.yaw          = yaw;
        adaptedPathPub.publish(s_path);
        return;



    }


    double time_limit = 0;
    std::vector<double> time;
    ros::Time path_time;
    for (int i = 0 ; i < sh_path.size() - 1 ; i++ )
    {

        time_limit += sq_dist(sh_path[0] , sh_path[1]) / 3;
        double time_l =  sq_dist(sh_path[0] , sh_path[1]) / 3;
        time.push_back(time_l);

        
    }

    //for (auto pos :sh_path)
    for (int i = 0 ; i < sh_path.size() ; i++ )
    {
        if (i >= 5) break;

        auto pos = sh_path[i];
        s_path.point_valid[0] = true;
        s_path.point_1.position.x   = pos(0);
        s_path.point_1.position.y   = pos(1);
        s_path.point_1.position.z   = pos(2);
        // s_path.point_1.position.x = iris_pose.pose.pose.position.x;
        // s_path.point_1.position.y = iris_pose.pose.pose.position.y;
        // s_path.point_1.position.z = iris_pose.pose.pose.position.z;
        s_path.point_1.velocity.x   = VEL_x;
        s_path.point_1.velocity.y   = VEL_y;
        s_path.point_1.velocity.z   = VEL_z;
        s_path.point_1.acceleration_or_force.x = ACCL_x;
        s_path.point_1.acceleration_or_force.y = ACCL_y;
        s_path.point_1.acceleration_or_force.z = ACCL_z;
        // s_path.point_1.yaw          = this->desiredPath.point_1.yaw
        s_path.point_1.yaw          = yaw;
        s_path.point_2.yaw          = RAD(0);
        Eigen::Vector3d pt;
        pt(0) = iris_pose.pose.pose.position.x;
        pt(1) = iris_pose.pose.pose.position.y;
        pt(2) = iris_pose.pose.pose.position.z;
        // while (sq_dist( pt , pos ) >= 0.1){
        ros::Time start_time = ros::Time::now();
        //ros::Duration timeout(time[i]); // Timeout of 2 seconds
        ros::Duration timeout(0.001); // Timeout of 2 seconds
        while (ros::Time::now() - start_time < timeout){
            adaptedPathPub.publish(s_path);
            //ROS_INFO("pt->%f %f %f pos-> %f %f %f" , pt(0) , pt(1) , pt(2) , pos(0) , pos(1) , pos(2)); 
            //ROS_INFO("sq dist %f" ,sq_dist( pt , pos ) );

        }
        ROS_INFO("path reached"); 
            
    }
    // adaptedPathPub.publish(s_path);

}


void LocalRrtPlanner::visualizePAth(std::vector<Eigen::Vector3d> path)
{


    visualization_msgs::Marker points,line_list;
    points.header.frame_id =  line_list.header.frame_id = "/map";
    points.header.stamp =  line_list.header.stamp = ros::Time::now();
    points.ns =  line_list.ns = "points_and_lines";
    points.action  = line_list.action = visualization_msgs::Marker::ADD;

    points.id = 0;
    line_list.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.05;
    points.scale.y = 0.05;

   
    line_list.scale.x = 0.05;

    points.color.g = 1.0f;
    points.color.a = 1.0;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.color.g = 1.0;

    for (int i = 0 ; i < path.size() -1 ; i++)
    {

        geometry_msgs::Point p1 , p2;
        toPointmsg(path[i] , p1);
        toPointmsg(path[i + 1], p2);
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        points.points.push_back(p1);
        points.points.push_back(p2);

    }
    pVis.publish(line_list);
}


void LocalRrtPlanner::visualizeRRT()
{
    

    visualization_msgs::Marker points,line_list;
    points.header.frame_id =  line_list.header.frame_id = "/map";
    points.header.stamp =  line_list.header.stamp = ros::Time::now();
    points.ns =  line_list.ns = "points_and_lines";
    points.action  = line_list.action = visualization_msgs::Marker::ADD;

    points.id = 0;
    line_list.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.05;
    points.scale.y = 0.05;

   
    line_list.scale.x = 0.05;

    points.color.g = 1.0f;
    points.color.a = 1.0;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;


    std::vector<visualization_msgs::Marker> line_lists;
    NodePtr m_node = root_node;
    std::size_t conns = nodes.size();

    for (auto nodeptr : nodes)
    {
        for (auto child : nodeptr->child_nodes)
        {

            geometry_msgs::Point p1 , p2;
            toPointmsg(nodeptr->pos , p1);
            toPointmsg(child->pos, p2);
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
            points.points.push_back(p1);
            points.points.push_back(p2);

        }

    }

    rrtVis.publish(line_list);
    rrtVis.publish(points);
    nodes.clear();

}

inline int LocalRrtPlanner::checkNodeRelation( double dis, NodePtr node_1, NodePtr node_2 )
{


    int status;
    if( (dis + node_2->radius) == node_1->radius)
        status = 1;
    else if( (dis + 0.1) < 0.95 * (node_1->radius + node_2->radius) ) // && (dis > node_1->radius) && (dis > node_2->radius)
        status = -1;
    else
        status = 0;
    
    return status;
}



inline bool LocalRrtPlanner::isSuccessor(NodePtr curPtr, NodePtr nearPtr) // check if curPtr is father of nearPtr
{     
    NodePtr prePtr = nearPtr->parent_node;
    
    while(prePtr != NULL)
    {
        if(prePtr == curPtr) return true;
        else
          prePtr = prePtr->parent_node;
    }

    return false;
}


void LocalRrtPlanner::tree_rewire(Eigen::Vector3d s_pt ,Eigen::Vector3d n_pt ,  NodePtr nn , NodePtr& new_node)
{
    ROS_INFO("tree rewiring");
    NodePtr near_n = nn;
    NodePtr newPtr = new_node;
    float range = nn->radius * 2.0f;
    double pos[3] = {s_pt(0) , s_pt(1) , s_pt(2)};

    struct kdres* nearest_node_range = kd_nearest_range(kd_tree , pos , range);

    std::vector<NodePtr> nearPtrList;
    std::vector<Eigen::Vector3d> nearPtlist;

    bool isInvalid = false;
    while( !kd_res_end( nearest_node_range ) ) { // Pull all the nodes outside the result data structure from the kd-tree
                                         // And go through all the nearby vertex, check the relations, to see whether the newPtr should be discarded
             
            double pos[3];
            NodePtr nearPtr = (NodePtr)kd_res_item( nearest_node_range , pos );
            Eigen::Vector3d near_pt = {pos[0] , pos[1] , pos[2]};
            double dis = sq_dist( near_pt, s_pt );
            int   res = checkNodeRelation( dis, nearPtr, newPtr );
            nearPtr->rel_id  = res;    // temporary variables to record the local realtions with nearby nodes, to avoid repeated calculation in this two terms
            nearPtr->rel_dis = dis;

            if( res == 1 ){
                newPtr->valid = false;
                isInvalid = true;
                nearPtrList.push_back(nearPtr);
                nearPtlist.push_back(near_pt);
                break;
            }
            else{
                nearPtrList.push_back(nearPtr);
                nearPtlist.push_back(near_pt);
                kd_res_next( nearest_node_range );
            }
      }
    kd_res_free(nearest_node_range );
    ROS_INFO("tree rewiring2");
    //   if(isInvalid){
    //       for(auto nodeptr: nearPtrList){ // release all the temporary variables
    //           nodeptr->rel_id  = -2;
    //           nodeptr->rel_dis = -1.0;
    //       }
    //       return;
    //   }
      double min_cost = near_n->g +  sq_dist( n_pt, s_pt );
      newPtr->parent_node = near_n;
      newPtr->g = min_cost;
      near_n->child_nodes.push_back(newPtr);
      NodePtr lstParentPtr = near_n; // a pointer records the last best father of the new node
      
      std::vector<NodePtr> nearVertex;
      std::vector<Eigen::Vector3d> nearVertexpt;
      /*  Go through all the nearby vertex again, to deal with another two case. Note, better not mix this go through with the previous one  */
      ROS_INFO("tree rewiring3");
      for (int i = 0 ; i < nearPtrList.size() ; i++){
      
        NodePtr nearPtr = nearPtrList[i];
        Eigen::Vector3d pos = nearPtlist[i];
        // Choose the parent
        int res   = nearPtr->rel_id;
        double dis = nearPtr->rel_dis;
        double cost = nearPtr->g + dis;
        
        if( res == -1 ){
            if( cost < min_cost ){ // has a shorter path if new_ptr's father is the new node
                min_cost = cost;
                newPtr->parent_node = nearPtr;
                newPtr->g = min_cost;
                lstParentPtr->child_nodes.pop_back();
                lstParentPtr = nearPtr; // change a father, record the last father
                lstParentPtr->child_nodes.push_back(newPtr);
            }
            nearVertex.push_back(nearPtr);
            nearVertexpt.push_back(pos);
        }      
        nearPtr->rel_id  = -2;
        nearPtr->rel_dis = -1.0;
      }
      ROS_INFO("tree rewiring9");
      /*  Rewire the neighbors  */
      for(int i = 0; i < int(nearVertex.size()); i++ ){
          NodePtr nodeptr = nearVertex[i];
          Eigen::Vector3d npt = nearVertexpt[i];
          NodePtr nearPtr = nodeptr;
          if(nearPtr->valid == false) continue;
          ROS_INFO("tree rewiring10");
          double dis =sq_dist( npt, s_pt );
          double cost = dis + newPtr->g;
          
          if( cost < nearPtr->g){ // a near Node changed a father, delete this node from its parent's childer list
              
              if(isSuccessor(nearPtr, newPtr->parent_node)) 
                  continue; 
              
              if(nearPtr->parent_node == NULL ){
                  nearPtr->parent_node = newPtr;
                  nearPtr->g = cost;
              }
              else{
                  NodePtr lstNearParent = nearPtr->parent_node;
                  nearPtr->parent_node = newPtr;
                  nearPtr->g = cost;
                  
                  nearPtr->change = true; // use a temporary flag to indicate this pointer should be deleted
                  std::vector<NodePtr> child = lstNearParent->child_nodes;
                      
                  lstNearParent->child_nodes.clear();
                  for(auto ptr: child){
                      if( ptr->change == true ) continue;
                      else lstNearParent->child_nodes.push_back(ptr);
                  }
                  nearPtr->change = false; // release the flag after the delete  
              }
            
              newPtr->child_nodes.push_back(nearPtr);
          }
      }
}



LocalRrtPlanner::~LocalRrtPlanner()
{

    kd_free(kd_tree);
}


















