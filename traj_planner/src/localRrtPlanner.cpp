

#include "localRrtPlanner.h"
#include "nav_msgs/Odometry.h"
#include "kdtree.h"
#include <iostream>
#include <pcl/io/ply_io.h>
#include <string>
#include <visualization_msgs/Marker.h>


#define RESOLUTION 0.05

#define to_pcl_type(x) (pcl::PointXYZ(x(0) , x(1) , x(2)))

void delete_node(void* node)
{
      delete static_cast<NodePtr>(node);
}


LocalRrtPlanner::LocalRrtPlanner(ros::NodeHandle &nh):cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                                    local_octree(RESOLUTION),step_dist(0.1) , safe_radius(0.6),
                                                    listener_(buffer_),
                                                    fov(30.5 * M_PI/180 , 22.5 * M_PI/180 , 0.1 , 8.0),
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
    adaptedPathPub  = nh.advertise<mavros_msgs::Trajectory>("traj_planner/trajectory/generated" , 20 );
    waypoint_sub  = nh.subscribe("mavros/trajectory/desired" , 10, &LocalRrtPlanner::desiredTraj , this);
    edgs.clear();

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


}


void LocalRrtPlanner::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_in,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    pcl::fromPCLPointCloud2(pcl_pc2,free_cloud);
    pcl::fromPCLPointCloud2(pcl_pc2,blocked_cloud);
    local_octree.deleteTree();
    local_octree.setInputCloud(cloud);
    local_octree.addPointsFromInputCloud();
}

Eigen::Vector3d LocalRrtPlanner::sample_rand_point()
{

    double r_z = rand_z(generator);
    double width_x =  r_z * tan(30   * M_PI /180.0);
    double width_y =  r_z * tan(22.5 * M_PI/180.0);
    double r_x = rand_x(generator) * width_x;
    double r_y = rand_y(generator) * width_y;
    Eigen::Vector3d rand_point(r_x , r_y , r_z);
    return rand_point;
}

Eigen::Vector3d LocalRrtPlanner::apply_point_transform(Eigen::Vector3d pt, const std::string& target_frame , const std::string& source_frame)
{
    Eigen::Vector3d rand_point;

    tf2::Vector3 point(pt(0), pt(1) , pt(2));
    while(!buffer_.canTransform("color" , "map" , ros::Time::now() , ros::Duration(1,0))){
        std::cout << "transform unavailable" << std::endl;   
        continue;
    }

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
        double max = *std::max_element(pointRadSquaredDistance.begin(), pointRadSquaredDistance.end());
        std::cout << "radius max is " << max << std::endl;
        return max;
    }   
    else{
        std::cout << "no points within safe distance" << std::endl;
        return safe_radius;
    }
}

bool LocalRrtPlanner::radius_free_search(Eigen::Vector3d pt)
{

    pcl::PointXYZ searchPoint;
    searchPoint.x = pt(0);
    searchPoint.y = pt(1);
    searchPoint.z = pt(2);
    pointIdxRadSearch.clear();
    pointRadSquaredDistance.clear();
    if ( (local_octree.radiusSearch (searchPoint,safe_radius ,pointIdxRadSearch, pointRadSquaredDistance))> 0){
        
        return false;
    }   
    else{
        std::cout << "no points within safe distance" << std::endl;
        return true;
    }

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
    pub_point.header.frame_id = "map";
    pub_point.point.x = pt(0);
    pub_point.point.y = pt(1);
    pub_point.point.z = pt(2);
    point_pub.publish(pub_point);
}


void LocalRrtPlanner::insertNode(NodePtr node)
{
    double arr[3] = {node->pos(0) ,node->pos(1) ,node->pos(2)};
    kd_insert(kd_tree , arr ,(void*)node);
}


void LocalRrtPlanner::create_root_node()
{
    
    Eigen::Vector3d cam_frame_pose(0 , 0 , 0.1);
    double arr[3] = {0 , 0 , 0.1};
    double current_rad = 0.5;
    std::cout << "creatng shared ptr\n";
    //NodePtr nroot_node = std::make_shared<Node>(cam_frame_pose , current_rad , 0 , 0);
    root_node = new Node(cam_frame_pose , current_rad , 0 , 0);
    //root_node = nroot_node;
    kd_insert(kd_tree , arr ,(void*)root_node);
}

inline double sq_dist(Eigen::Vector3d pt1 , Eigen::Vector3d pt2)
{

    double i = (pt2 - pt1).squaredNorm();
    return i;

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




NodePtr LocalRrtPlanner::nearestNeighbour(Eigen::Vector3d pt)
{

    double arr[3] = {pt(0) , pt(1) , pt(2)};
    struct kdres* nearest_node = kd_nearest(kd_tree , arr);
    NodePtr near((Node*)(kd_res_item(nearest_node, NULL)));
    kd_res_free(nearest_node);
    return near;
    
}

void LocalRrtPlanner::extendRRT(Eigen::Vector3d sampled_pt)
{
    NodePtr nn = nearestNeighbour(sampled_pt);

    if (sq_dist(nn->pos , sampled_pt) <= step_dist)
    {
        traced_pts.clear();
        if (radius_free_search(sampled_pt) == false)
                return;
        NodePtr new_node(new Node(sampled_pt , safe_radius , 0 , 0));
        new_node->parent_node = nn;
        nn->child_nodes.push_back(new_node);
        traced_pts.push_back(to_pcl_type(sampled_pt));
        edgs.push_back(std::make_pair(nn->pos , sampled_pt));
        
    }
    else
    {
        
        Eigen::Vector3d direction = (sampled_pt - nn->pos);
        direction.normalize();
        Eigen::Vector3d st_pt = nn->pos;
        double dist = sq_dist(st_pt , sampled_pt);
        
        double ray_dist = 0;
        Eigen::Vector3d move_pt = nn->pos;
        traced_pts.clear();
        while(ray_dist <= dist)
        {
            move_pt += step_dist * direction;
            traced_pts.push_back(to_pcl_type(sampled_pt));
            if (radius_free_search(move_pt) == false)
                return;
            NodePtr new_node(new Node(move_pt , safe_radius , 0 , 0));
            new_node->valid = true;
            new_node->parent_node = nn;
            nn->child_nodes.push_back(new_node);
            insertNode(new_node);
            edgs.push_back(std::make_pair(nn->pos , move_pt));
            nn = new_node;
            ray_dist = sq_dist(st_pt , move_pt);
        }

    }
}

inline void toPointmsg(Eigen::Vector3d pt , geometry_msgs::Point& p)
{
    p.x = pt(0);
    p.y = pt(1);
    p.z = pt(2);

}

void LocalRrtPlanner::visualizeRRT()
{

    visualization_msgs::Marker points,line_list;
    points.header.frame_id =  line_list.header.frame_id = "/color";
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

    // std::vector<visualization_msgs::Marker> line_lists;

    NodePtr m_node = root_node;
    std::cout << "tree visulizer for " << edgs.size() << " tree edges\n";
    for (int i = 0 ; i < edgs.size() ; i++)
    {
        geometry_msgs::Point p1 , p2;
        toPointmsg(edgs[i].first , p1);
        toPointmsg(edgs[i].second, p2);
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        points.points.push_back(p1);
        points.points.push_back(p2);
    }
    rrtVis.publish(line_list);
    //rrtVis.publish(points);

}

void LocalRrtPlanner::build_local_rrt()
{
    
   
    edgs.clear();
    std::cout << "build local rrt\n";
    create_root_node();
    for (int i = 0 ; i < 500 ; i++)
    {
        Eigen::Vector3d pt = sample_rand_point();
        double max = radius_search(pt);
        std::cout <<  "radius of "<< max << std::endl;
        pcl::PointXYZ searchPoint(pt(0) , pt(1) , pt(2));
        extendRRT(pt);   
    }
    visualizeRRT();
    std::cout << "point published" << std::endl;
    std::cout << "clearing kdtree" << std::endl;
    kd_clear(kd_tree);
    
}


LocalRrtPlanner::~LocalRrtPlanner()
{

    kd_free(kd_tree);
}






































