
#include <ros/ros.h>
#include <ros/time.h>
#include <deque>
#include <string>
#include <vector>
#include <utility>
#include <thread>
#include <random>
#include <math.h>
#include "kdtree.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <mavros_msgs/Trajectory.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include "node.h"




class LocalRrtPlanner{
public:
    LocalRrtPlanner(ros::NodeHandle &nh);
    ~LocalRrtPlanner(); 
    typedef std::pair<Eigen::Vector3d , double> Sphere;
    // std::pair<bool , std::vector<Eigen::Vector3d>> traced_pts;
    std::vector<pcl::PointXYZ> traced_pts;
    struct fov{
        double h_fov;
        double v_fov;
        double near_plane_depth;
        double far_plane_depth;

        fov(double h , double v , double n_p , double f_p){
            h_fov = h;
            v_fov = v;
            near_plane_depth = n_p;
            far_plane_depth = f_p;
        }
    }fov;
    //kdtree to store the nodes in the RRT for nearest neighbour calculations
    kdtree* kd_tree;
    //octree created from local pointcloud
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> local_octree;
    // pcl::search::KdTree<pcl::PointXYZ> local_octree;
     //store the converted pointcloud from the rgb-d camera
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered;  
    // pcl::PointCloud<pcl::PointXYZ> free_cloud , blocked_cloud;  
    pcl::PassThrough<pcl::PointXYZ> pass; 
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    //container to store the collision free reigons from the sampled nodes            
    std::vector<std::pair<Eigen::Vector3d , double>> free_space; 
    std::vector<int> pointIdxRadSearch;
    std::vector<float> pointRadSquaredDistance;
    std::vector<std::vector<Eigen::Vector3d>> valid_paths;
    std::vector<std::pair<Eigen::Vector3d , Eigen::Vector3d>> edgs;
    NodePtr root_node;
    NodePtr best_path;
        // std::vector<visualization_msgs::Marker> line_lists;
    std::vector<NodePtr> nodes;
    double step_dist , safe_radius;
    Eigen::Vector3d goal_point;
    Eigen::Vector3d root_pos;
    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_in);
    void publish_point(Eigen::Vector3d pt);
    void desiredTraj(const mavros_msgs::Trajectory::ConstPtr& _msg);

    nav_msgs::Odometry iris_pose;
    mavros_msgs::Trajectory desiredPath;
    mavros_msgs::Trajectory adaptedPath;
    ros::Subscriber iris_pose_sub;
    ros::Subscriber cloud_sub;
    ros::Subscriber waypoint_sub;
    ros::Publisher adaptedPathPub;
    ros::Publisher point_pub;
    ros::Publisher rrtVis;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> rand_z;
    std::uniform_real_distribution<double> rand_x , rand_y;

    Eigen::Vector3d sample_rand_point();
    Eigen::Vector3d apply_point_transform(Eigen::Vector3d pt, const std::string& target_frame , const std::string& source_frame);
    double radius_search(Eigen::Vector3d pt);
    float radius_free_search(Eigen::Vector3d pt);
    void add_free_space(Eigen::Vector3d pt , double radius);
    void insertNode(NodePtr node , Eigen::Vector3d pos);
    NodePtr nearestNeighbour(Eigen::Vector3d pt , Eigen::Vector3d& nearest_pt);

    void extendRRT(Eigen::Vector3d sampled_pt);
    // void extendRRTStar(Eigen::Vector3d sampled_pt);
    void extendRRTStar(Eigen::Vector3d sampled_pt , Eigen::Vector3d n_pt ,  NodePtr nn , NodePtr& new_node);
    void extendRRT(Eigen::Vector3d sampled_pt , Eigen::Vector3d& near_pt , NodePtr nn);
    void visualizeRRT();
    bool checkIntersect(Eigen::Vector3d pt_cam_frame);

    inline double get_dist(Eigen::Vector3d pt1 , Eigen::Vector3d pt2);
    void create_root_node();
    void build_local_rrt(double time_limit);
    void rrt_expansion(double time_limit);
    void tree_rewire(Eigen::Vector3d s_pt ,Eigen::Vector3d n_pt ,  NodePtr nn , NodePtr& new_node);
    inline bool isSuccessor(NodePtr curPtr, NodePtr nearPtr);
    inline int checkNodeRelation( double dis, NodePtr node_1, NodePtr node_2 );
    void addNode(Eigen::Vector3d s_pt ,Eigen::Vector3d n_pt, NodePtr nn , NodePtr& new_node);
    void extract_valid_paths(std::vector<std::vector<Eigen::Vector3d>> &valid_paths);
};


























































