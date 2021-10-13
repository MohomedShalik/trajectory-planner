
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
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

     //store the converted pointcloud from the rgb-d camera
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  
    pcl::PointCloud<pcl::PointXYZ> free_cloud , blocked_cloud;   
    //container to store the collision free reigons from the sampled nodes            
    std::vector<std::pair<Eigen::Vector3d , double>> free_space; 
    std::vector<int> pointIdxRadSearch;
    std::vector<float> pointRadSquaredDistance;
    std::vector<std::vector<Eigen::Vector3d>> valid_paths;
    
    NodePtr root_node;
    NodePtr best_path;

    double step_dist , safe_radius;

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_in);
    void publish_point(Eigen::Vector3d pt);

    nav_msgs::Odometry iris_pose;
    ros::Subscriber iris_pose_sub;
    ros::Subscriber cloud_sub;
    ros::Subscriber desired_waypoint_sub;
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
    bool radius_free_search(Eigen::Vector3d pt);
    void add_free_space(Eigen::Vector3d pt , double radius);
    void insertNode(NodePtr node);
    NodePtr nearestNeighbour(Eigen::Vector3d pt);
    void extendRRT(Eigen::Vector3d sampled_pt);
    void visualizeRRT();
    bool checkIntersect(Eigen::Vector3d pt_cam_frame);

    inline double get_dist(Eigen::Vector3d pt1 , Eigen::Vector3d pt2);
    void create_root_node();
    void build_local_rrt();
    void extract_valid_paths(std::vector<std::vector<Eigen::Vector3d>> &valid_paths);
};


























































