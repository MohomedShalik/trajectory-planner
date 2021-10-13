#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <ros/time.h>
#include <deque>
#include <string>
#include <vector>

#include <thread>

#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/filter.h>
#include "localRrtPlanner.h"



#define NODE_NAME "trajectory_planner"


int main(int argc , char** argv)
{

   ros::init(argc, argv, NODE_NAME);

   ros::NodeHandle node;
   ros::Rate rate(30.0);

  
   LocalRrtPlanner localRrt(node);
   while(ros::ok()){
      ros::spinOnce();
      localRrt.build_local_rrt();
      rate.sleep();
   }

   return 0;
    


}



































 // ros::Subscriber localPCSub = node.subscribe<sensor_msgs::PointCloud2>("/realsense_camera/camera/depth_registered/points" , 10 , depth_callback);

