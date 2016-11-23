//This program filters the map using algorithums in Probablist Robotics chapter 6
#include <ros/ros.h>
#include <vector>
  // PCL specific includes
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <math.h> 
#include "pcl_ros/point_cloud.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/radius_outlier_removal.h>

#define _USE_MATH_DEFINES

using namespace std;

ros::Publisher pub;
void makeCloud(const pcl::PCLPointCloud2ConstPtr& inputCloud)
{

  pcl::PointCloud<pcl::PointXYZ> tempcloud;
  pcl::fromPCLPointCloud2(*inputCloud, tempcloud);
  
  pub.publish (tempcloud);
  


}
/*
  The following methods find the prob. densities 
*/


int main (int argc, char** argv)
{
    // Initialize ROS 
  ros::init (argc, argv, "noFilter");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/tritech_micron_node/sonarscan", 1, makeCloud);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("noFilter", 1);

    // Spin
  ros::spin ();
}
