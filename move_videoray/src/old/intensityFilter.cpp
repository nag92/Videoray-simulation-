
#include <ros/ros.h>
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


  ros::Publisher pub;

  void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud2)
  {
    float minD = 2;
    float maxD =  8;
    float r = 0;
    float lastR = 0;
    float intensity = 120 ;
    float currentI = 0;
    float lastI = 0;
    float threash = 10;
    bool havePoint = false;


   
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZI> disFilter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sorCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI min;
    pcl::PointXYZI max;
    pcl::PointXYZI point;

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
   
    //tf::StampedTransform transform;
    //listener.lookupTransform("body", "odom" , ros::Time(0), transform);

    
    //pcl::getMinMax3D(cloud, min,max);
    

    for (int i = 0; i<cloud.points.size(); i++ )
    {
      currentI  = cloud.points[i].intensity;
      r = sqrt( (cloud.points[i].y)*(cloud.points[i].y) + (cloud.points[i].x)*(cloud.points[i].x) + (cloud.points[i].z)*(cloud.points[i].z)  ); 
      if (  (currentI >= lastI  && r < maxD && r >= minD && currentI  > 55 && r >= lastR))
        {
          
          disFilter.push_back(cloud.points[i]);
         // point = cloud.points[i];
          lastI = currentI;
          lastR = r;
          havePoint = true;
          //std::cout<<"Hello";
        } 
        else
        {
          if (havePoint)
          {
            break;
          }
        }
 


        //std::cout<<"Bye";  
       std::cout<<"\n";
    }  

    if( havePoint)
    {
      disFilter.header.frame_id = cloud.header.frame_id;
      *sorCloud = disFilter;
      // build the filter
      outrem.setInputCloud(sorCloud);
      outrem.setRadiusSearch(1);
      outrem.setMinNeighborsInRadius (1);
      // apply filter
      outrem.filter (*cloudFiltered);
      //disFilter.push_back(point);
      //disFilter.push_back(point);
      pub.publish (cloudFiltered);
  }
     
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS 
    ros::init (argc, argv, "intensityFilter");
    ros::NodeHandle nh;
    


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/tritech_micron_node/sonarscan", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("intensityFilter", 1);

    // Spin
    ros::spin ();
  }
