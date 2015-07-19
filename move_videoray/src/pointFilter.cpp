  #include <ros/ros.h>
  // PCL specific includes
  #include <pcl_conversions/pcl_conversions.h>
  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>
  #include <pcl/filters/passthrough.h>
  #include <pcl_conversions/pcl_conversions.h>
  #include <pcl/point_types.h>
  #include <pcl/PCLPointCloud2.h>
  #include <pcl/conversions.h>
  #include <pcl_ros/transforms.h>
  #include <pcl/point_types.h>
  #include <math.h> 
  #include "pcl_ros/point_cloud.h"
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
\
#include <pcl/surface/mls.h>

  ros::Publisher pub;

  void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud2)
  {
    float minD = 2;
    float maxD = 3;
    float r = 0;
    float intensity = 0;

        
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> sorted;
    pcl::PCLPointCloud2::Ptr  cloud_filtered (new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_f (new pcl::PointCloud<pcl::PointXYZ>);
  //Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
   

    pass.setInputCloud(cloud2);
    pass.setFilterFieldName("intensity"); 
    pass.setFilterLimits(intensity , FLT_MAX);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    pcl::fromPCLPointCloud2(*cloud_filtered, cloud);
    
    int i;
    
    for ( i = 0; i<cloud.points.size(); i++ )
    {

     r = sqrt( (cloud.points[i].y)*(cloud.points[i].y) + (cloud.points[i].x)*(cloud.points[i].x) ); 
     if (r<maxD && r > minD )
      {
        sorted.push_back(cloud.points[i]);
        //std::cout<<"Hello";
      } 
      //std::cout<<"Bye";
       std::cout<<"\n";
    }
   
    std::cout<<sorted.points.size();

   sorted.header.frame_id = cloud.header.frame_id;

  
   *sorted_f = sorted;
  // Output has the PointNormal type in order to store the normals calculated by MLS
  
  mls.setComputeNormals (true);

  // Set parameters

  mls.setInputCloud(sorted_f);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(10);
  // Reconstruct
  mls.process (mls_points);

  
  pub.publish (mls_points);

  }

  int main (int argc, char** argv)
  {
    // Initialize ROS 
    ros::init (argc, argv, "pointFilter");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/sonarscan", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("pointFilter", 1);

    // Spin
    ros::spin ();
  }
