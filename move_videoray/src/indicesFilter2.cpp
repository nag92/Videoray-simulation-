#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"

typedef pcl::PointXYZ PointT;
ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud2)
{
  // All the objects needed
  float minD = 2;
    float maxD = 3;
    float r = 0;
    float intensity = 40;
 
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::PCLPointCloud2::Ptr  intensityFiltered (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::PointCloud<pcl::PointXYZ> sorted;
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud2);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (80, FLT_MAX);
  // pass.setFilterLimitsNegative (true);
  pass.filter (*intensityFiltered);
  pcl::fromPCLPointCloud2 (*intensityFiltered, cloud);

    
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

  
   *cloud_filtered = sorted;



  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (3);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (true);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
 
  // Remove the planar inliers, extract the rest
  extract.setNegative (false);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (1);
  seg.setRadiusLimits (0, 3);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
 
  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  pub.publish (cloud_cylinder);
 
}
int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "indicesFilter2");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/sonarscan", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("indicesFilter2", 1);
    // Spin
    ros::spin ();
  }