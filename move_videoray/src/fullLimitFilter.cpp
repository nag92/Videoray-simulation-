  #include <ros/ros.h>
  // PCL specific includes
  #include <pcl_conversions/pcl_conversions.h>
  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>

  #include <pcl/filters/voxel_grid.h>
  #include <pcl/filters/passthrough.h>
  #include <pcl_conversions/pcl_conversions.h>
  #include <pcl/point_types.h>
  #include <pcl/PCLPointCloud2.h>
  #include <pcl/conversions.h>
  #include <pcl_ros/transforms.h>
  #include <pcl/point_types.h>
  #include <pcl/io/pcd_io.h>
  #include <pcl/kdtree/kdtree_flann.h>
  #include <pcl/surface/mls.h>

  ros::Publisher pub;

  void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
  {
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pcl::PCLPointCloud2::Ptr  cloud_intensity (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr  cloud_x (new pcl::PCLPointCloud2);
    //pcl::PCLPointCloud2::Ptr  cloud_y (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 cloud_filtered;
   

  
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits ( 1 , 15);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_x);

    pass.setInputCloud (cloud_x);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (1, 15);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_intensity);


    pass.setInputCloud (cloud_intensity);
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits (80 , FLT_MAX);
    //pass.setFilterLimitsNegative (true);
    pass.filter (cloud_filtered);

    
    pub.publish (cloud_intensity);
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "fullLimitFilter");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/sonarscan", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2> ("fullLimitFilter", 1);

    // Spin
    ros::spin ();
  }
