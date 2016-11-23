
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
  std::vector<pcl::PointCloud<pcl::PointXYZI> > bigCloud;
  int window = 1;
  double wd = .5;
    double wi = 1-wd;

 int calculateProp(std::vector<double> v, float maxD, float maxI, float distance, float intensity,pcl::PointCloud<pcl::PointXYZI> inputCloud);
 double getPHit(double point, double zStar,double zMax, double phi);
double getPShort(double point, double zStar,double lamda);
 double getPRand(double point,double zMax);
 double getPMax(double point, double zMax);
 std::vector<double> getZ(std::vector<float> intensity,std::vector<float> distance);
 double getDevation(std::vector<double> v, double mean);
  double getMean(std::vector<double> v);

void makeCloud(const pcl::PCLPointCloud2ConstPtr& inputCloud)
  {

    
    float minD = 2;
    float maxD =  8;
    float r = 0;
    float lastR = 0;
    float totalDistance = 0;
    float totalIntensity = 0;
    float intensity = 120 ;
    float currentI = 0;
    float lastI = 0;
    float threash = 10;
    int count = 0;
    bool havePoint = false;
    std::vector<float> distanceList;
    std::vector<float> intensityList;

    pcl::PointCloud<pcl::PointXYZI> tempcloud;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromPCLPointCloud2(*inputCloud, tempcloud);

    pcl::PointCloud<pcl::PointXYZI> disFilter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sorCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI min;
    pcl::PointXYZI max;
    pcl::PointXYZI point;

  

    if (bigCloud.size() > window )
    {
      bigCloud.erase(bigCloud.begin());
    }

    bigCloud.push_back(tempcloud);

    for( int i = 0; i < bigCloud.size(); i++)
    {
     
      for (int j = 0; j < bigCloud[i].points.size(); j++)
      {
        
       cloud.push_back(bigCloud[i].points[j]);
      }
    }
      cloud.header.frame_id = tempcloud.header.frame_id;
  

    for (int i = 0; i<cloud.points.size(); i++ )
    {
      currentI  = cloud.points[i].intensity;
      r = sqrt( (cloud.points[i].y)*(cloud.points[i].y) + (cloud.points[i].x)*(cloud.points[i].x) + (cloud.points[i].z)*(cloud.points[i].z)  ); 
      if (  (currentI >= lastI && r >= lastR ) && r < maxD && r >= minD && currentI  > 60 )
        {
          
          //disFilter.push_back(cloud.points[i]);
         // point = cloud.points[i];
          distanceList.push_back(r);
          intensityList.push_back(currentI);
          lastI = currentI;
          totalIntensity += currentI;
          totalDistance +=r;
          lastR = r;
          havePoint = true;
          count++;
          //std::cout<<"Hello";
        } 
        else
        {
          if (havePoint)
          {
            break;
          }
        }
    }  
  
   if(havePoint)
   {
    disFilter.header.frame_id = cloud.header.frame_id;
    //pub.publish (disFilter);
    calculateProp( getZ(intensityList,distanceList), lastR,lastI, totalDistance/count, totalIntensity/count,cloud );

   }

  }


  std::vector<double> getZ(std::vector<float> intensity,std::vector<float> distance)
  {

    

    int length = intensity.size();
    std::vector<double> z;
    for(int i = 0;i < length; i++ )
    {
      z.push_back(wd*distance[i]+wi*intensity[i]);

    }

    return z;
  }


  int calculateProp(std::vector<double> zList, float maxD, float maxI, float dis, float inten,pcl::PointCloud<pcl::PointXYZI> inputCloud)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.header.frame_id = inputCloud.header.frame_id;
   
    double tol = 0.99;
    double z_Hit = 0.5;
    double z_Max = 0.25;
    double z_Short = 0.125;
    double z_Rand = 0.25; 

    double pHit = 0;
    double pMax = 0;
    double pShort = 0;
    double pRand = 0; 

    double eHit = 0;
    double eMax = 0;
    double eShort = 0;
    double eRand = 0; 

    double z = 0;
    double r = 0;
    double intensity = 0;

    
    double q = 1;
    double p = 0;
    double nu = 0;
    double phi  = .001;
    double lamda = 1000;


    double zStar  =  wd*dis/maxD + wi*inten/maxI;
    // std::cout<< "zStar= " ;  
    //std::cout<< zStar << endl;
    double zMax  =   1; 
    int length = zList.size();
    phi = getDevation(zList,getMean(zList));
    std::cout<<phi;

    for (int i = 0; i<zList.size(); i++ )
    {
      
      std::cout<< z << endl;

      pHit = getPHit(zList[i],zStar,zMax,phi);
      pRand = getPRand(zList[i],zMax);
      pShort = getPShort(zList[i],zStar,lamda);
      pMax = getPMax(zList[i],zMax);

      /*
      nu = (pHit+pRand+pShort+pMax)^-1;

      eHit+=nu*pHit;
      eShort+=nu*pShort;
      eRand+=nu*pRand;
      eMax+=nu*pMax;

      z_Hit = eHit/length;
      z_Max = eMax/length;
      z_Short = eShort/length;
      z_Rand = eRand/length; 

      phi = (eHit*(z-zStar))/eHit;
      lamda = eShort/()
      */



       p = z_Hit*pHit+z_Short*pShort+z_Max*pMax+z_Rand*pRand;
       q = q*p;
       std::cout<< "P = ";
       std::cout<< p << endl;
       if(p > tol)
        cloud.push_back(inputCloud.points[i]);

    }


    pub.publish (cloud);


  }



  double getPHit(double point, double zStar,double zMax, double phi)
  {

    double det = 1.0/(2.*M_PI*phi);
    double expo = (-0.5/phi)*(point-zStar);

    if(point < zMax)
      return det*exp(expo);
    else
      return 0.0;

  }

  double getPShort(double point, double zStar, double lamda )
  {
   
    double nu = 1.0/ ( 1.0 - exp(-1*lamda*zStar));

    if(point <= zStar)
      return nu*lamda*exp(-1*lamda*zStar);
    else
      return 0;

  }

  double getPRand(double point,double zMax)
  {
    if(point < zMax )
      return 1.0/zMax;
    else
      return 0;

  }

  double getPMax(double point, double zMax)
  {

    if (point = zMax)
      return 1;
    else
      return 0;

  }

  double getMean(std::vector<double> v)
  {

    double sum = 0;
    for(int i = 0; i< v.size();i++)
      sum+=v[i];
    return sum/v.size();
  }

  double getDevation(std::vector<double> v, double mean)
  {
    double sum = 0.0;
    double temp =0.0;
    double var =0.0;
       
    for ( int j =0; j < v.size(); j++)
    {
      temp = pow(v[j] - mean,2);
      sum += temp;
    }
       
    return var = sum/(v.size() -2);
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS 
    ros::init (argc, argv, "propFilter");
    ros::NodeHandle nh;
    


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/tritech_micron_node/sonarscan", 1, makeCloud);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("propFilter", 1);

    // Spin
    ros::spin ();
  }
