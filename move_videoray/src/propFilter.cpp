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

//declare functions
int calculateProp(pcl::PointCloud<pcl::PointXYZI> inPutCloud, float maxD, float maxI, float distance, float intensity);
double getPHit(double point, double zStar,double zMax, double phi);
double getPShort(double point, double zStar,double lamda);
double getPRand(double point,double zMax);
double getPMax(double point, double zMax);

//intilize global varibles
ros::Publisher pub;
std::vector<pcl::PointCloud<pcl::PointXYZI> > bigCloud;
int window = 1;

//function that is called when the point cloud message is recieved. 
void makeCloud(const pcl::PCLPointCloud2ConstPtr& inputCloud)
{

  //distance window
  float minD = 1;
  float maxD =  10;
  //hold the current distance
  float r = 0;
  //hols the last distance
  float lastR = 0;    
  //holds the total distance
  float totalDistance = 0;
  //holds the total intensity
  float totalIntensity = 0;
  //intensity threashold
  float intensity = 80 ;
  //hold current intensity
  float currentI = 0;
  //holds the last intensity 
  float lastI = 0;
  //hold the count of points
  int count = 0;
  //flag 
  bool havePoint = false;

  //point cloud varibles
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

  
  //exmaines if the the
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


  //filter the pointclouds
 for (int i = 0; i<cloud.points.size(); i++ )
 {
  //get the intensity of the point
  currentI  = cloud.points[i].intensity;
  //calcualted the distance for the point
  r = sqrt( (cloud.points[i].y)*(cloud.points[i].y) + (cloud.points[i].x)*(cloud.points[i].x) + (cloud.points[i].z)*(cloud.points[i].z)  ); 
  if (  (currentI >= lastI && r >= lastR ) && r < maxD && r >= minD && currentI   > intensity)
  {
    //add point to cloud
    disFilter.push_back(cloud.points[i]);
    // point = cloud.points[i]; 
    //save the information   
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
    //DONT USED
    //break if done 
    if (havePoint)
    {
           // break;
    }
  }
}  

//if a cloud as been contructed pass it to the probablity filter
  if(havePoint)
  {
    disFilter.header.frame_id = cloud.header.frame_id;
    //pub.publish (disFilter);
    calculateProp( disFilter, lastR,lastI, totalDistance/count, totalIntensity/count );

  }

}


/*
  determines the likilhood of the point being a hit 
  see chapter 6 of Probablist Robotics
  This method use the beam range finder algorithum to calculate the likilhood of the point being a hit.
  However a change was made to calculate the instric varibles on each beam using the tunig algorithum also in chapeter 6.
  
  The measurment varible is a wieghting of the distance and the intensity of the point, normilized to 1.
  then using a voxel filter to down sample the cloud. 
*/
int calculateProp(pcl::PointCloud<pcl::PointXYZI> inputCloud, float maxD, float maxI, float dis, float inten)
{
  //clouds to hold new pointclouds
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> sorted;
  pcl::VoxelGrid<pcl::PointXYZI> sor;
 
  //tolerance to plot the points
  double tol = .5;
  //weighting of the distance
  double wd = .6;
  //weighting of the intensity
  double wi = 1-wd;

  //starting weights 
  double z_Hit = 0.25;
  double z_Max = 0.25;
  double z_Short = 0.25;
  double z_Rand = 0.25; 

  //hold the probablities desities of the points
  double pHit = 0;
  double pMax = 0;
  double pShort = 0;
  double pRand = 0; 

  //need to calcuale the intrincisc parameters
  double eHit = 0;
  double eMax = 0;
  double eShort = 0;
  double eRand = 0; 

  //hold data of a point
  double z = 0;
  double r = 0;
  double intensity = 0;
  double q = 1;
  double p = 0;
  double nu = 0;
  double eHitSum = 0;
  double eHitSumZ = 0;
  double eShortSum = 0;
  double eShortSumZ = 0;
  double totalZ = 0;

  //starting values for calcualting the intristic parameters
  double phi  = .075; 
  double lamda = 2750;

  //get the referacne point
  double zStar  =  wd*dis/maxD + wi*inten/maxI;
  //max point
  double zMax  =   1; 
  //size of the data
  int length = inputCloud.points.size();

  cloud.header.frame_id = inputCloud.header.frame_id;


  //get the total value of the points
  for (int i = 0; i<inputCloud.points.size(); i++ )
  {

    r = sqrt( (inputCloud.points[i].y)*(inputCloud.points[i].y) + (inputCloud.points[i].x)*(inputCloud.points[i].x) + (inputCloud.points[i].z)*(inputCloud.points[i].z)  ); 
    intensity = inputCloud.points[i].intensity;
    z = (wd*r/maxD + wi*intensity/maxI);
    totalZ += z;

  }

  //calculate the intrinsic varibles
  for (int i = 0; i<inputCloud.points.size(); i++ )
  {
    //get the measument value
    r = sqrt( (inputCloud.points[i].y)*(inputCloud.points[i].y) + (inputCloud.points[i].x)*(inputCloud.points[i].x) + (inputCloud.points[i].z)*(inputCloud.points[i].z)  ); 
    intensity = inputCloud.points[i].intensity;
    z = (wd*r/maxD + wi*intensity/maxI);
    std::cout<<"z= ";

    std::cout<< z << endl;
    //get the probablities of the point
    pHit = getPHit(z,zStar,zMax,phi);
    pRand = getPRand(z,zMax);
    pShort = getPShort(z,zStar,lamda);
    pMax = getPMax(z,zMax);


    nu = 1/(pHit+pRand+pShort+pMax);

    eHit+=nu*pHit;
    eShort+=nu*pShort;
    eRand+=nu*pRand;
    eMax+=nu*pMax;

    z_Hit = eHit/abs(totalZ);
    z_Max = eMax/abs(totalZ);
    z_Short = eShort/abs(totalZ);
    z_Rand = eRand/abs(totalZ); 

    eHitSum+=eHit;
    eHitSumZ+=eHit*(z-zStar)*(z-zStar);
    eShortSum+=eShort;
    eShortSumZ+=eShort*z;

    lamda = eShortSum/eShortSumZ;
    phi = eHitSumZ/eHitSum;


  }

  //do the filtering
  for (int i = 0; i<inputCloud.points.size(); i++ )
  {

    r = sqrt( (inputCloud.points[i].y)*(inputCloud.points[i].y) + (inputCloud.points[i].x)*(inputCloud.points[i].x) + (inputCloud.points[i].z)*(inputCloud.points[i].z)  ); 
    intensity = inputCloud.points[i].intensity;
    z = (wd*r/maxD + wi*intensity/maxI);
    std::cout<<"z= ";

    std::cout<< z << endl;

    pHit = getPHit(z,zStar,zMax,phi);
    pRand = getPRand(z,zMax);
    pShort = getPShort(z,zStar,lamda);
    pMax = getPMax(z,zMax);

    p = z_Hit*pHit+z_Short*pShort+z_Max*pMax+z_Rand*pRand;
    q = q*p;
    std::cout<< "P = ";
    std::cout<< p << endl;

    //add points to the cloud
    if(p > tol)
     cloud.push_back(inputCloud.points[i]);

 }

 //do the voxel filtering
 *cloud_filtered  = cloud;
 sor.setInputCloud (cloud_filtered);
 sor.setLeafSize (.0100, .0100, .0100);
 sor.filter (sorted); 

  //publish the cloud
  pub.publish (sorted);
  //pub.publish (cloud);


}
/*
  The following methods find the prob. densities 
*/

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

int main (int argc, char** argv)
{
    // Initialize ROS 
  ros::init (argc, argv, "propFilter");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/videoray_sonar/sonarscan", 1, makeCloud);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("propFilter", 1);

    // Spin
  ros::spin ();
}
