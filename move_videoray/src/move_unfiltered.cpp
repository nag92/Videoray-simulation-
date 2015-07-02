/*
    Nathaniel Goldfarb 
    6/25/15
    I&E scholar
    VideoRay research group
    
    Ths program moves the videoray model in rviz using 
    data from the /usble_pose node
    based on "Using urdf with robot_state_publisher" tutorial 


*/


#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>



//global varibles
geometry_msgs::TransformStamped odom_trans;

ros::Publisher joint_pub;

//declarations
void usbl_move( const geometry_msgs::PoseStamped& pos);
void pose_move( const geometry_msgs::Pose& pos);

int main(int argc, char** argv) 
{   
    //set up 
    ros::init(argc, argv, "move_unfiltered");
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "body";
    ros::NodeHandle n;
    
    ros::Rate loop_rate(30);
    //define  the links
    ros::Subscriber usbl_sub = n.subscribe("/usbl_pose", 1, usbl_move);
    ros::Subscriber pose_sub = n.subscribe("/pose_only", 1, pose_move);
    tf::TransformBroadcaster broadcaster;
    odom_trans.header.stamp = ros::Time::now();

    broadcaster.sendTransform(odom_trans);
   

    ros::spin();


    return 0;
}
//move the videoray using the data from the /pose_only node
void pose_move( const geometry_msgs::Pose& pos)
{
    //pos.position.z is in kPa, has to be convereted to depth
    // P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
    float z_real = -1*(pos.position.z -101.325)/9.81;
   //update the movement
    tf::TransformBroadcaster broadcaster;
    odom_trans.transform.translation.z = z_real;
    odom_trans.transform.rotation.x = pos.orientation.x;
    odom_trans.transform.rotation.y = pos.orientation.y;
    odom_trans.transform.rotation.z = pos.orientation.z;
    odom_trans.transform.rotation.w = pos.orientation.w;
    //publish the movement
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);
}

//move the videoray using the /usbl_pose node
void usbl_move( const geometry_msgs::PoseStamped& pos)
{
    //update the movement
    tf::TransformBroadcaster broadcaster;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x =  pos.pose.position.x;
    odom_trans.transform.translation.y =  pos.pose.position.y;
    //publish the movement
   
    broadcaster.sendTransform(odom_trans);
    
}