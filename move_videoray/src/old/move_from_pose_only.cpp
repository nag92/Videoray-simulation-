
/*
    Nathaniel Goldfarb 
    6/25/15
    I&E scholar
    VideoRay research group
    
    Ths program moves the videoray model in rviz using 
    data from the /pose_ node
    based on "Using urdf with robot_state_publisher" tutorial 


*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::TransformStamped odom_trans;
sensor_msgs::JointState joint_state;
ros::Publisher joint_pub;
void move_videoray( const geometry_msgs::Pose& pos)
{

   
    //ROS_INFO("z: %f " ,pos.position.z);
    tf::TransformBroadcaster broadcaster;
    //odom_trans.header.stamp = ros::Time::now();
    //odom_trans.transform.translation.x =  pos.position.x; //cos(angle)*2;
    //odom_trans.transform.translation.y =  pos.position.y;//sin(angle)*2;
    odom_trans.transform.translation.z =  pos.position.z;
   // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.pose.orientation.w);
    odom_trans.transform.rotation.x = pos.orientation.x;
    odom_trans.transform.rotation.y = pos.orientation.y;
    odom_trans.transform.rotation.z = pos.orientation.z;
    odom_trans.transform.rotation.w = pos.orientation.w;
        //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);
    
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "move_from_pose_only");
     odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "body";
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);   
    ros::Rate loop_rate(1);
    ros::Subscriber sub = n.subscribe("/pose_only", 1, move_videoray);
  tf::TransformBroadcaster broadcaster;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 1;// pos.position.x; //cos(angle)*2;
    odom_trans.transform.translation.y =  1;// pos.position.y;//sin(angle)*2;
    odom_trans.transform.translation.z = 1;// pos.position.z;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);
   

    

    while (ros::ok()) 
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
