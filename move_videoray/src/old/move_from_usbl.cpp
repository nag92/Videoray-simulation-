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

geometry_msgs::TransformStamped odom_trans;
sensor_msgs::JointState joint_state;
ros::Publisher joint_pub;
void move_videoray( const geometry_msgs::PoseStamped& pos)
{

   
    //ROS_INFO("z: %f " ,pos.position.z);
    tf::TransformBroadcaster broadcaster;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x =  pos.pose.position.x*100; //cos(angle)*2;
    odom_trans.transform.translation.y =  pos.pose.position.y*100;//sin(angle)*2;
    //odom_trans.transform.translation.z =  pos.pose.position.z*100;
    //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.pose.orientation.w);
    //odom_trans.transform.rotation.x = pos.pose.orientation.x;
    //odom_trans.transform.rotation.y = pos.pose.orientation.y;
    //odom_trans.transform.rotation.z = pos.pose.orientation.z;
    //odom_trans.transform.rotation.w = pos.pose.orientation.w;
        //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);
    
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "move_from_usbl");
     odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "body";
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);   
    ros::Rate loop_rate(1);
    ros::Subscriber sub = n.subscribe("/usbl_pose", 1, move_videoray);
    tf::TransformBroadcaster broadcaster;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0;// pos.position.x; //cos(angle)*2;
    odom_trans.transform.translation.y =  0;// pos.position.y;//sin(angle)*2;
    odom_trans.transform.translation.z = 0;// pos.position.z;
    odom_trans.transform.rotation.x = 0;
    odom_trans.transform.rotation.y = 0;
    odom_trans.transform.rotation.z = 0;
    odom_trans.transform.rotation.w = 0;

        //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);
   

    

    while (ros::ok()) 
    {
        //update joint_state
        
        ROS_INFO("foo");
        
        // update transform
        // (moving in a circle with radius=2)
        

        
        // This will adjust as needed per iteration
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
