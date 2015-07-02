
/*
    Nathaniel Goldfarb 
    6/25/15
    I&E scholar
    VideoRay research group
    
    Ths program moves the videoray model in rviz using a loop
    to step the robot foward
    based on "Using urdf with robot_state_publisher" tutorial 


*/

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_straight");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;
    const double step = .1;
    double x=0,y=0;


    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "body";

    while (ros::ok()) {
        //update joint_state
        
        
        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x;// cos(angle)*2;
        odom_trans.transform.translation.y = y;//sin(angle)*2;
        odom_trans.transform.translation.z = 15;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(45);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0)
             tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) 
            hinc *= -1;
        swivel += degree;
        angle += degree/4;
        x+=step;
        y+=step;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
