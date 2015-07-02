#!/usr/bin/env python


'''
    Nathaniel Goldfarb 
    6/29/15
    I&E scholar
    VideoRay research group
    
    Ths program moves the videoray model in rviz using 
    data from the /usble_pose node
    based on "Using urdf with robot_state_publisher" tutorial

'''

import rospy
import roslib
import math
import tf
import outlier_filter
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, TransformStamped
from matplotlib import matplotlib_fname
from mpl_toolkits.mplot3d import Axes3D
import sys
from matplotlib.pyplot import plot
from numpy import mean, std
import matplotlib as mpl

import numpy as np
import pandas as pd
import random
import matplotlib.pyplot as plt
plt.ion()
import matplotlib
mpl.rc("savefig", dpi=150)
import matplotlib.animation as animation
import time

#set the robot at the center


#//move the videoray using the data from the /pose_only node
def usbl_move(pos,current):

	print 'hello'
	broadcaster = tf.TransformBroadcaster()
	if outlier_filter.filter(pos.pose.position.x,pos.pose.position.y,current.position.z):
		current.position.x = pos.pose.position.x
		current.position.y = pos.pose.position.y

	
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "odom", "body" )
#move the videoray using the data from the /pose_only node	
def pose_move(pos,current):

	#pos.position.z is in kPa, has to be convereted to depth
	# P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
	z_real = (pos.position.z - 101.325)/9.81;

	#update the movement
	broadcaster = tf.TransformBroadcaster()
	current.orientation.x = pos.orientation.x
	current.orientation.y = pos.orientation.y
	current.orientation.z = pos.orientation.z
	current.orientation.w = pos.orientation.w
	current.position.z = z_real
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "odom", "body" )



if __name__ == '__main__':
	#set up the node
	rospy.init_node('move_unfiltered', anonymous=True)
	#make a broadcaster foir the tf frame
	broadcaster = tf.TransformBroadcaster()
	#make intilial values
	
	current = Pose()
	current.position.x = 0
	current.position.y = 0
	current.position.z = 0
	current.orientation.x = 0
	current.orientation.y = 0
	current.orientation.z = 0
	current.orientation.w = 0
	#send the tf frame
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "odom", "body" )

	#listen for information
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move,current)
	rospy.Subscriber("/pose_only", Pose, pose_move, current);
	rospy.spin()



