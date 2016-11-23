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
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, TransformStamped
import sys
from numpy import mean, std
import numpy as np
import random
import time
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import kalman_filter
import md_filter
import poly_filter

def usbl_update(pos):

	broadcaster = tf.TransformBroadcaster()

	if( usbl_update.md_filter.update( [ pos.pose.position.x, pos.pose.position.y ]) ):
		if( usbl_update.poly_x.update(pos.pose.position.x) and usbl_update.poly_y.update(pos.pose.position.y)):
			Z = np.matrix( [[0],[0],[0],[pos.pose.position.x],[pos.pose.position.y],[current.position.z] ])
			U = np.matrix( [[0]])
			usbl_update.kalman.move(U,Z)
			kalman_pos = usbl_update.kalman.getState()
			current.position.y = kalman_pos[3]
			current.position.x = kalman_pos[4]

	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									rospy.Time.now(), "body", "odom" )

def imu_update(pos):

	#pos.position.z is in kPa, has to be convereted to depth
	# P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
	z_real = pos.pose.position.z# -(1000*pos.position.z-101.325)/9.81 
	current.position.z = z_real
	broadcaster = tf.TransformBroadcaster()
	
	(roll, pitch, yaw) = euler_from_quaternion([ pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z,pos.pose.orientation.w ] )

	Z = np.matrix([ [0], [0], [0], [roll], [pitch], [yaw] ])
	U = np.matrix([[0]])
	
	imu_update.kalman.move(U,Z)

	
	kalman_pos = imu_update.kalman.getState()

	(x, y, z, w) = quaternion_from_euler( kalman_pos[3],kalman_pos[4],kalman_pos[5] )
	current.orientation.x = x
	current.orientation.y = y
	current.orientation.z = z
	current.orientation.w = w
	

	#update the tf
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,
								current.orientation.w),
								rospy.Time.now(), "body", "odom" )



def usbl_setup():
	
	dt = 1
	q = 5000
	r = .5000

	loc = np.matrix([[0],#v_x
					 [0],#v_y
					 [0],#v_z
					 [0],#x
					 [0],#y
					 [0]])#z

	A = np.matrix([[1,  0,  0,  0, 0, 0,],
			   	   [0,  1,  0,  0, 0, 0,],
			   	   [0,  0,  1,  0, 0, 0,],
			       [dt, 0,  0,  1, 0, 0,],
			       [0,  dt, 0,  0, 1, 0,],
			       [0,  0,  dt, 0, 0, 1, ]])

	B = np.matrix([0])
	C = np.eye(loc.shape[0])
	Q = np.eye(loc.shape[0])*q
	R = np.eye(loc.shape[0])*r
	P = np.eye(loc.shape[0])
	U = np.matrix( [[0]])
	Z = np.matrix( [[0],[0],[0],[0],[0],[0] ])

	kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,loc)

	prop = 2.77
	n = 10
	md = md_filter.md_filter(2,prop, n, [0, 1 ])

	poly_x = poly_filter.poly_filter(100,4)
	poly_y = poly_filter.poly_filter(100,4)
	
	
	return (kalman, md, poly_x, poly_y)

def imu_setup():
	
	dt = .5
	r = .50
	q = 50

	pos = np.matrix([[0],  # v_x
	   				 [0],  # v_y
					 [0],  # v_z
					 [0],  # x
					 [0],  # y
					 [0]])  # z

	A = np.matrix([[1, 0, 0, 0,  0, 0, ],
			       [0, 1, 0, 0,  0, 0, ],
				   [0, 0, 1, 0,  0, 0, ],
				   [dt, 0, 0, 1, 0, 0, ],
				   [0, dt, 0, 0, 1, 0, ],
				   [0, 0, dt, 0, 0, 1, ]])

	B = np.matrix([0])
	C = np.eye(pos.shape[0])
	Q = np.eye(pos.shape[0]) * q
	R = np.eye(pos.shape[0]) * r
	P = np.eye(pos.shape[0])
	U = np.matrix([[0]])
	Z = np.matrix([[0], [0], [0], [0], [0], [0] ])

	return kalman_filter.kalman_filter(A, B, C, Q, P, R, pos)
		

if __name__ == '__main__':
	#set up the node
	rospy.init_node('pose_filter', anonymous=True)
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
									 rospy.Time.now(), "body", "odom" )

	#listen for information
	
	#pub = rospy.Publisher("newPose", Pose)
	(usbl_update.kalman,usbl_update.md_filter,usbl_update.poly_x,usbl_update.poly_y) =  usbl_setup()
	imu_update.kalman = imu_setup()
	rospy.Subscriber("/videoray_usbl/usbl_pose", PoseStamped, usbl_update)
	rospy.Subscriber("/videoray_control/pose", PoseStamped, imu_update);
	rospy.spin()