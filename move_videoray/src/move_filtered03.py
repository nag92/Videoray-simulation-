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
#import outlier_filter
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, TransformStamped
from matplotlib import matplotlib_fname
from mpl_toolkits.mplot3d import Axes3D
import sys
from matplotlib.pyplot import plot
from numpy import mean, std
import matplotlib as mpl
import make_graph
import outlier_filter
import kalman_filter

import numpy as np
import random
import matplotlib.pyplot as plt
plt.ion()
import matplotlib
mpl.rc("savefig", dpi=150)
import time



#//move the videoray using the data from the /pose_only node
def usbl_move(pos,current):

	broadcaster = tf.TransformBroadcaster()
	if usbl_move.start:
		dt = .93

		loc = np.matrix([[0],#v_x
						 [0],#v_y
						 [0],#v_z
						 [pos.pose.position.x],#x
						 [pos.pose.position.y],#y
						 [current.position.z]])#z
	
		A = np.matrix([[1,  0,  0,  0, 0, 0,],
				   	   [0,  1,  0,  0, 0, 0,],
				   	   [0,  0,  1,  0, 0, 0,],
				       [dt, 0,  0,  1, 0, 0,],
				       [0,  dt, 0,  0, 1, 0,],
				       [0,  0,  dt, 0, 0, 1, ]])
		B = np.matrix([0])
		C = np.eye(loc.shape[0])
		Q = np.eye(loc.shape[0])*0.05
		R = np.eye(loc.shape[0])*5000
		P = np.eye(loc.shape[0])
		U = np.matrix( [[0]])
		Z = np.matrix( [[0],[0],[0],[0],[0],[0] ])
		
		usbl_move.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,loc)
		usbl_move.start = 0

	
	if outlier_filter.outlier(pos.pose.position.x,pos.pose.position.y,current.position.z):
		#update.ax.scatter(pos.position.x,pos.position.y,-1*current.position.z,color='b')
		current.position.x = pos.pose.position.x
		current.position.y = pos.pose.position.y
		#current.position.z = pos.pose.position.z
		#update('b')
		Z = np.matrix( [[0],[0],[0],[pos.pose.position.x],[pos.pose.position.y],[current.position.z] ])
		U = np.matrix( [[0]])
		usbl_move.kalman.move(U,Z)
		kalman_pos = usbl_move.kalman.getState()
		current.position.y = kalman_pos[3]
		current.position.x = kalman_pos[4]
		#current.position.z = kalman_pos[5]
		#update.ax.scatter(kalman_pos[3], kalman_pos[4],kalman_pos[5],color='g')
		#update('g')
	
	# 	broadcaster.sendTransform( (0,0,0), 
	# 							(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
	# 								rospy.Time.now(), "odom", "body" )
	
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									rospy.Time.now(), "body", "odom" )
	

#move the videoray using the data from the /pose_only node	
def pose_move(pos):

	#pos.position.z is in kPa, has to be convereted to depth
	# P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
	z_real = -(pos.position.z -101.325)/9.81
	current.position.z = z_real
	broadcaster = tf.TransformBroadcaster()
	if pose_move.start:

		dt = .04

		loc = np.matrix([[0],#v_x
						[0],#v_y
						[0],#v_z
						[0],#v_w
						[pos.orientation.x],#x
						[pos.orientation.y],#y
						[pos.orientation.z],#z
						[pos.orientation.w]])#w
	
		A = np.matrix([[1,  0,  0,  0, 0, 0, 0, 0,],
				   	   [0,  1,  0,  0, 0, 0, 0, 0,],
				   	   [0,  0,  1,  0, 0, 0, 0, 0,],
				       [0,  0,  0,  1, 0, 0, 0, 0,],
				       [dt, 0,  0,  0, 1, 0, 0, 0,],
				       [0,  dt, 0,  0, 0, 1, 0, 0,],
				       [0,  0,  dt, 0, 0, 0, 1, 0,],
				       [0,  0,  0,  dt, 0, 0,0, 1,]])
		B = np.matrix([0])
		C = np.eye(loc.shape[0])
		Q = np.eye(loc.shape[0])*.05
		R = np.eye(loc.shape[0])*500000
		P = np.eye(loc.shape[0])
		U = np.matrix( [[0]])
		Z = np.matrix( [[0],[0],[0],[0],[0],[0],[0],[0] ])
	
		pose_move.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,loc)
		pose_move.start = 0


	Z = np.matrix( [[0],[0],[0],[0],[pos.orientation.x],[pos.orientation.y],[pos.orientation.z],[pos.orientation.w] ])
	U = np.matrix( [[0]])
	pose_move.kalman.move(U,Z)
	
	kalman_pos = pose_move.kalman.getState()
	current.orientation.x = kalman_pos[5]
	current.orientation.y = kalman_pos[4]*-1
	current.orientation.z = kalman_pos[6]*-1
	current.orientation.w = kalman_pos[7]
	#update('b')

	# broadcaster.sendTransform( (0,0,0), 
	# 							(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
	# 								rospy.Time.now(), "odom", "body" )
	
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,
								current.orientation.w),
								rospy.Time.now(), "body", "odom" )

def update(c):

	broadcaster = tf.TransformBroadcaster()
	
	if update.start:
		update.plt,update.ax  = make_graph.makeGraph()
		update.start = 0

	
	update.ax.scatter(current.position.x,current.position.y,current.position.z,color=c)
	
	#update.plt.draw();

	
	
	#broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
	#							(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
	#								 rospy.Time.now(), "odom", "body" )


if __name__ == '__main__':
	#set up the node
	rospy.init_node('move_filtered03', anonymous=True)
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
	update.start  = 1 
	usbl_move.start = 1
	pose_move.start = 1
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move,current)
	rospy.Subscriber("/pose_only", Pose, pose_move);
	rospy.spin()



