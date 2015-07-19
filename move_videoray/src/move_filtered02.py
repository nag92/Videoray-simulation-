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
	
	
	update(pos.pose)

	
	#return current

#move the videoray using the data from the /pose_only node	
def pose_move(pos):

	#pos.position.z is in kPa, has to be convereted to depth
	# P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
	z_real = (pos.position.z -101.325)/9.81;

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


def update(pos):
	
	if update.start:
		update.plt,update.ax  = make_graph.makeGraph()
		dt = 1

		loc = np.matrix([[0],#v_x
						 [0],#v_y
						 [0],#v_z
						 [pos.position.x],#x
						 [pos.position.y],#y
						 [pos.position.z]])#z
	
		A = np.matrix([[1,  0,  0,  0, 0, 0,],
				   	   [0,  1,  0,  0, 0, 0,],
				   	   [0,  0,  1,  0, 0, 0,],
				       [dt, 0,  0,  1, 0, 0,],
				       [0,  dt, 0,  0, 1, 0,],
				       [0,  0,  dt, 0, 0, 1, ]])
		B = np.matrix([0])
		C = np.eye(loc.shape[0])
		Q = np.eye(loc.shape[0])*10
		R = np.eye(loc.shape[0])*30
		P = np.eye(loc.shape[0])
		U = np.matrix( [[0]])
		Z = np.matrix( [[0],[0],[0],[0],[0],[0] ])
	
		update.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,loc)
		update.start = 0

	
	broadcaster = tf.TransformBroadcaster()

	if outlier_filter.outlier(pos.position.x,pos.position.y,-1*current.position.z):


		update.ax.scatter(pos.position.x,pos.position.y,-1*current.position.z,color='b')
		Z = np.matrix( [[0],[0],[0],[pos.position.x],[pos.position.y],[-1*current.position.z] ])
		U = np.matrix( [[0]])
		update.kalman.move(U,Z)
		kalman_pos = update.kalman.getState()
		current.position.x = kalman_pos[3]
		current.position.y = kalman_pos[4]
		current.position.z = kalman_pos[5]
		update.ax.scatter(kalman_pos[3], kalman_pos[4],kalman_pos[5],color='g')
		broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "odom", "body" )

	else:	
		update.ax.scatter(pos.position.x,pos.position.y,-1*pos.position.z,color='r')
	plt.draw()
	
	


if __name__ == '__main__':
	#set up the node
	rospy.init_node('move_filtered02', anonymous=True)
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
	update.start  = 1 
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move,current)
	rospy.Subscriber("/pose_only", Pose, pose_move);
	rospy.spin()



