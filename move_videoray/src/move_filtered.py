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

import numpy as np
import random
import matplotlib.pyplot as plt
plt.ion()
import matplotlib
mpl.rc("savefig", dpi=150)
import time

#filter stuff
#window size
n = 10
#make some starting values
#random distance
md =[random.random() for _ in range(0, n)]
#random points
x_list = [random.random() for _ in range(0, n)]
y_list =[random.random() for _ in range(0, n)]
#start = 1
#setup list to hold outliers and good points to plot

#set up graph

# This is a comment to test bitbucket


#set the robot at the center


#//move the videoray using the data from the /pose_only node
def usbl_move(pos,current):
	
	
	if usbl_move.start:
		usbl_move.fig = plt.figure()
		usbl_move.ax = usbl_move.fig.add_subplot(111, projection='3d')
		usbl_move.ax.set_xlabel('X')
		usbl_move.ax.set_ylabel('Y')
		usbl_move.ax.set_zlabel('Z')
		usbl_move.ax.set_title('XY Outlier rejection \n with Mahalanobis distance and rolling mean3')
		plt.show()
		usbl_move.start = 0

	color ='r'
	broadcaster = tf.TransformBroadcaster()

	if filter(pos.pose.position.x,pos.pose.position.y,current.position.z):
		current.position.x = pos.pose.position.x
		current.position.y = pos.pose.position.y
		color = 'b'
	
	usbl_move.ax.scatter(current.position.x,current.position.y,-1*current.position.z,color=color)
	plt.draw()
	
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "odom", "body" )
	#return current

#move the videoray using the data from the /pose_only node	
def pose_move(pos,current):

	#pos.position.z is in kPa, has to be convereted to depth
	# P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
	z_real = 1*(pos.position.z -101.325)/9.81;

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
	#return current

	
#call the fitle the date 
def filter(x,y,z):
    # update the window
    is_good = False
    x_list.append(x)
    y_list.append(y)
    x_list.pop(0)
    y_list.pop(0)
 
    #get the covariance matrix
    v = np.linalg.inv(np.cov(x_list,y_list,rowvar=0))
    #get the mean vector
    r_mean = mean(x_list), mean(y_list)  
    #subtract the mean vector from the point
    x_diff = np.array([i - r_mean[0] for i in x_list])
    y_diff = np.array([i - r_mean[1] for i in y_list])
    #combinded and transpose the x,y diff matrix
    diff_xy = np.transpose([x_diff, y_diff])
    # calculate the Mahalanobis distance
    dis = np.sqrt(np.dot(np.dot(np.transpose(diff_xy[n-1]),v),diff_xy[n-1]))
    # update the window 
    md.append( dis)
    md.pop(0)
    #find mean and standard standard deviation of the standard deviation list
    mu  = np.mean(md)
    sigma = np.std(md)
    #threshold to find if a outlier
    if dis < mu + .5*sigma:
        is_good =  True

	return is_good
	
	#plt.show()




if __name__ == '__main__':
	#set up the node
	rospy.init_node('move_filtered', anonymous=True)
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
	usbl_move.start  = 1 
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move,current)
	rospy.Subscriber("/pose_only", Pose, pose_move, current);
	rospy.spin()



