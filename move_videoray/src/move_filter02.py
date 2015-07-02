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
import pandas as pd
import random
import matplotlib.pyplot as plt
#plt.ion()
import matplotlib
mpl.rc("savefig", dpi=150)
import matplotlib.animation as animation
import time

#set the robot at the center


#//move the videoray using the data from the /pose_only node
def usbl_move(pos,current):

	print 'hello'
	broadcaster = tf.TransformBroadcaster()
	if filter(pos.pose.position.x,pos.pose.position.y,current.position.z):
		current.position.x = pos.pose.position.x
		current.position.y = pos.pose.position.y

	
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

def filter(x,y,z):
    is_good = False
    print 'HI'
    # update the window
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
    if dis < mu + 1.5*sigma:
        #filt_x.append(x)
        #filt_y.append(y)
        #filt_depth.append(z)
        ax.scatter(x,y,z,color='b')
        is_good  = True
    else:
        #outlier_x.append(x)
        #outlier_y.append(y)
        #outlier_depth.append(z)
        ax.scatter(x,y,z,color='r')
    #update the plot
    elf.lines.set_xdata(xdata)
      self.lines.set_ydata(ydata)
    plt.draw()
    #ax.scatter(filt_x,filt_y,filt_depth,color='b')
    #ax.scatter(outlier_x,outlier_y,outlier_depth,color='r')

	
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
	figure, ax = plt.subplots()	
	ax.scatter(0,0,0,color='b')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	#ax.set_zlabel('Z')
	lines, = self.ax.plot([],[], 'o')
	ax.set_title('XY Outlier rejection \n with Mahalanobis distance and rolling mean3')
	n = 10
	#make some starting values
	#random distance
	md =[random.random() for _ in range(0, n)]
	#random points
	x_list = [random.random() for _ in range(0, n)]
	y_list =[random.random() for _ in range(0, n)]
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move,current)
	rospy.Subscriber("/pose_only", Pose, pose_move, current);
	rospy.spin()



