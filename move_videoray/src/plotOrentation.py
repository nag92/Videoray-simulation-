# '''
# Created on Jul 10, 2015

# @author: nathaniel, I&E summer 2015
# This program use a kalman filer to smooth the Quaternion of the videoray

# '''

import kalman_filter
import matplotlib as mpl
from numpy import arctan2, arcsin
mpl.rc("savefig", dpi=150)
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
#import rospy
from tf.transformations import euler_from_quaternion

mpl.rcParams.update({'font.size': 18})
#import and save data from a CSV file

df = pd.DataFrame.from_csv('/home/nathaniel/KingsPointData/15_21_23/pose_only.csv', index_col=None)
# print df
my_x = df['field.orientation.x'].values.tolist()
my_y = df['field.orientation.y'].values.tolist()
my_z = df['field.orientation.z'].values.tolist()
my_w = df['field.orientation.w'].values.tolist()
t = range(0, len(my_x))
print len(t)
print len(my_w)
new_x = []
new_y = []
new_z = []
new_w = []
roll =[]
pitch = []
yaw = []

#kalman parameter set up

#wieght of the covarence matrixes
q = .05#0.05
r =  500000#5456



dt = .001
loc = np.matrix([[0],  # v_x
				[0],  # v_y
				[0],  # v_z
				[0],  # v_w
				[my_x[0]],  # x
				[my_y[0]],  # y
				[my_z[0]],  # z
				[my_w[0]]])  # w
A = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, ],
			  [0, 1, 0, 0, 0, 0, 0, 0, ],
			  [0, 0, 1, 0, 0, 0, 0, 0, ],
			  [0, 0, 0, 1, 0, 0, 0, 0, ],
			  [dt, 0, 0, 0, 1, 0, 0, 0, ],
			  [0, dt, 0, 0, 0, 1, 0, 0, ],
			  [0, 0, dt, 0, 0, 0, 1, 0, ],
			  [0, 0, 0, dt, 0, 0, 0, 1, ]])
B = np.matrix([0])
C = np.eye(loc.shape[0])
Q = np.eye(loc.shape[0]) * q
R = np.eye(loc.shape[0]) * r
P = np.eye(loc.shape[0])
U = np.matrix([[0]])
Z = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0] ])
	
kalman = kalman_filter.kalman_filter(A, B, C, Q, P, R, loc)


#loop through data and apply Kalman filter
for i in xrange(len(t)):
    kalman.move(U, Z)
    temp = kalman.getState()
    new_x.append(temp[5])
    new_y.append(temp[4]*-1)
    new_z.append(temp[6]*-1)
    new_w.append(temp[7])
    Z = np.matrix([[0], [0], [0], [0], [my_x[i]], [my_y[i]], [my_z[i]], [my_w[i]] ])
    U = np.matrix([[0]])
    



for i in xrange(len(new_x)):
	q0 = new_w[i]
	q1 = new_x[i]
	q2 = new_y[i]
	q3 = new_z[i]
	roll.append((180/3.14)*arctan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2)))
	pitch.append((180/3.14)*arcsin(2*(q0*q2 - q3*q1)))
	yaw.append((180/3.14)*arctan2(2*(q0*q3+q1*q2), 1-2*(q2*q2 + q3*q3)))
	
	
# #print raw and filtered data
# 
# f1 = plt.figure()
# ax1 = f1.add_subplot(111)
# plt.axis([0, len(t), -1.2, 1.2])
# plt.xlabel('Time step')
# plt.ylabel('Quaternion angle')
# #plt.title('Raw')
# ax1.scatter(t, my_y, color='r',label='x')
# ax1.scatter(t, my_x, color='b',label='y')
# 
# ax1.scatter(t, my_z, color='g',label='z')
# ax1.scatter(t, my_w, color='y',label='w')
# legend = ax1.legend(loc='lower center', shadow=True)
# frame = legend.get_frame()
# for label in legend.get_texts():
#     label.set_fontsize('large')
# 
# for label in legend.get_lines():
#     label.set_linewidth(1.5)  # the legend line width
# plt.show()
# 
# 
# 	
# f2 = plt.figure()
# ax2 = f2.add_subplot(111)
# plt.axis([0, len(t), -1.2,1.2 ])
# plt.xlabel('Time step')
# plt.ylabel('Quaternion angle')
# #plt.title('Kalman Filter')
# #ax2.text(10000,-1, 'q = ' + str(q) + '\n' + 'r = ' + str(r),fontsize=20, bbox={'facecolor':'red', 'alpha':0.5, 'pad':10})
# ax2.scatter(t, new_y, color='r',label='x')
# ax2.scatter(t, new_x, color='b',label='y')
# 
# ax2.scatter(t, new_z, color='g',label='z')
# ax2.scatter(t, new_w, color='y',label='w')
# 
# legend = ax2.legend(loc='lower center', shadow=True)
# frame = legend.get_frame()
# for label in legend.get_texts():
#     label.set_fontsize('large')
# 
# for label in legend.get_lines():
#     label.set_linewidth(1.5)  # the legend line width
# #plt.show()



f3 = plt.figure()
ax3 = f3.add_subplot(111)
plt.axis([0, len(t), -1.2,1.2 ])
plt.xlabel('Time step')
plt.ylabel('angle')

ax3.scatter(t, roll, color='r',label='x')
ax3.scatter(t, pitch, color='b',label='y')
ax3.scatter(t, yaw, color='g',label='z')


legend = ax3.legend(loc='lower center', shadow=True)
frame = legend.get_frame()
for label in legend.get_texts():
    label.set_fontsize('large')

for label in legend.get_lines():
    label.set_linewidth(1.5)  # the legend line width
plt.show()



