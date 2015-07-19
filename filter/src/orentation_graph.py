'''
Created on Jul 10, 2015

@author: nathaniel, I&E summer 2015
This program use a kalman filer to smooth the Quaternion of the videoray

'''

import kalman_filter
import matplotlib as mpl
mpl.rc("savefig", dpi=150)
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#import and save data from a CSV file

df = pd.DataFrame.from_csv('/home/nathaniel/Documents/pythonWorkSpace/filter/pose_only.csv', index_col=None)
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

#kalman parameter set up

#wieght of the covarence matrixes
q = 0.05
r =  5456

dt = .0001
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
    new_x.append(temp[4])
    new_y.append(temp[5])
    new_z.append(temp[6])
    new_w.append(temp[7])
    Z = np.matrix([[0], [0], [0], [0], [my_x[i]], [my_y[i]], [my_z[i]], [my_w[i]] ])
    U = np.matrix([[0]])



#print raw and filtered data
f1 = plt.figure()
f2 = plt.figure()
ax1 = f1.add_subplot(111)
ax2 = f2.add_subplot(111)

ax1.scatter(t, my_x, color='b')
ax1.scatter(t, my_y, color='r')
ax1.scatter(t, my_z, color='g')
ax1.scatter(t, my_w, color='y')

ax2.scatter(t, new_x, color='b')
ax2.scatter(t, new_y, color='r')
ax2.scatter(t, new_z, color='g')
ax2.scatter(t, new_w, color='y')


plt.show()
