'''
Created on Jul 10, 2015

@author: nathaniel
'''
import kalman_filter
import outlier_fitler
import matplotlib as mpl
mpl.rc("savefig", dpi=150)
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#kalman filter parameters 

#impoirt and save data from CSV files
df = pd.DataFrame.from_csv('/home/nathaniel/Documents/pythonWorkSpace/filter/usbl_pose.csv', index_col=None)
df2 = pd.DataFrame.from_csv('/home/nathaniel/Documents/pythonWorkSpace/filter/pose_only.csv', index_col=None)

my_x = df['field.pose.position.x'].values.tolist()
my_y = df['field.pose.position.y'].values.tolist()
my_z = df2['field.position.z'].values.tolist()
print len(my_z  )
new_x = []
new_y = []
new_z = []
t = range(0, len(my_x))

#kalman filter paramaters    
dt = 1
pos = np.matrix([[0],#v_x
                [0],#v_y
                [0],#v_z
                [my_x[0]],#x
                [my_y[0]],#y
                [my_z[0]]])#z
A = np.matrix([[1,  0,  0,  0, 0, 0,],
               [0,  1,  0,  0, 0, 0,],
                [0,  0,  1,  0, 0, 0,],
                [dt, 0,  0,  1, 0, 0,],
                [0,  dt, 0,  0, 1, 0,],
                [0,  0,  dt, 0, 0, 1, ]])
B = np.matrix([0])
C = np.eye(pos.shape[0])
Q = np.eye(pos.shape[0])*.05
R = np.eye(pos.shape[0])*5645
P = np.eye(pos.shape[0])
U = np.matrix( [[0]])
Z = np.matrix( [[0],[0],[0],[0],[0],[0] ])
    
kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,pos)
#varible to hold the last position to for plotting reasons
last = pos
#loop through data
for i in xrange(len(t)):
    #if not outlier out it into kalman filter
    if(outlier_fitler.outlier(my_x[i], my_y[i],my_z[i] )):
        Z = np.matrix( [[0],[0],[0],[my_x[i]],[my_y[i]],[-1*my_z[i]] ])
        kalman.move(U,Z)
        pos = kalman.getState()
        new_x.append(pos[3])
        new_y.append(pos[4])
        new_z.append(pos[5])
        last = pos   
    else:
        new_x.append(last[3])
        new_y.append(last[4])
        new_z.append(last[5])
        
    
#plot the raw and filtered data
f1 = plt.figure()
f2 = plt.figure()
ax1 = f1.add_subplot(111)
ax2 = f2.add_subplot(111)
ax1.scatter(t, my_x, color='b')
ax1.scatter(t, my_y, color='r')
#ax1.scatter(t, my_z, color='g')


ax1.scatter(t, new_x, color='g')
ax1.scatter(t, new_y, color='y')
#ax2.scatter(t, new_z, color='g')

plt.show()

        
    