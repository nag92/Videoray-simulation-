'''
Created on Jun 30, 2015

@author: nathaniel
'''

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
import matplotlib
mpl.rc("savefig", dpi=150)
plt.ion()


#window size
n = 10
#make some starting values
#random distance
md =[random.random() for _ in range(0, n)]
#random points
x_list = [random.random() for _ in range(0, n)]
y_list =[random.random() for _ in range(0, n)]
#setup list to hold outliers and good points to plot
outlier_x = []
outlier_y = []
outlier_depth = [] 
filt_x = []
filt_y = []
filt_depth = []
#set up graph
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(filt_x,filt_y,filt_depth,color='b')
ax.scatter(outlier_x,outlier_y,outlier_depth,color='r') 
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('XY Outlier rejection \n with Mahalanobis distance and rolling mean3')

#call the fitle the date 
def filter(x,y,z):
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
        ax.scatter(x,y,z,color='b')
        is_good =  True
    else:
        ax.scatter(x,y,z,color='r')
        is_good = False
    #update the plot
    plt.draw()
    return is_good

 





 



 





 
