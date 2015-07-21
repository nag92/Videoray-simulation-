'''
Created on Jul 3, 2015

@author: nathaniel, I&E summer 2015

This program filters out the outlier in of the x,y coordinates 
using a Mahalanobis distance with a rolling average and standard deveation 
The crirtia for a oulier is md > mu + 1.25*sigma

'''

import numpy as np
import random
#outler stuff
n = 10
count = 0

md =[random.random() for _ in range(0, n)]
#random points
x_list = [random.random() for _ in range(0, n)]
y_list =[random.random() for _ in range(0, n)]

def outlier(x,y,z):
    # update the window
    is_good = False
    x_list.append(x)
    y_list.append(y)
    x_list.pop(0)
    y_list.pop(0)
    
    #get the covariance matrix
    v = np.linalg.inv(np.cov(x_list,y_list,rowvar=0))
   
    #get the mean vector
    r_mean = np.mean(x_list), np.mean(y_list)  
    #subtract the mean vector from the point
    x_diff = np.array([i - r_mean[0] for i in x_list])
    y_diff = np.array([i - r_mean[1] for i in y_list])
    #combinded and transpose the x,y diff matrix
    diff_xy = np.transpose([x_diff, y_diff])
    # calculate the Mahalanobis distance
    dis = np.sqrt(np.dot(np.dot(np.transpose(diff_xy[n-1]),v),diff_xy[n-1]))
    # update the window 
    #print dis
    print outlier.count
    print outlier.w
    #find mean and standard standard deviation of the standard deviation list
    if outlier.count <= n:
        outlier.count = outlier.count + 1
        md.append( dis)
        md.pop(0)
    mu  = np.mean(md)
    sigma = np.std(md)
    #print md
    
    
    # compare to threshold to see if a outlier
    if dis < mu + outlier.w*sigma:
        is_good = True    
        if outlier.count > n:
            outlier.count = outlier.count + 1
            md.append( dis)
            outlier.w = 1.5
            md.pop(0)
    return is_good
outlier.count = 0
outlier.w = 1.25