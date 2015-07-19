
import numpy as np
import random
#outler stuff
n = 10
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
    md.append( dis)
    md.pop(0)
    #find mean and standard standard deviation of the standard deviation list
    mu  = np.mean(md)
    sigma = np.std(md)
    print md
    
    # compare to threshold to see if a outlier
    if dis < mu + 1.25*sigma:
        is_good = True
    return is_good