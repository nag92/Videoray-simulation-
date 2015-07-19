'''
Created on Jul 3, 2015

@author: nathaniel, I&E summer 2015

This program tests the outlier and kalman filer in 3D

'''

import matplotlib as mpl
import pandas as pd
import matplotlib.pyplot as plt
import makeGraph
import outlier_fitler
import kalman_filter
import numpy as np
import random


mpl.rc("savefig", dpi=150)
#plt.ion()




if __name__ == '__main__':
    #get data from CSV file and save it into lists
    df = pd.DataFrame.from_csv('/home/nathaniel/Documents/pythonWorkSpace/outlier_filter/usbl_data.csv', index_col=None)
    #print df
    my_x = df['x'].values.tolist()
    my_y= df['y'].values.tolist()
    my_z =  df['z'].values.tolist()
    my_depth =  df['depth'].values.tolist()
   

    plt, ax = makeGraph.makeGraph()
    #make all the Kalman filer parameters

    dt = 1
    
    pos = np.matrix([[0],#v_x
                     [0],#v_y
                     [0],#v_z
                     [0],#x
                     [0],#y
                     [0]])#z
    
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
    #plt.ion()
    
    plt.show()
    #loop throught data
    for i in xrange(len(my_x)):
        print i
        #if not outlier out it into kalman filter
        if(outlier_fitler.outlier(my_x[i], my_y[i],my_depth[i] )):
            #print the NOT outleirs
            ax.scatter(my_x[i], my_y[i],-my_depth[i],color='b')
            #calculate and get the kalman points
            Z = np.matrix( [[0],[0],[0],[my_x[i]],[my_y[i]],[my_depth[i]] ])
            kalman.move(U,Z)
            pos = kalman.getState()
            #print the kalman points
            ax.scatter(pos[3], pos[4],-pos[5],color='g')
        else:
            #print the outliers
            ax.scatter(my_x[i], my_y[i],-my_depth[i],color='r')
        #print pos
        #print 'x' +str(pos[3])
        #print 'y' +str(pos[4])
        #print 'z' +str(pos[5])
        
        #update graph thing
        plt.draw()

    #update graph thing
    plt.ioff()
    plt.show()





