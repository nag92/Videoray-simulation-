'''
Created on Jul 3, 2015

@author: nathaniel,I&E summer 2015
make a 3D plot to print to
'''
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



def makeGraph():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('XY Outlier rejection and Kalman filtering')
  
    plt.ion()
    return  plt,ax