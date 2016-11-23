'''
Created on Jul 3, 2015

@author: nathaniel
generial linear Kalman fitler 
'''
import numpy as np
from Crypto.Util.number import size
#step

class kalman_filter:
    #take in all the parameters of the linear kalman filter
    def __init__(self,A,B,C,Q,P,R,x):
        self.A = A #state trasition matrix
        self.B = B#control matrix
        self.C = C#measurement model
        self.Q = Q#proccess noise covariance
        self.P = P#predeciton
        self.R = R##measurement noise covariance
        self.state = x
        
    #move to the  next position
    def move(self,u,z ):
        self.predict(u)
        self.update(z)
        
    #get the current state
    def getState(self):
        return self.state
        
    #predict the next state 
    def predict(self,u):
        self.state = self.A*self.state + self.B*u
        self.P = self.A*self.P*self.A.T + self.Q
    #update the model
    def update(self,z):
        #find the kalman gain
        K = self.P*(self.C.T)*np.linalg.inv((self.C*self.P*(self.C.T) +self.R))
        #get the current state
        self.state = self.state + K*(z - self.C*self.state)
        #get the next prediction
        size = self.state.shape[0]
        self.P = (np.eye(size) - K*self.C )*self.P 
        
        
    