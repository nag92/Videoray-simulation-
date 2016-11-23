'''
Created on Jul 3, 2015

@author: nathaniel, I&E summer 2015

This program filters out the outlier in of the x,y coordinates 
using a Mahalanobis distance with a rolling average and standard deveation 
The crirtia for a oulier is md > mu + 1.25*sigma

'''

import numpy as np
import random
from cmath import sqrt
#import matplotlib.pyplot as plt
#import matplotlib as mpl

class md_filter:
	
   def __init__(self, size, threshold, window, range): 
	   # get values
	   self.size = size
	   self.threshold = sqrt(threshold) # values for weighting the tolerance
	   self.window = window  # window size to hold values
	   self.range = range  # range of starting values
	   self.count = 0
	   # make the random list
	   self.md_list = [ random.uniform(self.range[0], self.range[1]) for _ in xrange(0, self.window)]
	   #print self.md_list
	   self.var_list = []# [ random.uniform(self.range[0], self.range[1]) for _ in xrange(0, self.window)]
	   md_filter.temp = 0
	   self.noise = 0
	   for i in xrange(self.size):
			temp = [ random.uniform(self.range[0], self.range[1]) for _ in xrange(0, self.window)]
			self.var_list.append(temp)
	   #self.var_list = np.vstack(self.var_list)
	   #print self.var_list
   
   	
   def getNoise(self):
   		return self.noise 
   	
   	
	
   def update(self, list):
   		#print "###############################################33"
   		
   		is_good = False
   		#upadate the list
   		mean = []
		for i in xrange(self.size):
			#print str(i)
			self.var_list[i].append(list[i])
			self.var_list[i].pop(0)
			mean.append(np.mean(self.var_list[i]))
		#stackt the list
  		stackList = np.vstack(self.var_list)
  	
  		mean = np.vstack(mean)
  		diff = np.transpose(stackList - mean)
  		#diff = np.transpose(diff)
  		#print stackList
  		v = np.linalg.inv(np.cov(stackList))
  		dis = np.sqrt(np.dot(np.dot(np.transpose(diff[self.window-1]),v),diff[self.window-1]))
  		
  		if self.count <= self.window:
  			self.count = self.count + 1
  			self.md_list.append(dis)
  			self.md_list.pop(0)
  		
  		mu = np.mean(self.md_list)
  		sigma = np.std(self.md_list)
		#print "mu:  " + str(mu) + "self.w*sigma " + str(self.w*sigma)
  		self.noise = dis# - (mu + self.w*sigma)
  		
  		if dis <= self.threshold:#
  			#self.noise  = 0 
  			is_good = True
  			
  			if self.count > self.window:
  			 	self.md_list.pop(0)
  			 	self.md_list.append(dis)
  			 	
  		
  		return is_good
  	
  
  			 	