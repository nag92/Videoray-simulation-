'''
Created on Nov 7, 2015

@author: nathaniel
'''

import numpy as np
from cmath import sqrt
class poly_filter(object):
	'''
	classdocs
	'''
	

	def __init__(self,window_size,degree):
		'''
		Constructor
		'''
		self.file = open("newfile.txt", "w")
		self.window = window_size
		self.degree = degree
		self. y = []
		self.x = []
		self.count = 0
			
	def update(self, data):
		
		self.time = 0
		self.old_MSE = 0 
		self.count = self.count + 1
		self.x.append(self.count)
		self.y.append(data)
		#print "---------"
		weights = np.diag( [1 for _ in xrange(self.window)] )
		threash = 2
		isGood = True
		
		if len(self.x) >= self.window:
			
			resid = self.regress(self.y,self.x,weights )
			
			squared_resid = [i ** 2 for i in resid]
			
			
			mse =  sum(squared_resid) / len(resid) ;
			print "mse " + str(mse)
			if abs( resid[-1]/sqrt(mse) ) > threash :
				isGood = False
				#print isGood
			self.x.pop(0)
			self.y.pop(0)
		return isGood
			
	
	def getCoef(self, y,x,weights):
			
		my_x = self.make_x(x)	
		temp1 = np.dot( np.transpose(my_x) , weights )  
		temp2 = np.dot(temp1, my_x)
		#print temp2
		if np.linalg.det(temp2) != 0:
			temp3 = np.linalg.inv(temp2)
		else:
			temp3 = np.linalg.pinv(temp2)
		temp4 = np.dot(temp3, np.transpose(my_x))
		temp5 = np.dot(temp4, weights)
		temp6 = np.dot(temp5, np.transpose(y))
		
		return temp6
				
	def make_x(self,  data):
		
		m = []
		for x in data:
			temp = []
			for i in xrange(self.degree+1):
				temp.append(x**i)
		#print temp
			m.append(temp)
	#print np.transpose(zip(*m))
		m = (zip(*m))
		return  np.transpose(np.transpose(zip(*m)))
			
	def get_residuals(self,y,x, coef ):
	
		ybar = np.polyval(list(reversed(coef)),x)
		r = y-ybar
		#print r
		return r
		
	
	def bisquare(self, resid ):
		
		MAD = sum(abs(resid - np.mean(resid)))/len(resid);
		
		k = (4.685*MAD)/.6745
		#print "k " + str(k)
		w = []
		
		for r in resid:
			if abs(r) <= k:
				w.append(  (1 - (r/k)**2 )**2  )
			else:
				w.append(0)
		return w
			
# 	def regress(self,y,x,w):
# 		
# 		error = .00001
# 		coef = self.getCoef(y, x, w)
# 		resid = self.get_residuals(y, x, coef)
# 		#print "resid " + str(resid)
# 		
# 		resid_squared  = [i ** 2 for i in resid]
# 		mse = sum(resid_squared)/ len(resid);
# 		
# 		if abs( self.old_MSE - mse ) < error or self.time == 11:
# 			
# 			self.old_MSE = mse
# 			self.time = self.time + 1
# 			new_w = np.diag( self.bisquare(resid) )
# 			resid = self.regress(y,x,new_w )
# 		
# 		return resid




# 	
	def regress(self,y,x,w):
		
		error = .00001
		not_convergered = True
		new_w = w
		time = 0 
		mse = 0
		old_MSE = 1
		while not_convergered:
			
			coef = self.getCoef(y, x, new_w)
			
			resid = self.get_residuals(y, x, coef)
			
			#print "resid " + str(resid)
			resid_squared  = [i ** 2 for i in resid]
			mse = sum(resid_squared)/ len(resid);
			time = time + 1
			new_w = np.diag( self.bisquare(resid) )
			#print "weights " + str(self.bisquare(resid))
			#print "time " +  str(time)
			if abs( old_MSE - mse ) < error or time == 10:
				not_convergered = False
			
			old_MSE = mse
			#print "mse " + str(mse)
		return resid
# 				
				
		
			