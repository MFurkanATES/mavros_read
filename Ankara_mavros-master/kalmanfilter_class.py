#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  4 19:31:53 2018

@author: omer
"""

import numpy as np
#import pandas as pd
#import matplotlib.pyplot as plt
#import random as rd
#import matplotlib.mlab as mlap
#import scipy.stats as stats

class KalmanFilter:
    
	def __init__(self,Zt):
        
        	self.P=[[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]]
        	self.A=np.matrix(self.P)
        	self.i=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        	self.l=np.matrix((self.i),dtype=float)
        	self.Rt=np.matrix(self.l)
        	self.Ct=np.matrix(([[1,0,0,0],[0,1,0,0]]),dtype=float)
        	self.l1=np.matrix([[1,0],[0,1]])
        	self.Qt1=np.multiply(self.l1,120)
        
        	self.Mt2=np.matrix([[0],[0],[0],[0]],dtype=float)
        	self.E1=np.matrix(self.l)
        	self.mt1=[[0,0,0,0]]
        	self.Mt1=np.matrix((self.mt1),dtype=float).T
        	self.E2=np.matrix([])
        	self.Mt=np.matrix([[0],[0],[0],[0]],dtype=float)
        	self.rr=np.matrix([[0,0],[0,0]])     
        	self.Mt1[0,0]=Zt[0,0]
        	self.Mt1[1,0]=Zt[0,1]
        	   	
        	
	def KalmanUpdate(self,Zt):
        
        	self.Mt=self.A*self.Mt1
        	self.Et=self.A*self.E1*self.A.T+self.Rt
        	self.rr=self.Ct*self.Et*self.Ct.T+self.Qt1
        	self.Kt=np.matmul(np.matmul(self.Et,self.Ct.T),self.rr.I)
        	self.Mt2=self.Mt+self.Kt*(Zt.T-self.Ct*self.Mt)
        
        	self.E2=(self.l-self.Kt*self.Ct)*self.Et    
#        self.E1=self.E2
        	self.Mt1=self.Mt2
        	self.E1=self.E2
        
        	return self.Mt2












































