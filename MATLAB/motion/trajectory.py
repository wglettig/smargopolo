# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
#import numpy as np
import matplotlib.pyplot as plt


class traj:
    t = 0.
    x = 0.
    v = 0.
    a = 0.
    t_step =0.01
    x_target = 10.
    # parameters    
    v_max = 0.
    a_max = 1.
    
    def set (self, t, x_current, v_current):
        return
    def set_step (self, time):
        self.t_step = time
        return
    def run (self, step_t):
        #based on current position wrt target position
        #check if with current position and velocity, we 
        offdist = self.x_target-self.x
        brakedist = -self.v*self.v/-(2*self.a_max)
        
        if (offdist > brakedist):
            a_sign = 1
        else:
            a_sign = -1
                 
        #reset motion equations
        self.a = a_sign*self.a_max
        self.v = self.v + self.a*step_t
        self.x = self.x + self.v*step_t
        return
    def runstep (self):
        self.t = self.t+self.t_step
        self.run(self.t)
        return
    

#Create Object
tr = traj();

#initialize List
T = [tr.t]
X = [tr.x]
V = [tr.v]
A = [tr.a]
#initialize trajectory
tr.set(0, 0, 0)

#step though trajectory
for i in range (0,50):
    tr.runstep()
    T.append(tr.t)
    X.append(tr.x)
    V.append(tr.v)
    A.append(tr.a)
   
#plot curves
plt.plot(T,X,'-+')
plt.plot(T,V)
plt.plot(T,A)
plt.title("Title")
plt.xlabel('t[s]')
plt.ylabel('x[m]')


