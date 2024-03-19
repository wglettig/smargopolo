#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#########EPICS COMMANDS

#read speed Omega Achse -> epics.caget("X06MX-ES-DF1:OMEGA-SETV")
#write speed Omega Achse -> epics.caput("X06MX-ES-DF1:OMEGA-SETV",20)
#
#write relative Omega Achse -> epics.caput("X06MX-ES-DF1:OMEGA-INCP",90)
#write absolute Omega Achse -> epics.caput("X06MX-ES-DF1:OMEGA-SETP",20)
#
#read ReadBAck Omega Achse -> epics.caget("X06MX-ES-DF1:OMEGA-RBV")
#read User ReadBAck Omega Achse -> epics.caget("X06MX-ES-DF1:OMEGA-GETP")

#Created on Tue May 18 18:03:02 2021
#Buntschu Dominik
#

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import requests
import json 
import epics
import time
import csv
from datetime import date
import sys
import math
import smaract.ctl as ctl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

#temporary subscriber to check function
DMS=[]
Position=[]
Secs=[]
Secs2=[]
Nsecs=[]
Nsecs2=[]
Seq=[]
Seq2=[]
Liste=[]
LJU6_JointStateOmega=0
CHI_Cal=0
PHI_Cal=0
OMEGA=0
OmegaRDB=0

#neccessary subscriber to put in csv
CHI=[]
PHI=[]
SHX=[]
SHY=[]
SHZ=[]
OX=[]
OY=[]
OZ=[]

DMS_X = []
DMS_Y = []
DMS_Z = []
#d_handle
    
Counter = 0
motion=True

#now = date.time()
today = date.today()
# dd/mm/YY
day = today.strftime("%Y_%m_%d_")
#print("d1 =", d1)
smargopolo_server = "http://smargopolo:3000"

def init_Smargon():
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")

def create_CSV_File():
    #Writing CSV File from List 
    Liste.extend([Seq,Seq2,Secs,Secs2,Nsecs,Nsecs2,DMS_X,DMS_Y,DMS_Z,CHI,PHI])
    with open(day+'getToolError.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["OMEGA","Radius","AI0","AI1","Sequenz1","Sequenz2","Sekunden1","Sekunden1","NaoSek1","NanoSek2","DMS1","DMS2","DMS3","CHI","PHI"])
        writer.writerow(Liste)
        #for row in Liste:
         #   writer.writerow(row)

def move_to_CHI90_Start():
    print("==Move to ZERO at Start============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    #Motion moving to yero Position
    epics.caput("X06SA-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06SA-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06SA-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    
    else:
        print("Omega < 180")
        epics.caput("X06SA-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    get_CHI=readbackSCS['CHI']
    get_PHI=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06SA-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")
    
def move_to_CHI90_Stopp():
    print("==Move to ZERO at Stopp============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    #Motion moving to yero Position
    epics.caput("X06SA-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06SA-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06SA-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(20)
    
    else:
        print("Omega < 180")
        epics.caput("X06SA-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    get_CHI=readbackSCS['CHI']
    get_PHI=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06SA-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")
    
    
def callback_LJUE9_JointState(data):
    
    global Nsecs,Secs,Seq,Liste,CHI,PHI,Position,Counter,OMEGA  
                
    Position=(data.position)
    OMEGA=(data.position[0])
    Secs=(data.header.stamp.secs)
    Nsecs=(data.header.stamp.nsecs)
    Seq=(data.header.seq)
    Liste=list(Position) 
    
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    CHI=readbackSCS['CHI']
    PHI=readbackSCS['PHI']
        
def callback_readbackCAL_JointState(data):   
    
    global DMS_X,DMS_Y,DMS_Z,Seq2,Secs2,Nsecs2
    
    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    Secs2.append(data.header.stamp.secs)
    Nsecs2.append(data.header.stamp.nsecs)
    Seq2.append(data.header.seq)

def plot_graph():

    #3D PLot
    fig = plt.figure()
    ax=fig.add_subplot(111, projection='3d')
    ax.plot(DMS_Z,DMS_X,DMS_Y)
    ax.set_xlabel("DMS_Z")
    ax.set_ylabel("DMS_X")
    ax.set_zlabel("DMS_Y")
    #ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'rx')
    set_axes_equal(ax)
    fig.show()

    #2D PLot
    fig2 = plt.figure()
    ax2=fig2.add_subplot(111)
    ax2.plot(OMEGA, DMS_X, label='DMS_X')
    ax2.plot(OMEGA, DMS_Y, label='DMS_Y')
    ax2.plot(OMEGA, DMS_Z, label='DMS_Z')
    ax2.set_xlabel("OMEGA")
    ax2.legend()
    fig2.show()

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    print (f"x_range: {x_range}")
    print (f"y_range: {y_range}")
    print (f"z_range: {z_range}")
    print (f"x_middle: {x_middle}")
    print (f"y_middle: {y_middle}")
    print (f"z_middle: {z_middle}")
    
    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])    

def calculate_correction(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    print (f"MAX= {max(VECT)}, MIN= {min(VECT)}")
    print (f"current {current}")
    print (f"centre {centre}")
    print (f"CORRECTION: {correction}")
        
def start_ToolError_routine():
    
    print()
    print("============First Motion===========")
    
    #get Smargon
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    get_CHI=round(readbackSCS['CHI'])
    get_PHI=round(readbackSCS['PHI'])
    
    #get Aerotech
    OmegaRDB=epics.caget("X06SA-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)

    #set amount of rotations in degree
    PHI_Rotation = 360
    
    if get_CHI==90 and OmegaRDB == 0:
            
        print("Move PHI to " +str(PHI_Rotation) +" degree")
        response = requests.put(smargopolo_server+'/targetSCS?PHI='+str(PHI_Rotation))
        time.sleep(15)
        print("Move PHI to 0 degree")
        response = requests.put(smargopolo_server+'/targetSCS?PHI='+str(0))
        time.sleep(15)
            
    #get Position Feedback
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text) 
    get_CHI=readbackSCS['CHI']
    get_PHI=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06SA-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")
    
    
if __name__ == '__main__':   
    
    init_Smargon()
    move_to_CHI90_Start()
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
    subs2=rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
    start_ToolError_routine()
    subs1.unregister()
    subs2.unregister()
    create_CSV_File()
    plot_graph()
    calculate_correction(DMS_X)
    calculate_correction(DMS_Y)
    calculate_correction(DMS_Z)
    move_to_CHI90_Stopp()

    


        
        
        





        
        
        
        
        
        


        
        
        




