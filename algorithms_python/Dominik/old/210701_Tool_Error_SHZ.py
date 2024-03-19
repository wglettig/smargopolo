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
#
#GMX, GMY, GMZ

#Created on Tue May 18 18:03:02 2021

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

#temporary subscriber to check function
DMS=[]
Position=[]
Secs=[]
Nsecs=[]
Seq=[]
Liste=[]
LJU6_JointStateOmega=0
CHI_Cal=0
PHI_Cal=0
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

p_x = []
p_y = []
p_z = []
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
    #generate new CSV fileLJU6_JointStateTimeSekLJU6_JointStateTimeSek
    with open(day+'ToolError_SHZ.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["OMEGA","Radius","AI0","AI1","Sequenz1","Sequenz2","Sekunden1","Sekunden1","NaoSek1","NanoSek2","DMS1","DMS2","DMS3","CHI","PHI"]) 

def move_to_CHI90_Start():
    print("==Move to ZERO at Start============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    get_CHI=readbackSCS['CHI']
    get_PHI=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")
    
def move_to_CHI90_Stopp():
    print("==Move to ZERO at Stopp============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(20)
    
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    get_CHI=readbackSCS['CHI']
    get_PHI=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")
    
    
def callback_LJUE9_JointState(data):
    
    global Nsecs,Secs,Seq,Liste,CHI,PHI,Position,Counter,p_y,p_z,p_x 
                
    Position=(data.position)
    Secs=(data.header.stamp.secs)
    Nsecs=(data.header.stamp.nsecs)
    Seq=(data.header.seq)
    Liste=list(Position) 
    
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    CHI=readbackSCS['CHI']
    PHI=readbackSCS['PHI']
    
    plt.xlabel("Time")
    plt.title("Calibrator Data")
    plt.ion()
    plt.show()

    p_y.append(DMS_Y)
    p_z.append(DMS_Z)
    p_x.append(DMS_X)
    plt.plot(p_y,p_x)
#    plt.plot(p_z, '-g', label="Y")
#    plt.plot(p_x, '-r', label="Z")
    
    Liste.extend([Seq,Seq2,Secs,Secs2,Nsecs,Nsecs2,DMS_Y,DMS_Z,DMS_X,CHI,PHI])

    with open(day+'ToolError_SHZ.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(Liste) 
        
def callback_readbackCAL_JointState(data):   
    
    global DMS_X,DMS_Y,DMS_Z,Seq2,Secs2,Nsecs2
    
    DMS_Y = (data.position[0])
    DMS_Z = (data.position[1])
    DMS_X = (data.position[2])
    Secs2=(data.header.stamp.secs)
    Nsecs2=(data.header.stamp.nsecs)
    Seq2=(data.header.seq)
        
def start_ToolError_SHZ_routine():
    
    print()
    print("============First Motion===========")
    
    #Fget Smargon
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    get_CHI=round(readbackSCS['CHI'])
    get_PHI=round(readbackSCS['PHI'])
    
    #get Aerotech
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    
    CHI_Rotation = 0
    
    if get_CHI==90 and OmegaRDB == 0:
            
        print("Move CHI to " +str(CHI_Rotation) +" degree")
        response = requests.put(smargopolo_server+'/targetSCS?CHI='+str(CHI_Rotation))
        time.sleep(15)
        print("Move CHI to 0 degree")
        response = requests.put(smargopolo_server+'/targetSCS?CHI='+str(90))
        time.sleep(15)
#        print("Move PHI to " +str(PHI_Rotation) +" degree")
#        response = requests.put(smargopolo_server+'/targetSCS?PHI='+str(PHI_Rotation))
#        time.sleep(15)
#        print("Move PHI to 0 degree")
#        response = requests.put(smargopolo_server+'/targetSCS?PHI='+str(0))
#        time.sleep(15)
            
    else:
        exit()
    #get Position Feedback
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text) 
    get_CHI=readbackSCS['CHI']
    get_PHI=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")
    #////////////////////////////////////////////////
    
    
if __name__ == '__main__':   
    
#    smargopolo_server = "http://smargopolo:3000"
#    response = requests.get(smargopolo_server+"/readbackSCS")
#    readbackMCS = json.loads(response.text)
    
    init_Smargon()
    create_CSV_File()
    move_to_CHI90_Start()
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
    subs2=rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
    start_ToolError_SHZ_routine()   
    subs1.unregister() 
    subs2.unregister()           
    move_to_CHI90_Stopp()

    


        
        
        





        
        
        
        
        
        


        
        
        




