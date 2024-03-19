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
UE9=[]
Secs=[]
Secs2=[]
Nsecs=[]
Nsecs2=[]
Seq=[]
Seq2=[]
LJU6_JointStateOmega=0
CHI_Cal=0
PHI_Cal=0
get_CHI=0
get_PHI=0
OmegaRDB=0
Liste=[]

#neccessary subscriber to put in csv
CHI=[]
PHI=[]
SHX=[]
SHY=[]
SHZ=[]
OX=[]
OY=[]
OZ=[]


DMS_X=[]
DMS_Y=[]
DMS_Z=[]

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
#   response = requests.get(smargopolo_server+"/readbackMCS") 

def create_CSV_File():
     #Writing CSV File from List 
    #generate new CSV fileLJU6_JointStateTimeSekLJU6_JointStateTimeSek
    with open(day+'SmargonError.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["OMEGA","Radius","AI0","AI1","Counter","Sequenz1","Sequenz2","Sekunden1","Sekunden1","NaoSek1","NanoSek2","DMS1","DMS2","DMS3","CHI","PHI"]) 
 


def move_to_ZERO_Start():
    print("==Move to ZERO============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS") 
    #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    CHI_Cal=readbackSCS['CHI']
    PHI_Cal=readbackSCS['PHI']
    
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
    print("==Move to ZERO -> DONE============")
    print("")
    
def move_to_ZERO_Stopp():
    print("==Move to ZERO============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")

    #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    CHI_Cal=readbackSCS['CHI']
    PHI_Cal=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
    print("==Move to ZERO -> DONE============")
    print("")
    
    
def callback_LJUE9_JointState(data):
    
    global Nsecs,Secs,Seq,Liste,CHI,PHI,Omega,Counter,loop
        
#    start_time = time.time() 
        
    UE9=(data.position)
    Secs=(data.header.stamp.secs)
    Nsecs=(data.header.stamp.nsecs)
    Seq=(data.header.seq)
    Liste=list(UE9)
        
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    CHI=readbackSCS['CHI']
    PHI=readbackSCS['PHI']

    
    Liste.extend([Counter,Seq,Seq2,Secs,Secs2,Nsecs,Nsecs2,DMS_X,DMS_Y,DMS_Z,CHI,PHI])
    
    with open(day+'SmargonError.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(Liste) 

def callback_readbackCAL_JointState(data):   
    
    global DMS_X,DMS_Y,DMS_Z,Seq2,Secs2,Nsecs2
    
    DMS_X = (data.position[0])
    DMS_Y = (data.position[1])
    DMS_Z = (data.position[2])
    Secs2=(data.header.stamp.secs)
    Nsecs2=(data.header.stamp.nsecs)
    Seq2=(data.header.seq)

def start_Calibration_routine(loop):
    
    global Counter,get_CHI,get_PHI
    
    if loop==0:
        CHI_Cal = loop
        Omega_Cal = 360
    else:
        CHI_Cal = loop *10
        Omega_Cal = (loop+1)*360 
        
    Counter= loop
    
    #get Aerotech
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    #get Smargon
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    get_CHI=round(readbackSCS['CHI'])
    get_PHI=round(readbackSCS['PHI'])  
    print('============Move 360degree with CHI: '+ str(CHI_Cal))
    #First Move CHI PHI = 0    
    response = requests.put(smargopolo_server+'/targetSCS?CHI='+ str(CHI_Cal))
    response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
    time.sleep(3)
    #Move Omega 360 degree with CHI + PHI = 0
    epics.caput("X06MX-ES-DF1:OMEGA-SETP",Omega_Cal)
    time.sleep(8)
    
    get_CHI=round(readbackSCS['CHI'])
    get_PHI=round(readbackSCS['PHI'])
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
    move_to_ZERO_Start()
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
    subs2=rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
    
    for loop in range(0,5,1):#0,5,1 = 0-40Grad
        start_Calibration_routine(loop)
    subs1.unregister() 
    subs2.unregister() 
         
#   rospy.spin()
    move_to_ZERO_Stopp()

    


        
        
        




