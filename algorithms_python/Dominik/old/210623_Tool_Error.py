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

#temporary subscriber to check function
Position=[]
Secs=[]
Nsecs=[]
Seq=[]
Liste=[]
LJU6_JointStateOmega=0
CHI_Cal=0
PHI_Cal=0
OmegaRDB=0
Counter=[]

#neccessary subscriber to put in csv
CHI=[]
PHI=[]
SHX=[]
SHY=[]
SHZ=[]
OX=[]
OY=[]
OZ=[]

Counter = 0
motion=True


#now = date.time()
today = date.today()
# dd/mm/YY
day = today.strftime("%Y_%m_%d_")
#print("d1 =", d1)
smargopolo_server = "http://smargopolo:3000"
response = requests.get(smargopolo_server+"/readbackSCS")
response = requests.get(smargopolo_server+"/readbackMCS") 
#readbackMCS = json.loads(response.text) 
#readbackSCS = json.loads(response.text) 

     #Writing CSV File from List 
    #generate new CSV fileLJU6_JointStateTimeSekLJU6_JointStateTimeSek
with open(day+'ToolError.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA","Sensor1","Sensor2","Sensor3","spare1","spare2","Sequenz","Sekunden","NanoSekunden","CHI","PHI"]) 
 
#def motion():
#    global motion
#    motion=True
#    
#    time.sleep(20)
#    motion=False

def callback_LJU6_JointState(data):
    
    global Secs,Nsecs,Seq,Liste,CHI,PHI,Position,LJU6_JointStateTimeSek,LJU6_JointStateTimeNanSek,LJU6_JointStateOmega,LJU6_JointStateSensor1,LJU6_JointStateSensor2,LJU6_JointStateSensor3, Counter
    #while loop defines how long subscriber channels are enabled
    #can be related to the executet motion in the future
#    smargopolo_server = "http://smargopolo:3000"
#    response = requests.get(smargopolo_server+"/readbackSCS") 
#    readbackSCS = json.loads(response.text) 
    
    Position=(data.position)
    Secs=(data.header.stamp.secs)
    Nsecs=(data.header.stamp.nsecs)
    Seq=(data.header.seq)
    Liste=list(Position)

  
    LJU6_JointStateOmega=data.position[0]

    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)

    CHI=readbackSCS['CHI']
    PHI=readbackSCS['PHI']
    
    Liste.extend([Seq,Secs,Nsecs,CHI,PHI,Counter])
      

    with open(day+'ToolError.csv', 'a', newline='') as file:
        writer = csv.writer(file)
#        writer.writerows(zip(ROW,LJU6_JointStateTimeSek,LJU6_JointStateTimeNanSek,LJU6_JointStateOmega,LJU6_JointStateSensor1,LJU6_JointStateSensor2,LJU6_JointStateSensor3))
        writer.writerow(Liste)
    
if __name__ == '__main__':
    print("=======Move to ZERO============")
#        #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    if OmegaRDB > 180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-INCP",-OmegaRDB)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-INCP",-OmegaRDB)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=90')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackMCS = json.loads(response.text)
    CHI_Cal=readbackMCS['CHI']
    PHI_Cal=readbackMCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
    print("")
    
    # Initialize ROS and register subscribers:
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJU6_JointState", JointState, callback_LJU6_JointState)
#    rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
    while (motion):
        
        
        #Start Calibration_______________________________
        #////////////////////////////////////////////////
        
        print("============First Motion===========")
        #First Move CHI PHI = 0
        if CHI_Cal==90 and PHI_Cal==0:
            
            print("Move PHI 1800 degree")
            response = requests.put(smargopolo_server+'/targetSCS?PHI=360')
            time.sleep(15)
            print("Move PHI -1800 degree")
            response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
            time.sleep(15)
            print("Move PHI 720 degree")
            response = requests.put(smargopolo_server+'/targetSCS?PHI=-360')
            time.sleep(15)
            print("Move PHI -720 degree")
            response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
            time.sleep(15)
            
        else:
            exit()
        #get Position Feedback
        readbackMCS = json.loads(response.text) 
        CHI_Cal=readbackMCS['CHI']
        PHI_Cal=readbackMCS['PHI']
        OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
        OmegaRDB=round(OmegaRDB, 2)
        print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
        print("")
        #////////////////////////////////////////////////
        
        
        
        subs1.unregister()        
#        rospy.spin()
        motion=False
        #////////////////////////////////////////////////
        
    print("============Move to Zero===========")
    #Fifth Move CHI = 0 PHI = 0
    if CHI_Cal==0:
            
            
    #Move CHI =10 PHI = 0
        print("Move CHI to 0 degree")
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(3)
            
        print("Move Omega to 0 degree")
        OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
        epics.caput("X06MX-ES-DF1:OMEGA-INCP",-OmegaRDB)
            
        time.sleep(10)   
              
        #get Position Feedback
        readbackMCS = json.loads(response.text) 
        CHI_Cal=readbackMCS['CHI']
        PHI_Cal=readbackMCS['PHI']
        OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
        OmegaRDB=round(OmegaRDB, 2)
        print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)     
        print("")
        

        
        
        



