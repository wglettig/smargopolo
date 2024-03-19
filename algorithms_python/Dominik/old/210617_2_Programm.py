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
LJU6_JointStateTimeSek = []
LJU6_JointStateTimeNanSek = []
LJU6_JointStateOmega = []
LJU6_JointStateSensor1 = []
LJU6_JointStateSensor2 = []
LJU6_JointStateSensor3 = []
ROW=[]
Position=[]

LJUE9_JointStateTimeSek = []
LJUE9_JointStateTimeNanSek = []
LJUE9_AIN0 = []
LJUE9_AIN1 = []
LJUE9_RADIUS = []


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

     #Writing CSV File from List 
    #generate new CSV fileLJU6_JointStateTimeSekLJU6_JointStateTimeSek
with open(day+'idohave.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA","Sensor1","Sensor2","Sensor3","spare1","spare2"]) 
 
#def motion():
#    global motion
#    motion=True
#    
#    time.sleep(20)
#    motion=False

def callback_LJU6_JointState(data):
    
    global Position,LJU6_JointStateTimeSek,LJU6_JointStateTimeNanSek,LJU6_JointStateOmega,LJU6_JointStateSensor1,LJU6_JointStateSensor2,LJU6_JointStateSensor3, Counter
    #while loop defines how long subscriber channels are enabled
    #can be related to the executet motion in the future

    Position=(data.position)
    print(Position)
        
    #after while loop (above) is over, subcriber are closed,data stream = closed
    #this mode can be used to generate an csv file out of the recorded datas    
#    subs1.unregister()      
    

    with open(day+'idohave.csv', 'a', newline='') as file:
        writer = csv.writer(file,delimiter=',')
#        writer.writerows(zip(ROW,LJU6_JointStateTimeSek,LJU6_JointStateTimeNanSek,LJU6_JointStateOmega,LJU6_JointStateSensor1,LJU6_JointStateSensor2,LJU6_JointStateSensor3))
        writer.writerow(Position)
    
if __name__ == '__main__':
    # Initialize ROS and register subscribers:
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJU6_JointState", JointState, callback_LJU6_JointState)
#    rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
    while (motion):
        rospy.spin()
        motion=False
        




