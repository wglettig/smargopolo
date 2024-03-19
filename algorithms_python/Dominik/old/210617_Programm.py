#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

#now = date.time()
today = date.today()
# dd/mm/YY
day = today.strftime("%Y_%m_%d_")
#print("d1 =", d1)

#generate new CSV fileLJU6_JointStateTimeSekLJU6_JointStateTimeSek
with open(day+'idohave.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Counter","secs","nanosecs","OMEGA","Sensor1","Sensor2","Sensor3"])  

def callback_LJU6_JointState(data):
    
    global LJU6_JointStateTimeSek,LJU6_JointStateTimeNanSek,LJU6_JointStateOmega,LJU6_JointStateSensor1,LJU6_JointStateSensor2,LJU6_JointStateSensor3, Counter
    
#    smargopolo_server = "http://smargopolo:3000"
#    response = requests.get(smargopolo_server+"/readbackSCS") 
#    readbackSCS = json.loads(response.text) 

    #while loop defines how long subscriber channels are enabled
    #can be related to the executet motion in the future
    while (Counter < 50):
        
        LJU6_JointStateTimeSek.append(data.header.stamp.secs)
        LJU6_JointStateTimeNanSek.append(data.header.stamp.nsecs)
        LJU6_JointStateOmega.append(data.position[0])
        LJU6_JointStateSensor1.append(data.position[1])
        LJU6_JointStateSensor2.append(data.position[2])
        LJU6_JointStateSensor3.append(data.position[3])

        print("The Counter Value is     :" ,Counter)
        time.sleep(0.1)
        Counter+=1
        
    #after while loop (above) is over, subcriber are closed,data stream = closed
    #this mode can be used to generate an csv file out of the recorded datas    
    subs1.unregister()      

    #Writing CSV File from List   
#    with open(day+'idohave.csv', 'a', newline='') as file:
#        i=1
#        for i in range(Counter):
#            print("Step",i)
##            print("Counter",Counter)
#            writer = csv.writer(file)
#            writer.writerow([Counter[i],LJU6_JointStateTimeSek[i],LJU6_JointStateTimeNanSek[i],LJU6_JointStateOmega[i],LJU6_JointStateSensor1[i],LJU6_JointStateSensor2[i],LJU6_JointStateSensor3[i]])

    
if __name__ == '__main__':
    # Initialize ROS and register subscribers:
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJU6_JointState", JointState, callback_LJU6_JointState)
#    rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
      




