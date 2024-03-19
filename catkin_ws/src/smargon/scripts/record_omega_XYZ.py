#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import epics

LJ_AIN0 = 0
LJ_AIN1 = 0
LJ_DEG = 0
LJ_RADIUS = 0
LJ_cycle_us = 0
readbackOMEGA = 0

idegSum = 0
degElems = 0

def callback_LJ_AIN0(data):
    LJ_AIN0 = data.data

def callback_LJ_AIN1(data):
    LJ_AIN1 = data.data

def callback_LJ_DEG(data):
    global degSum, degElems, LJ_DEG
    LJ_DEG = data.data
    print (LJ_DEG)
    #degElems += 1
    #degSum += LJ_DEG
    #print (degSum/degElems)

def callback_LJ_RADIUS(data):
    LJ_RADIUS = data.data

def callback_LJ_cycle_us(data):
    LJ_cycle_us = data.data
    print(LJ_cycle_us)

def callback_readbackOMEGA(data):
    readbackOMEGA = data.data

if __name__ == '__main__':
    # Initialize ROS and register subscribers:
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("LJ_AIN0", Float32, callback_LJ_AIN0)
    rospy.Subscriber("LJ_AIN1", Float32, callback_LJ_AIN1)
    rospy.Subscriber("LJ_DEG", Float32, callback_LJ_DEG)
    rospy.Subscriber("LJ_RADIUS", Float32, callback_LJ_RADIUS)
    rospy.Subscriber("LJ_cycle_us", Int32, callback_LJ_cycle_us)
    rospy.Subscriber("readbackOMEGA", Float32, callback_readbackOMEGA)
    
    epics.caget('X06MX-ES-DF1:OMEGA-GETP')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
