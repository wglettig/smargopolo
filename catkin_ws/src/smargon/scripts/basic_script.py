#! /usr/bin/env python

# It also works with an anaconda installation (conda activate)
# and rospkg. How to install:
# pip install -U rospkg
################################################################################

#ROS related libraries
import rospy
from std_msgs.msg import Float32

#General libraries
#import math
#from datetime import datetime
#import time

#Global Variables
LJ_AIN0 = 0

#ROS Callback function for the /LU_AIN0 subscriber
def LJ_AIN0_Callback(msg):
    LJ_AIN0 = msg.data
    print(LJ_AIN0)

#Main Function
if __name__ == '__main__':
    #ROS: Initialize stuff:
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Script1', anonymous=True)

    #ROS: Here the Topics are registered (Subscribers & Publishers)
    rospy.Subscriber("/LJ_AIN0", Float32, LJ_AIN0_Callback)

    rospy.spin()
