#!/usr/bin/env python

# This Script creates a lookup table from the measured error CSV
# Wayne Glettig, 16.7.2021

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import requests
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

OMEGA_DEG=[];
RADIUS=[];
AIN0=[];
AIN1=[];

def callback_LJUE9_JointState(data):
    global OMEGA_DEG, RADIUS, AIN0, AIN1
    OMEGA_DEG.append(data.position[0])
    RADIUS.append(data.position[1])
    AIN0.append(data.position[2])
    AIN1.append(data.position[3])

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

#if __name__ == '__main__':
smargopolo_server = "http://smargopolo:3000"

print ("Setting up ROS")
#connect to ROS topics for OMEGA and DMS values:
rospy.init_node('OMEGA_Analog_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
print ("Recording for 15s")

time.sleep(15)

#stop ROS to stop measuring.
rospy.signal_shutdown('finished measuring')
print ("Stopped collecting data.")


fig = plt.figure()
#ax=fig.add_subplot(111, projection='3d')
#ax.plot(DMS_X,DMS_Y,DMS_Z)
#ax.set_xlabel("DMS_X")
#ax.set_ylabel("DMS_Y")
#ax.set_zlabel("DMS_Z")
#set_axes_equal(ax)
#
#fig.show()

ax=fig.add_subplot(111)
ax.plot(AIN0, AIN1)
ax.axis('equal')
ax.grid()
fig.show()

print (f"AIN0 MAX:{max(AIN0)}, CENTRE:{(max(AIN0)+min(AIN0))/2} MIN: {min(AIN0)} RADIUS: {(max(AIN0)-min(AIN0))/2} ")
print (f"AIN1 MAX:{max(AIN1)}, CENTRE:{(max(AIN1)+min(AIN1))/2} MIN: {min(AIN1)} RADIUS: {(max(AIN1)-min(AIN1))/2} ")



#calculate_correction(DMS_X)
#calculate_correction(DMS_Y)
#calculate_correction(DMS_Z)
#
#
