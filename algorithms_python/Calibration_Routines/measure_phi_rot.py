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


DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA =0;

def callback_LJUE9_JointState(data):

    global OMEGA
    OMEGA = data.position[0]


def callback_readbackCAL_JointState(data):

    global DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs

    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    

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

response = requests.put(smargopolo_server+"/targetSCS?PHI=0")
print ("Setting up ROS")
#connect to ROS topics for OMEGA and DMS values:
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
time.sleep(1)

print ("Moving phi to -180deg")
response = requests.put(smargopolo_server+"/targetSCS?PHI=-90")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=-180")
time.sleep(5)
print ("moving to phi=180deg")
response = requests.put(smargopolo_server+"/targetSCS?PHI=-90")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=0")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=90")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=180")
time.sleep(5)

print ("moving to phi=0deg")
response = requests.put(smargopolo_server+"/targetSCS?PHI=0")
time.sleep(8)

#stop ROS to stop measuring.
rospy.signal_shutdown('finished measuring')
print ("Stopped collecting data.")

################################################################################

fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z")
ax.set_ylabel("DMS_X")
ax.set_zlabel("DMS_Y")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'rx')
set_axes_equal(ax)

fig.show()

fig2 = plt.figure()
ax2=fig2.add_subplot(111)
ax2.plot(DMS_X, label='DMS_X')
ax2.plot(DMS_Y, label='DMS_Y')
ax2.plot(DMS_Z, label='DMS_Z')
ax2.legend()
fig2.show()

################################################################################

calculate_correction(DMS_X)
calculate_correction(DMS_Y)
calculate_correction(DMS_Z)


