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
import csv

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
SCS_CHI=[];
#SCS_Seq=[];
#SCS_Secs=[];
#SCS_Nsecs=[];
OMEGA =0
CHI =0

def callback_LJUE9_JointState(data):
    global OMEGA
    OMEGA = data.position[0]

def callback_readbackCAL_JointState(data):
    global DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs,CHI
    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    
    SCS_CHI.append(CHI)

def callback_readbackSCS_JointState(data):
    global CHI
    CHI = data.position[4]

def getCurrentPoint():
    return [DMS_X[-1],DMS_Y[-1],DMS_Z[-1]]

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
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"CORRECTION value for __: 	{correction}")

#######################################################################################
#if __name__ == '__main__':
smargopolo_server = "http://smargopolo:3000"

response = requests.put(smargopolo_server+"/targetSCS?CHI=0")
print ("Setting up ROS")
#connect to ROS topics for OMEGA and DMS values:
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
subsSCS  =rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)
time.sleep(1)

startPoint = getCurrentPoint()
print ("Moving chi to 90deg")
response = requests.put(smargopolo_server+"/targetSCS?CHI=30")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=60")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=90")
time.sleep(5)

endPoint = getCurrentPoint()
print ("Moving chi to 0deg")
response = requests.put(smargopolo_server+"/targetSCS?CHI=60")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=30")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=0")
time.sleep(7)

#response = requests.put(smargopolo_server+"/targetSCS?CHI=0")
#time.sleep(8)

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
ax.plot([startPoint[2]],[startPoint[0]],[startPoint[1]],'rx')
ax.plot([endPoint[2]],  [endPoint[0]],  [endPoint[1]],  'ro')
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
#calculate_correction(DMS_X)
#calculate_correction(DMS_Y)
#calculate_correction(DMS_Z)

################################################################################
#Find Centre point of CHI rotation in DMS_X DMS_Y plane (2D projection)

P0 = [startPoint[0], startPoint[1]]
P1 = [endPoint[0],   endPoint[1]]

v01half   = [ (P1[0]-P0[0]) / 2. , (P1[1]-P0[1]) / 2. ]
v01half90 = [ -v01half[1]  , v01half[0] ]
v0c = [v01half[0]+v01half90[0], v01half[1]+v01half90[1]]

PC = [P0[0]+v0c[0], P0[1]+v0c[1]]

ax.plot([startPoint[2]], [PC[0]], [PC[1]], 'g+')
fig.show()

print (f"Centre Point of Quarter Circle: DMS_X: {PC[0]} DMS_Y: {PC[1]}")

print (f"Correction from current postion: in DMS_X: {v0c[0]} in DMS_Y: {v0c[1]}")
print (f"CORRECTION value for SHZ: 	{v0c[0]}")

################################################################################
# Save Data to CSV

rows = zip(SCS_CHI, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open('measure_chi_rot_OUTPUT.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["SCS_CHI", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)

################################################################################

