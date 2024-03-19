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
import epics
import csv

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA_inst=0
OMEGA=[];

def callback_LJUE9_JointState(data):

    global OMEGA_inst
    OMEGA_inst = data.position[0]


def callback_readbackCAL_JointState(data):

    global OMEGA,DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs

    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    OMEGA.append(OMEGA_inst)

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
### START SCRIPT HERE #########################################################
#AEROTECH_EPICS_RECORD = "X06MX-ES-DF1"
AEROTECH_EPICS_RECORD = "X06SA-ES-DF1"

print ("This script rotates the Aerotech OMEGA axis from 0-360deg,")
print ("and records the calibration tool position during this motion.")
print ("Make sure:")
print ("* OMEGA is ready to turn freely.")
print ("* Calibration Tool is ready, in contact and feedback is active.")
key=input ("OK to continue? (y/n) ")
if (key != "y"):
    print ('Stopping script.')
    exit(code=None)

#make sure OMEGA is at 0deg
print ("Moving OMEGA to 0deg")
movebackSpeed = 40 # deg/s
currentOMEGA = epics.caget(AEROTECH_EPICS_RECORD + ":OMEGA-GETP")
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV", movebackSpeed)
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETP", 0)
time.sleep(currentOMEGA/movebackSpeed + 2)

print ("Setting up ROS")
#connect to ROS topics for OMEGA and DMS values:
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
print ("Starting data collection...")


print ("Moving OMEGA to 360deg ")
dataCollectionSpeed = 5 #deg/s
print (f"Data collection will take approx {360/dataCollectionSpeed+5} [s]")
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV",dataCollectionSpeed)
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETP",360)
time.sleep(360/dataCollectionSpeed + 5)


#stop ROS to stop measuring.
rospy.signal_shutdown('finished measuring')
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV", movebackSpeed)
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETP",0)
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
ax2.plot(OMEGA, DMS_X, label='DMS_X')
ax2.plot(OMEGA, DMS_Y, label='DMS_Y')
ax2.plot(OMEGA, DMS_Z, label='DMS_Z')
ax2.set_xlabel("OMEGA")
ax2.legend()
fig2.show()

################################################################################

calculate_correction(DMS_X)
calculate_correction(DMS_Y)
calculate_correction(DMS_Z)

################################################################################
# Save Data to CSV

rows = zip(OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open('measure_omega_rot_OUTPUT.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)


input("done.")
