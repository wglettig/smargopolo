#!/usr/bin/env pythoni

# This Script creates a lookup table from the measured error CSV
# Dominik Buntschu, 2.8.2021

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import requests
import time
from datetime import date
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import epics
import csv
import math

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA_inst=0
OMEGA=[];

today = date.today()
current_day = today.strftime("%Y_%m_%d_")
#print(current_day)

now = datetime.now()
current_time = now.strftime("%H:%M:%S_")
#print(current_time)
#print(current_day+current_time)

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


print ("Setting up ROS")
#connect to ROS topics for OMEGA and DMS values:
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
print ("Starting data collection...")

smargopolo_server = "http://smargopolo:3000"
        
for i in range(0,5,1):
    print("Move BZ to +2")
    response = requests.put(smargopolo_server+'/nudgeBCS?BZ=2')
    time.sleep(2)
    startPoint = getCurrentPoint()
    print("Move BY to +1")
    response = requests.put(smargopolo_server+'/nudgeBCS?BY=2')
    time.sleep(2)
    secondPoint = getCurrentPoint()
    print("Move BZ to +1")
    response = requests.put(smargopolo_server+'/nudgeBCS?BZ=-2')
    time.sleep(2)
    print("Move BY to +1")
    response = requests.put(smargopolo_server+'/nudgeBCS?BY=-2')
    time.sleep(2)
    print("Move BX to +1")
    response = requests.put(smargopolo_server+'/nudgeBCS?BX=1')
    time.sleep(2)



#stop ROS to stop measuring.
rospy.signal_shutdown('finished measuring')
################################################################################

fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z")
ax.set_ylabel("DMS_X")
ax.set_zlabel("DMS_Y")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'rx')
ax.plot([startPoint[2]],[startPoint[0]],[startPoint[1]],'rx')
ax.plot([secondPoint[2]],  [secondPoint[0]],  [secondPoint[1]],  'ro')
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

#Omega Offset in degree
print ('Omega Offset in degree')
YOffset= secondPoint[1]-startPoint[1]
ZOffset= secondPoint[2]-startPoint[2]
OmegaOffset = (math.tan(ZOffset/YOffset))*(180/math.pi)
print (f"Omega Offset: {OmegaOffset}")

##########################################i######################################
# Save Data to CSV

rows = zip(OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_repeatability_smargon_2.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)

print("Move BX to +1")
response = requests.put(smargopolo_server+'/nudgeBCS?BX=-5')
time.sleep(2)

input("done.")
