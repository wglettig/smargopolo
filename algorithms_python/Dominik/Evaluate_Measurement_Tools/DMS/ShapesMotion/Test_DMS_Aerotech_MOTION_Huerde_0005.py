#!/usr/bin/env python

# Bedingung: 
# 1. Ausrichtung des DMS Tools zu Aerotech mit Script: 210827_measure_DMS_Tool_linearity.py
# 2. Omega Achse ausgereichtet mit Script:210827_measure_Omega_Offset_to_Smargon.py
# 3. Die Kalibrierpin Vektoren wurden durch das Script: measure_PinVector_SHX_SHY.py ermittelt
# 4. Der Kalibrierpin Vektor wurde durch das Script: measure_PinVector_SHZ_2.py ermittelt

# Dieses Script verfaehrt die Aeroetch Omega Achse um den Fehler des Smargon in X und Y yu ermitteln.
# Ziel der Messung ist das der Kalibrierpin auf die Omega Achse faellt

# Das Fehlerbild sollte moeglichst klein sein

# Dominik Buntschu, 27.8.2021

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import requests
import time
from datetime import date
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import epics
import csv
import json 

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA_inst=0
OMEGA_Seq=0;
OMEGA_Secs=0;
OMEGA_Nsecs=0;
OMEGA=[];
SCS_CHI=[];
CHI=0;

smargopolo_server = "http://smargopolo:3000"

# Aktueller Tag
today = date.today()
current_day = today.strftime("%Y_%m_%d_")
#print(current_day)

# Aktuelle Zeit
now = datetime.now()
current_time = now.strftime("%H:%M:%S_")
#print(current_time)
#print(current_day+current_time)

# Auslesen der Omega Achse
def callback_LJUE9_JointState(data):

    global OMEGA_inst,OMEGA_Seq,OMEGA_Secs,OMEGA_Nsecs
    OMEGA_inst = data.position[0]
    DMS_Secs.append(OMEGA_Secs)
    DMS_Nsecs.append(OMEGA_Nsecs)
    DMS_Seq.append(OMEGA_Seq)

# Auselsen des Kalibriertools
# Auslesen Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA_inst)
def callback_readbackCAL_JointState(data):

    global OMEGA,DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs,CHI
    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    OMEGA.append(OMEGA_inst)
    SCS_CHI.append(CHI)

# Auslesen des Chi Winkels von Smargon
def callback_readbackSCS_JointState(data):
    global CHI
    CHI = data.position[4]

# Stellt Seitenlaengen des 3D Plot in Relation 
def set_axes_equal(ax):

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

    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


### START SCRIPT HERE #########################################################

# Setzt die Geschwidigkeit der Aerotech Achsen
epics.caput("X06MX-ES-DF1:GMX-SETV",2)
epics.caput("X06MX-ES-DF1:GMZ-SETV",2)
epics.caput("X06MX-ES-DF1:GMY-SETV",2)

# Faehrt Aerotech Achsen in Startpositionen
epics.caput("X06MX-ES-DF1:GMX-SETP",89)
epics.caput("X06MX-ES-DF1:GMY-SETP",4)
epics.caput("X06MX-ES-DF1:GMZ-SETP",46)
time.sleep(3)

# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
subsSCS  =rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)
print ("Starting data collection...")

for i in range(0,5,1):
    print("Move GMY to 0.0005")
    epics.caput("X06MX-ES-DF1:GMY-INCP",0.0005)
    time.sleep(3)
    print("Move GMZ to 0.0005")
    epics.caput("X06MX-ES-DF1:GMZ-INCP",0.0005)
    time.sleep(3)
    print("Move GMY to -0.0005")
    epics.caput("X06MX-ES-DF1:GMY-INCP",-0.0005)
    time.sleep(3)
    print("Move GMZ to -0.0005")
    epics.caput("X06MX-ES-DF1:GMZ-INCP",-0.0005)
    time.sleep(3) 
    print("Move GMX 0.0005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",0.0005)
    time.sleep(3) 
	
# beenden der Datenaufzeichnung
rospy.signal_shutdown('finished measuring')

################################################################################

# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z [mm]")
ax.set_ylabel("DMS_X [mm]")
ax.set_zlabel("DMS_Y [mm]")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'rx')
ax.set_title("3D Plot: Hurdles motion Aerotech.csv",fontsize=14,fontweight="bold")
ax.legend()
set_axes_equal(ax)
fig.show()


################################################################################

# Save Data to CSV

rows = zip(SCS_CHI,OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'Test_DMS_Aerotech_Motion_Huerde_0005.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["CHI","OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)


input("done.")
