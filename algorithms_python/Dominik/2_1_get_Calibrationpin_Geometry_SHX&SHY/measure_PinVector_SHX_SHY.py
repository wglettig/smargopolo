#!/usr/bin/env python

# Bedingung: 
# 1.Ausrichtung des DMS Tools zu Aerotech mit Script: 	measure_DMS_Tool_angleError.py
# 2.Omega Achse ausgereichtet mit Script:			measure_Omega_Offset_to_Smargon.py

# Dieses Script verfaehrt die PHI Achse des Smargon um den Korrektur Vektor SHX und SHY des Kalibrierpins zu 
# bestimmen.

# Dominik Buntschu, 13.11.2021

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
import csv
import math

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA_Seq=0;
OMEGA_Secs=0;
OMEGA_Nsecs=0;
OMEGA =0;

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

    global OMEGA,OMEGA_Seq,OMEGA_Secs,OMEGA_Nsecs
    OMEGA = data.position[0]
    OMEGA_Seq = data.header.seq
    OMEGA_Secs = data.header.stamp.secs
    OMEGA_Nsecs = data.header.stamp.nsecs

# Auselsen des Kalibriertools
# Auslesen Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA_inst) 
def callback_readbackCAL_JointState(data):

    global DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs

    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(OMEGA_Secs)
    DMS_Nsecs.append(OMEGA_Nsecs)
    DMS_Seq.append(OMEGA_Seq)

# Funktion um aktueller Werte des Kaliobriertools zu speichern
# Wird fuer Makierung im 3D Plot verwendet 
def getCurrentPoint():
    return [DMS_X[-1],DMS_Y[-1],DMS_Z[-1]]
    
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

    #print (f"x_range: {x_range}")
    #print (f"y_range: {y_range}")
    #print (f"z_range: {z_range}")
    #print (f"x_middle: {x_middle}")
    #print (f"y_middle: {y_middle}")
    #print (f"z_middle: {z_middle}")

    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# Ermittlung des SHX Vektors
def calculate_correction_SHX(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"Korrekturwert SHX [mm]: 	{correction*-1}")
 
# Ermittlung des SHY Vektors
def calculate_correction_SHY(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				 {max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:			 	{current}")
    #print (f"centre:		 		{centre}")
    print (f"Korrekturwert SHY [mm]:		{correction*-1}")
    
        
#Definiert Variabel fuer den Smargon Server
smargopolo_server = "http://smargopolo:3000"

### START SCRIPT HERE #########################################################

# Bewegung von PHI zur StartPosition = PHI 0 Grad
response = requests.put(smargopolo_server+"/targetSCS?PHI=0")

# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)

response = requests.put(smargopolo_server+"/targetSCS?PHI=-90")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=-180")
time.sleep(5)
Point1 = getCurrentPoint()
response = requests.put(smargopolo_server+"/targetSCS?PHI=-90")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=0")
time.sleep(5)
Point2 = getCurrentPoint()
response = requests.put(smargopolo_server+"/targetSCS?PHI=90")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=180")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?PHI=0")
time.sleep(5)

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
ax.set_title("3D Plot: Pin Vector SHX, SHY",fontsize=14,fontweight="bold")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0],label='StartPoint',marker=(5,0),markersize=10)
ax.plot([Point1[2]],[Point1[0]],[Point1[1]],label='Point1',marker=(5,1),markersize=10)
ax.plot([Point2[2]],  [Point2[0]],  [Point2[1]],label='Point2',  marker=(5,2),markersize=10)
ax.legend()
set_axes_equal(ax)
fig.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
#fig2 = plt.figure()
#ax2.set_title("2D Plot: Measure Offset SHX , SHY")
#ax2=fig2.add_subplot(111)
#ax2.plot(DMS_X, label='DMS_X')
#ax2.plot(DMS_Y, label='DMS_Y')
#ax2.plot(DMS_Z, label='DMS_Z')
#set_axes_equal(ax2)
#ax2.legend()
#fig2.show()

################################################################################

calculate_correction_SHX(DMS_Y)
calculate_correction_SHY(DMS_Z)

################################################################################

print ("**Info: der Korrekturwert sollte im Bereich von +-1um sein.**")

# Speichern der Daten in ein CSV.
# Bennenung des Files mit Datum und Zeitstempel
rows = zip(DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_PinVector_SHX_SHY.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)
