#!/usr/bin/env python

# Bedingung: 
# 1. Ausrichtung des DMS Tools zu Aerotech mit Script: 210827_measure_DMS_Tool_linearity.py
# 2. Omega Achse ausgereichtet mit Script:210827_measure_Omega_Offset_to_Smargon.py
# 3. Die Kalibrierpin Vektoren wurden durch das Script: measure_PinVector_SHX_SHY.py ermittelt

# Dieses Script verfaehrt die Chi Achse des Smargon von 0 - 90 Grad  um den Korrektur Vektor des SHZ Wertes 
# des Kalibrierpins zu ermitteln.

# Das Fehlerbild sollte moeglichst klein sein

# Dominik Buntschu, 27.8.2021

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
SCS_CHI=[];
OMEGA =0
CHI =0

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
    global OMEGA
    OMEGA = data.position[0]

# Auselsen des Kalibriertools
# Auslesen Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA) und CHI Winkel von Smargon
def callback_readbackCAL_JointState(data):
    global DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs,CHI
    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    SCS_CHI.append(CHI)

# Auslesen des Chi Winkels von Smargon
def callback_readbackSCS_JointState(data):
    global CHI
    CHI = data.position[4]

#speichert aktuelle Position der Kalibrierstation (DMS)
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

# Ermittlung des SHZ Vektors
def calculate_correction(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"CORRECTION value for __: 	{correction}")

####### START SCRIPT HERE ################################################################################

#Definiert Variabel fuer den Smargon Server
smargopolo_server = "http://smargopolo:3000"

# Bewegung von CHI zur StartPosition = CHI 0 Grad
response = requests.put(smargopolo_server+"/targetSCS?CHI=0")

# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
subsSCS  =rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)

time.sleep(1)
#speichert Start Punkt anhand DMS Kalibirerstation
Point1 = getCurrentPoint()
response = requests.put(smargopolo_server+"/targetSCS?CHI=30")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=60")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=90")
time.sleep(5)
#speichert End Punkt anhand DMS Kalibirerstation
Point2 = getCurrentPoint()
response = requests.put(smargopolo_server+"/targetSCS?CHI=60")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=30")
time.sleep(5)
response = requests.put(smargopolo_server+"/targetSCS?CHI=0")
time.sleep(7)


# beenden der Datenaufzeichnung
rospy.signal_shutdown('finished measuring')

################################################################################

# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
# Endpunkt der Bewegung = o Punkt
fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z [mm]")
ax.set_ylabel("DMS_X [mm]")
ax.set_zlabel("DMS_Y [mm]")
ax.set_title("3D Plot: Measure Offset Q4 and Pin Vector SHZ",fontsize=14,fontweight="bold")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0],label='StartPoint',marker=(5,0))
ax.plot([Point1[2]],[Point1[0]],[Point1[1]],label='Point1',marker=(5,1))
ax.plot([Point2[2]],  [Point2[0]],  [Point2[1]],label='Point2',  marker=(5,2))
set_axes_equal(ax)

# 2D Plot der Bewegung mit den Werten des Kaibriertools
fig2 = plt.figure()
ax2=fig2.add_subplot(111)
ax2.plot(DMS_X, label='DMS_X')
ax2.plot(CHI, label='CHI')
#ax2.plot(DMS_Z, label='DMS_Z')
ax2.legend()
ax2.set_title("Measure Offset Q4 and Pin Vector SHZ",fontsize=14,fontweight="bold")
fig2.show()
################################################################################

#Find Centre point of CHI rotation in DMS_X DMS_Y plane (2D projection)
P0 = [Point1[0], Point1[1]]
P1 = [Point2[0],   Point2[1]]

v01half   = [ (P1[0]-P0[0]) / 2. , (P1[1]-P0[1]) / 2. ]
v01half90 = [ -v01half[1]  , v01half[0] ]
v0c = [v01half[0]+v01half90[0], v01half[1]+v01half90[1]]

PC = [P0[0]+v0c[0], P0[1]+v0c[1]]

ax.plot([Point1[2]], [PC[0]], [PC[1]],label='Mittelpunkt',marker=(5,3))

#Berrechnung des Winkelversatzes der Q4 Achse
XOffset= P0[0]-PC[0]
YOffset= PC[1]-P0[1]
Q4Winkelfehler = (math.tan(YOffset/XOffset))*(180/math.pi)

ax.legend()
fig.show()

print("Der Winkelfehler Q4 muss vor der SHZ Korrektur korrigiert werden!")
print(f"Q4 Winkelfehler: {Q4Winkelfehler}")
print("----------------------------------")
print (f"Centre Point of Quarter Circle: DMS_X: {PC[0]} DMS_Y: {PC[1]}")
print (f"Correction from current postion: in DMS_X: {v0c[0]} in DMS_Y: {v0c[1]}")
print (f"Korrekturwert SHZ: 	{v0c[0]*-1}")


################################################################################

# Speichern der Daten in ein CSV.
# Bennenung des Files mit Datum und Zeitstempel
rows = zip(SCS_CHI, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_PinVector_SHZ.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["SCS_CHI", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)

################################################################################

