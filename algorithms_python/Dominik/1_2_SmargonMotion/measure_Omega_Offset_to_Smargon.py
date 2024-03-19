#!/usr/bin/env pythoni

# Dieses Script verfaehrt den Smargon translativ im Smargon Koordintane System OX,OY,OZ.
# Der Ausgangswert des Scripts beschreibt den Winkelversatz zwischen der Aerotech Omega Achse und
# dem Smargon.
# Ziel ist es einen moeglichst kleinen Winkelfehler zu haben. Smargon muss sich rechtwinklig zum Aeroetch 
# bewegen

# Dominik Buntschu, 13.11.2021

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
import math

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA_inst=0;
OMEGA_Seq=0;
OMEGA_Secs=0;
OMEGA_Nsecs=0;
OMEGA=[];

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
    OMEGA_Seq = data.header.seq
    OMEGA_Secs = data.header.stamp.secs
    OMEGA_Nsecs = data.header.stamp.nsecs

# Auselsen des Kalibriertools
# Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA_inst) 
def callback_readbackCAL_JointState(data):

    global OMEGA,DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs

    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(OMEGA_Secs)
    DMS_Nsecs.append(OMEGA_Nsecs)
    DMS_Seq.append(OMEGA_Seq)
    OMEGA.append(OMEGA_inst)

# Funktion um aktueller Werte des Kalibriertools zu speichern
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

### START SCRIPT HERE #########################################################

#AEROTECH_EPICS_RECORD = "X06MX-ES-DF1"
AEROTECH_EPICS_RECORD = "X06SA-ES-DF1"

print ("Dieses Script verfaehrt Smargon mit OX,OY und OZ in 2mm Schritten")
print ("und zeichnet den Fehler mittels des DMS Instrumentes auf")
print ("Achtung:")
print ("**Kalibriertool muss aktiviert und in Kontakt mit der Keramik- Kugel sein.**")
key=input ("OK? (y/n) ")
if (key != "y"):
    print ('Stopping script.')
    exit(code=None)

# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
print ("Starting data collection...")

#Definiert Variabel fuer den Smargon Server
smargopolo_server = "http://smargopolo:3000"
 
# Macht X Iterationen eines Rechteckes mit jeweils einem Versatz
# Veranschaulicht den Winkelversatz zwischen Aerotech und Smargon         
for i in range(0,3,1):
    print("Move OY to +2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OY=2')
    time.sleep(5)
    Point1 = getCurrentPoint()
    print("Move OX to +2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OX=2')
    time.sleep(5)
    Point2 = getCurrentPoint()
    print("Move OY to -2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OY=-2')
    time.sleep(5)
    print("Move OX to -2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OX=-2')
    time.sleep(5)
    print("Move OZ to +1")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OZ=1')
    time.sleep(5)



# beenden der Datenaufzeichnung
rospy.signal_shutdown('finished measuring')
################################################################################

# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z [mm]")
ax.set_ylabel("DMS_X [mm]")
ax.set_zlabel("DMS_Y [mm]")
ax.set_title("3D Plot: Measure Omega Offset to Smargon",fontsize=14,fontweight="bold")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0],label='StartPoint',marker=(5,0),markersize=10)
ax.plot([Point1[2]],[Point1[0]],[Point1[1]],label='Point1',marker=(5,1),markersize=10)
ax.plot([Point2[2]],  [Point2[0]],  [Point2[1]],label='Point2',  marker=(5,2),markersize=10)
ax.legend()
set_axes_equal(ax)
fig.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
#fig2 = plt.figure()
#ax2=fig2.add_subplot(111)
#ax2.plot(OMEGA, DMS_X, label='DMS_X')
#ax2.plot(OMEGA, DMS_Y, label='DMS_Y')
#ax2.plot(OMEGA, DMS_Z, label='DMS_Z')
#ax2.set_xlabel("OMEGA")
#ax2.legend()
#fig2.show()

#Berrechnung des Winkelversatzes zwischen Aerotech und Smargon
YOffset= Point1[1]-Point2[1]
ZOffset= Point1[2]-Point2[2]
OmegaOffset = (math.tan(ZOffset/YOffset))*(180/math.pi)
print (f"Omega muss um folgenden Winkelfehler korrigiert werden: {OmegaOffset*-1}")

print ("**Info: Ein Winkelfehler von +-0.05 Grad sollte angestrebt werden.")
print ("**Info: Vorgeschlagener Wert muss in der Datei smargonCalibrator.cpp angepasst werden.")

################################################################################

# Speichern der Daten in ein CSV.
# Bennenung des Files mit Datum und Zeitstempel
rows = zip(OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_Omega_Offset_to_Smargon.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)

#Faehrt Smargon zurueck zur Startposition
#print("Move OZ back to Start Position")
response = requests.put(smargopolo_server+'/targetSCS_rel?OZ=-3')
time.sleep(10)

input("done.")
