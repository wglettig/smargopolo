#!/usr/bin/env pythoni

# Dieses Script verfaehrt den Smargon translativ im Smargon Koordintane System OX,OY,OZ.
# Der Ausgangswert des Scripts beschreibt den Winkelversatz zwischen der Aerotech Omega Achse und
# dem Smargon.
# Ziel ist es einen moeglichst kleinen Winkelfehler zu haben. Smargon muss sich rechtwinklig zum Aeroetch 
# bewegen

# Dominik Buntschu, 2.8.2021

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
OMEGA_inst=0
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

    global OMEGA_inst
    OMEGA_inst = data.position[0]

# Auselsen des Kalibriertools
# Auslesen Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA_inst) 
def callback_readbackCAL_JointState(data):

    global OMEGA,DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs

    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    OMEGA.append(OMEGA_inst)

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


def calculate_correction(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    print (f"MAX= {max(VECT)}, MIN= {min(VECT)}")
    print (f"current {current}")
    print (f"centre {centre}")
    print (f"CORRECTION: {correction}")
    

### START SCRIPT HERE #########################################################

#AEROTECH_EPICS_RECORD = "X06MX-ES-DF1"
AEROTECH_EPICS_RECORD = "X06SA-ES-DF1"

print ("This script moves Smargon in Beamline Coordinates BX BY and BZ by approx. 2mm,")
print ("and records the calibration tool position during this motion.")
print ("***** Calibration Tool is ready, in contact and feedback is active *******")

key=input ("OK to continue? (y/n) ")
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
for i in range(0,5,1):
    print("Move OY to +2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OY=2')
    time.sleep(2)
    startPoint = getCurrentPoint()
    print("Move OX to +2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OX=2')
    time.sleep(2)
    secondPoint = getCurrentPoint()
    print("Move OY to -2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OY=-2')
    time.sleep(2)
    print("Move OX to -2")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OX=-2')
    time.sleep(2)
    print("Move OZ to +1")
    response = requests.put(smargopolo_server+'/targetSCS_rel?OZ=1')
    time.sleep(2)



# beenden der Datenaufzeichnung
rospy.signal_shutdown('finished measuring')
################################################################################

# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z")
ax.set_ylabel("DMS_X")
ax.set_zlabel("DMS_Y")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'bx')
ax.plot([startPoint[2]],[startPoint[0]],[startPoint[1]],'rx')
ax.plot([secondPoint[2]],  [secondPoint[0]],  [secondPoint[1]],  'ro')
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
YOffset= secondPoint[1]-startPoint[1]
ZOffset= secondPoint[2]-startPoint[2]
OmegaOffset = (math.tan(ZOffset/YOffset))*(180/math.pi)
print (f"Omega Offset needs to be corrected by: {OmegaOffset}")

##########################################i######################################

# Speichern der Daten in ein CSV.
# Bennenung des Files mit Datum und Zeitstempel
rows = zip(OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_Omega_Offset_to_Smargon.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)

print("Move OZ back to Start Position")
response = requests.put(smargopolo_server+'/targetSCS_rel?OZ=-5')
time.sleep(2)

input("done.")
