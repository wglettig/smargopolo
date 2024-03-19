#!/usr/bin/env python

# Dieses Script verfaehrt den Aerotech translativ mit GMX,GMY und GMZ.
# Der Ausgangswert des Scripts beschreibt den Winkelversatz zwischen Aerotech und
# dem Kalibrier Tool.

# Dominik Buntschu, 2.8.2021

import rospy
from sensor_msgs.msg import JointState
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

print ("This script moves Aerotech GMX GMY and GMZ by aprox. 2mm")
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

# smargopolo_server = "http://smargopolo:3000"

# Setzt die Geschwidigkeit der Aerotech Achsen
epics.caput("X06MX-ES-DF1:GMX-SETV",2)
epics.caput("X06MX-ES-DF1:GMZ-SETV",2)
epics.caput("X06MX-ES-DF1:GMY-SETV",2)

# Macht X Iterationen eines Rechteckes mit jeweils einem Versatz
# Veranschaulicht den Winkelversatz zwischen Aerotech und Kalibriertool        
for i in range(0,5,1):
    print("Move GMZ to +2")
    epics.caput("X06MX-ES-DF1:GMZ-INCP",2)
    time.sleep(2)
    Point1 = getCurrentPoint()
    print("Move GMY to +2")
    epics.caput("X06MX-ES-DF1:GMY-INCP",2)
    time.sleep(2)
    Point2 = getCurrentPoint()
    print("Move GMZ to -2")
    epics.caput("X06MX-ES-DF1:GMZ-INCP",-2)
    time.sleep(2)
    Point3 = getCurrentPoint()
    print("Move GMY to -2")
    epics.caput("X06MX-ES-DF1:GMY-INCP",-2)
    time.sleep(2)
    print("Move GMX to +1")
    epics.caput("X06MX-ES-DF1:GMX-INCP",1)
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
ax.set_title("3D Plot: Measure Linearity to DMS Tool",fontsize=14,fontweight="bold")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0],label='StartPoint',marker=(5,0))
ax.plot([Point1[2]],[Point1[0]],[Point1[1]],label='Point1',marker=(5,1))
ax.plot([Point2[2]],  [Point2[0]],  [Point2[1]],label='Point2',  marker=(5,2))
ax.plot([Point3[2]],  [Point3[0]],  [Point3[1]],label='Point3',  marker=(5,3))
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

#Berrechnung des Versatzes zwischen Aerotech und Kalibirertool-> Rotation um GMX
YOffset= Point2[1]-Point1[1]
ZOffset= Point2[2]-Point1[2]
GMXOffset = (math.tan(ZOffset/YOffset))*(180/math.pi)
print (f"Winkelversatz um GMX: {GMXOffset}")

#Berrechnung des Versatzes zwischen Aerotech und Kalibirertool-> Rotation um GMZ
YOffset= Point2[1]-Point1[1]
XOffset= Point2[0]-Point1[0]
GMZOffset = (math.tan(XOffset/YOffset))*(180/math.pi)
print (f"Winkelversatz um GMZ: {GMZOffset}")

#Berrechnung des Versatzes zwischen Aerotech und Kalibirertool-> Rotation um GMY
ZOffset= Point2[2]-Point3[2]
XOffset= Point2[0]-Point3[0]
GMYOffset = (math.tan(XOffset/ZOffset))*(180/math.pi)
print (f"Winkelversatz um GMY: {GMYOffset}")

##########################################i######################################

# Speichern der Daten in ein CSV.
# Bennenung des Files mit Datum und Zeitstempel
rows = zip(OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_DMS_Tool_linearity.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)

# Bewegung der GMX Achse zurueck auf den Startwert
print("Move GMX back to Start Position")
epics.caput("X06MX-ES-DF1:GMX-INCP",-5)
time.sleep(2)

input("done.")


