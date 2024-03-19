#!/usr/bin/env python

# Bedingung: 
# 1_1.Ausrichtung des DMS Tools zu Aerotech mit Script: 	measure_DMS_Tool_angleError.py
# 1_2.Omega Achse ausgereichtet mit Script:			measure_Omega_Offset_to_Smargon.py
# 2_1.Die Kalibrierpin Vektoren ermittelt mit Script: 		measure_PinVector_SHX_SHY.py
# 2_2.Der Kalibrierpin Vektor ermittelt mit Script: 		measure_Offset_Q4_AND_PinVector_SHZ.py

# Dieses Script verfaehrt die Aeroetch Omega Achse um 360 Grad um den Fehler des Smargon in OX und OY yu ermitteln.

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
import json 

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
    OMEGA_Seq = data.header.seq
    OMEGA_Secs = data.header.stamp.secs
    OMEGA_Nsecs = data.header.stamp.nsecs
    

# Auselsen des Kalibriertools
# Auslesen Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA_inst)
def callback_readbackCAL_JointState(data):

    global OMEGA,DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs,CHI
    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(OMEGA_Secs)
    DMS_Nsecs.append(OMEGA_Nsecs)
    DMS_Seq.append(OMEGA_Seq)
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

# Ermittlung des OX Versatzes (DMY)
def calculate_correction_OX(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"Korrekturwert OX [mm]: 		{correction}")
 
# Ermittlung des OY Versatzes (DMZ)
def calculate_correction_OY(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"Korrekturwert OY [mm]: 		{correction}")

### START SCRIPT HERE #########################################################

# Labor oder Beamline Anwendung
#AEROTECH_EPICS_RECORD = "X06MX-ES-DF1"
AEROTECH_EPICS_RECORD = "X06SA-ES-DF1"

print ("Dieses Script verfaehrt die OMEGA Achse um 360 Grad,")
print ("und zeichnet den Fehler mittels des DMS Instrumentes auf")
print ("Achtung:")
print ("**Kalibriertool muss aktiviert und in Kontakt mit der Keramik- Kugel sein.**")
key=input ("OK? (y/n) ")
if (key != "y"):
    print ('Stopping script.')
    exit(code=None)

#Omega auf 0 Grad stellen
print ("Moving OMEGA to 0deg")
movebackSpeed = 40 # deg/s
currentOMEGA = epics.caget(AEROTECH_EPICS_RECORD + ":OMEGA-GETP")
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV", movebackSpeed)
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETP", 0)
time.sleep(currentOMEGA/movebackSpeed + 2)

#CHI auf 0 Grad stellen    
#response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
time.sleep(3)

# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
subsSCS  =rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)
print ("Starting data collection...")

# Start des Bewegungsablauf CHI0 Omega 360 Grad
dataCollectionSpeed = 20 #deg/s
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV",dataCollectionSpeed)
print (f"Data collection will take approx {1*(360/dataCollectionSpeed+5)} [s]")

print('**Move Omega 360 degree with CHI: 0 degree')   
#Move Omega 360 degree with CHI = 0
epics.caput(AEROTECH_EPICS_RECORD+ ":OMEGA-SETP",360)
time.sleep(360/dataCollectionSpeed + 5)

#get Aerotech
OmegaRDB=epics.caget(AEROTECH_EPICS_RECORD+ ":OMEGA-RBV")
OmegaRDB=round(OmegaRDB, 2)
    
#get Smargon
response = requests.get(smargopolo_server+"/readbackSCS")
readbackSCS = json.loads(response.text)
get_CHI=round(readbackSCS['CHI'])
	
# beenden der Datenaufzeichnung
rospy.signal_shutdown('finished measuring')

# Smargon zuruck in die Ausgangsposition
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV", movebackSpeed)
epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETP",0)
response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
print ("Stopped collecting data.")
################################################################################

# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z [mm]")
ax.set_ylabel("DMS_X [mm]")
ax.set_zlabel("DMS_Y [mm]")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0],label='StartPoint',marker=(5,0),markersize=15)
ax.set_title("3D Plot: Measure OX and OY Offset with CHI at 0",fontsize=14,fontweight="bold")
ax.legend()
set_axes_equal(ax)
fig.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
fig2 = plt.figure()
ax2=fig2.add_subplot(111)
#ax2.plot(OMEGA, DMS_X, label='DMS_X')
ax2.plot(OMEGA, DMS_Y, label='DMS_Y')
#ax2.plot(OMEGA, DMS_Z, label='DMS_Z')
ax2.set_xlabel("OMEGA")
ax2.set_title("2D Plot: Measure OX Offset with CHI 0",fontsize=14,fontweight="bold")
ax2.legend()
fig2.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
fig3 = plt.figure()
ax3=fig3.add_subplot(111)
#ax3.plot(OMEGA, DMS_X, label='DMS_X')
#ax3.plot(OMEGA, DMS_Y, label='DMS_Y')
ax3.plot(OMEGA, DMS_Z, label='DMS_Z')
ax3.set_xlabel("OMEGA")
ax3.set_title("2D Plot: Measure OY Offset with CHI 0",fontsize=14,fontweight="bold")
ax3.legend()
fig3.show()

################################################################################

calculate_correction_OX(DMS_Y)
calculate_correction_OY(DMS_Z)

################################################################################

print ("**Info: Die Korrekturwerte sollten im kleiner als 1um sein.**")

# Save Data to CSV
rows = zip(SCS_CHI,OMEGA, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_Smargon_Error_OXOY_CHI0.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["CHI","OMEGA", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)


input("done.")
