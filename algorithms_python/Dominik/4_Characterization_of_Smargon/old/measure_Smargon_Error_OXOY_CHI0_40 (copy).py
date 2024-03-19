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
OMEGA=[];
OMEGA_Cont=[];
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

    global OMEGA_inst
    OMEGA_inst = data.position[0]

# Auselsen des Kalibriertools
# Auslesen Hinzufuegen eines Zeitstempels
# Einbinden des Aerotech Omegawinkels (OMEGA_inst)
def callback_readbackCAL_JointState(data):

    global OMEGA,OMEGA_Const,DMS_X,DMS_Y,DMS_Z,DMS_Seq,DMS_Secs,DMS_Nsecs,CHI
    DMS_X.append(data.position[0])
    DMS_Y.append(data.position[1])
    DMS_Z.append(data.position[2])
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    DMS_Seq.append(data.header.seq)
    OMEGA.append(OMEGA_inst)
    SCS_CHI.append(CHI)
    OMEGA_Cont.append(OMEGA_inst+(BlaBla*360))

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

# Ermittlung des OX Versatzes (DMY)
def calculate_correction_OX(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"CORRECTION value for OX: 	{correction}")
 
# Ermittlung des OY Versatzes (DMZ)
def calculate_correction_OY(VECT):
    current = VECT[0]
    centre = (max(VECT) + min(VECT))/2.
    correction = -(current-centre)
    #print (f"MAX=:				{max(VECT)}, MIN= {min(VECT)}")
    #print (f"current:				{current}")
    #print (f"centre: 				{centre}")
    print (f"CORRECTION value for OY: 	{correction}")

### START SCRIPT HERE #########################################################

# Labor oder Beamline Anwendung
AEROTECH_EPICS_RECORD = "X06MX-ES-DF1"
#AEROTECH_EPICS_RECORD = "X06SA-ES-DF1"

print ("This script rotates the Aerotech OMEGA axis from 0-360deg,")
print ("and records the calibration tool position during this motion.")
print ("Make sure:")
print ("* OMEGA is ready to turn freely.")
print ("* Calibration Tool is ready, in contact and feedback is active.")
key=input ("OK to continue? (y/n) ")
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


# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
subsSCS  =rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)
print ("Starting data collection...")

def start_Calibration_routine(loop):
    
    global Counter,get_CHI,BlaBla
    
    smargopolo_server = "http://smargopolo:3000"
    dataCollectionSpeed = 20 #deg/s
    epics.caput(AEROTECH_EPICS_RECORD + ":OMEGA-SETV",dataCollectionSpeed)
    
    if loop==0:
        CHI_Cal = loop
        Omega_Cal = 360
        
        print("")
        print (f"Data collection will take approx {4*(360/dataCollectionSpeed+20)} [s]")
    else:
        CHI_Cal = loop *10
        Omega_Cal = (loop+1)*360 
    BlaBla=loop
    	
    print('**Move Omega 360 degree with CHI: '+ str(CHI_Cal))
    
    #First Move CHI PHI = 0    
    response = requests.put(smargopolo_server+'/targetSCS?CHI='+ str(CHI_Cal))
    response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
    time.sleep(3)
    
    #Move Omega 360 degree with CHI + PHI = 0
    epics.caput(AEROTECH_EPICS_RECORD+ ":OMEGA-SETP",Omega_Cal)
    time.sleep(360/dataCollectionSpeed + 5)

    #get Aerotech
    OmegaRDB=epics.caget(AEROTECH_EPICS_RECORD+ ":OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    
    #get Smargon
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    get_CHI=round(readbackSCS['CHI'])
    #get_PHI=round(readbackSCS['PHI'])  

    print("Omega:", OmegaRDB,"    CHI: ",get_CHI)
    print("")


#Start Motion (CHI 0-40 degree)
for loop in range(0,5,1):#0,5,1 = 0-40Grad
	start_Calibration_routine(loop)
	
	
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
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'rx')
ax.set_title("3D Plot: Measure OX and OY Offset with CHI 0 to 40",fontsize=14,fontweight="bold")
set_axes_equal(ax)
fig.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
fig2 = plt.figure()
ax2=fig2.add_subplot(111)
#ax2.plot(OMEGA, DMS_X, label='DMS_X')
ax2.plot(OMEGA, DMS_Y, label='DMS_Y')
#ax2.plot(OMEGA, DMS_Z, label='DMS_Z')
ax2.set_xlabel("OMEGA")
ax2.set_title("3D Plot: Measure OX and OY Offset with CHI 0 to 40",fontsize=14,fontweight="bold")
ax2.legend()
fig2.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
fig3 = plt.figure()
ax3=fig3.add_subplot(111)
#ax3.plot(OMEGA, DMS_X, label='DMS_X')
#ax3.plot(OMEGA, DMS_Y, label='DMS_Y')
ax3.plot(OMEGA, DMS_Z, label='DMS_Z')
ax3.set_xlabel("OMEGA")
ax3.set_title("2D Plot: Measure OY Offset",fontsize=14,fontweight="bold")
ax3.legend()
fig3.show()

################################################################################

calculate_correction_OX(DMS_Y)
calculate_correction_OY(DMS_Z)

################################################################################
# Save Data to CSV

rows = zip(SCS_CHI,OMEGA,OMEGA_Cont, DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_Smargon_Error_OXOY_CHI0_40.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["CHI","OMEGA","OMEGA_Cont", "DMS_X", "DMS_Y", "DMS_Z", "DMS_Seq", "DMS_Secs", "DMS_Nsecs"])
    for row in rows:
        writer.writerow(row)


input("done.")
