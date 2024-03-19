#!/usr/bin/env python

# Bedingung: 
# 1. Ausrichtung des DMS Tools zu Aerotech mit Script: 210827_measure_DMS_Tool_linearity.py
# 2. Omega Achse ausgereichtet mit Script:210827_measure_Omega_Offset_to_Smargon.py
# 3. Die Kalibrierpin Vektoren wurden durch das Script: measure_PinVector_SHX_SHY.py ermittelt
# 4. Der Kalibrierpin Vektor wurde durch das Script: measure_PinVector_SHZ_2.py ermittelt
# 5. Smargon OX und OY wurden anahnd des Scripts: measure_Smargon_Error_OXOY.py ermittelt

# Dieses Script verfaehrt die Omega Achse und Smargon (CHI 0 -40 Grad) um das Fehlerbild zu bestimmen
# Anhand der erhaltenen Daten wird der Fehler analysiert und im naechsten Schritt kompensiert 

# Das Fehlerbild sollte moeglichst klein sein

# Dominik Buntschu, 28.8.2021

import rospy
from sensor_msgs.msg import JointState
import requests
import json 
import epics
import csv
import time
from datetime import date
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

DMS_X=[];
DMS_Y=[];
DMS_Z=[];
DMS_Seq=[];
DMS_Secs=[];
DMS_Nsecs=[];
OMEGA=[];
OMEGA_inst=0
SCS_CHI=[];
CHI =0
SCS_PHI=[];
PHI =0
Counter = 0

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
    DMS_Seq.append(data.header.seq)
    DMS_Secs.append(data.header.stamp.secs)
    DMS_Nsecs.append(data.header.stamp.nsecs)
    OMEGA.append(OMEGA_inst)
    SCS_CHI.append(CHI)
    SCS_PHI.append(PHI)

# Auslesen des Chi Winkels von Smargon
def callback_readbackSCS_JointState(data):
    global CHI,PHI
    CHI = data.position[4]
    PHI = data.position[5]

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
 

##### Functions ##############################################################   
def move_to_Start_Position():
    #Move to Start
    print("==Move to ZERO============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS") 
    #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    CHI_Cal=readbackSCS['CHI']
    PHI_Cal=readbackSCS['PHI']
    
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
    print("==Move to ZERO -> DONE============")
    print("")

def start_Calibration_routine(loop):
    
    global Counter,get_CHI,get_PHI
    
    if loop==0:
        CHI_Cal = loop
        Omega_Cal = 360
    else:
        CHI_Cal = loop *10
        Omega_Cal = (loop+1)*360 
        
    Counter= loop
    
    #get Aerotech
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    #get Smargon
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    readbackSCS = json.loads(response.text)
    get_CHI=round(readbackSCS['CHI'])
    get_PHI=round(readbackSCS['PHI'])  
    print('============Move 360degree with CHI: '+ str(CHI_Cal))
    #First Move CHI PHI = 0    
    response = requests.put(smargopolo_server+'/targetSCS?CHI='+ str(CHI_Cal))
    response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
    time.sleep(3)
    #Move Omega 360 degree with CHI + PHI = 0
    epics.caput("X06MX-ES-DF1:OMEGA-SETP",Omega_Cal)
    time.sleep(8)
    
    get_CHI=round(readbackSCS['CHI'])
    get_PHI=round(readbackSCS['PHI'])
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",get_CHI,"    PHI: ",get_PHI)
    print("")

def move_back_to_StartPosition():
    #Move back to Start Position
    print("==Move to ZERO============")
    
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS")
    
    #Motion moving to yero Position
    epics.caput("X06MX-ES-DF1:OMEGA-SETV",50)
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    
    if OmegaRDB > 180 or OmegaRDB < -180:
        print("Omega > 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(40)
    else:
        print("Omega < 180")
        epics.caput("X06MX-ES-DF1:OMEGA-SETP",0)
        response = requests.put(smargopolo_server+'/targetSCS?CHI=0')
        response = requests.put(smargopolo_server+'/targetSCS?PHI=0')
        time.sleep(5)
        
        #get Position Feedback
    readbackSCS = json.loads(response.text)
    CHI_Cal=readbackSCS['CHI']
    PHI_Cal=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
    print("==Move to ZERO -> DONE============")
    print("")
                
##### Start Routine ######################################################
print ("This script rotates the Aerotech OMEGA axis from 0-360deg and CHI from 0-40 degree,")
print ("and records the calibration tool position during this motion.")
print ("Make sure:")
print ("* OMEGA is ready to turn freely.")
print ("* Calibration Tool is ready, in contact and feedback is active.")
key=input ("OK to continue? (y/n) ")
if (key != "y"):
    print ('Stopping script.')
    exit(code=None)

        
#Definiert Variabel fuer den Smargon Server
smargopolo_server = "http://smargopolo:3000"

#Move to Start (CHI 0, Omega 0)
move_to_Start_Position()

# starten der Datenaufzeichnung
rospy.init_node('DMS_Recorder', anonymous=True)
subsOMEGA=rospy.Subscriber("/LJUE9_JointState", JointState, callback_LJUE9_JointState)
subsDMS  =rospy.Subscriber("/readbackCAL", JointState, callback_readbackCAL_JointState)
subsSCS  =rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)

#Run Loop with Omega and CHI Motion
for loop in range(0,5,1):#0,5,1 = 0-40Grad
        start_Calibration_routine(loop)    

# beenden der Datenaufzeichnung
rospy.signal_shutdown('finished measuring')

#Move back to Start (CHI 0, Omega 0)
move_back_to_StartPosition()
    
##############################################################################

# 3D Plot der Bewegung mit den Werten des Kalibirertools
# Start der Bewegung = x Punkt
# Endpunkt der Bewegung = o Punkt
fig = plt.figure()
ax=fig.add_subplot(111, projection='3d')
ax.plot(DMS_Z,DMS_X,DMS_Y)
ax.set_xlabel("DMS_Z")
ax.set_ylabel("DMS_X")
ax.set_zlabel("DMS_Y")
ax.plot(DMS_Z[0:1],DMS_X[0],DMS_Y[0], 'rx')
set_axes_equal(ax)
fig.show()

# 2D Plot der Bewegung mit den Werten des Kaibriertools
fig2 = plt.figure()
ax2=fig2.add_subplot(111)
#ax2.plot(DMS_X, label='DMS_X')
ax2.plot(DMS_Y, label='DMS_Y')
ax2.plot(DMS_Z, label='DMS_Z')
ax2.legend()
fig2.show()

#Save Data
rows = zip(OMEGA,SCS_CHI,SCS_PHI,DMS_X, DMS_Y, DMS_Z, DMS_Seq, DMS_Secs, DMS_Nsecs)
with open(current_day + current_time +'measure_Move_Omega360_CHI0_40.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["OMEGA","SCS_CHI","SCS_PHI","DMS_X","DMS_Y","DMS_Z","DMS_Seq","DMS_Secs","DMS_Nsecs"])     
    for row in rows:
        writer.writerow(row)


    


        
        
        



