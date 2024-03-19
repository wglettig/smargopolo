#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#########EPICS COMMANDS

#read speed Omega Achse -> epics.caget("X06MX-ES-DF1:OMEGA-SETV")
#write speed Omega Achse -> epics.caput("X06MX-ES-DF1:OMEGA-SETV",20)
#
#write relative Omega Achse -> epics.caput("X06MX-ES-DF1:OMEGA-INCP",90)
#write absolute Omega Achse -> epics.caput("X06MX-ES-DF1:OMEGA-SETP",20)
#
#read ReadBAck Omega Achse -> epics.caget("X06MX-ES-DF1:OMEGA-RBV")
#read User ReadBAck Omega Achse -> epics.caget("X06MX-ES-DF1:OMEGA-GETP")
#
#GMX, GMY, GMZ

#Created on Tue May 18 18:03:02 2021

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import requests
import json 
import epics
import time
import csv
from datetime import date
import sys
import math
import smaract.ctl as ctl
import matplotlib.pyplot as plt
import numpy as np

#temporary subscriber to check function
DMS=[]
Position=[]
Secs=[]
Nsecs=[]
Seq=[]
Liste=[]
LJU6_JointStateOmega=0
CHI_Cal=0
PHI_Cal=0
get_CHI=0
get_PHI=0
OmegaRDB=0

#neccessary subscriber to put in csv
CHI=[]
PHI=[]
SHX=[]
SHY=[]
SHZ=[]
OX=[]
OY=[]
OZ=[]

p_x = []
p_y = []
p_z = []
#d_handle
    
readbackCHI = 0
readbackPHI = 0

Counter = 0
motion=True
loop = 0
#now = date.time()
today = date.today()
# dd/mm/YY
day = today.strftime("%Y_%m_%d_")
#print("d1 =", d1)
smargopolo_server = "http://smargopolo:3000"

def init_Smargon():
    smargopolo_server = "http://smargopolo:3000"
    response = requests.get(smargopolo_server+"/readbackSCS") 
#   response = requests.get(smargopolo_server+"/readbackMCS") 

def create_CSV_File():
     #Writing CSV File from List 
    #generate new CSV fileLJU6_JointStateTimeSekLJU6_JointStateTimeSek
    with open(day+'SmargonError.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["OMEGA","Sensor1","Sensor2","Sensor3","spare1","spare2","Sequenz","Sekunden","NanoSekunden","CHI","PHI","Counter","DMS_X","DMS_Y","DMS_Z"]) 
 

def assert_lib_compatibility():
    """
    Checks that the major version numbers of the Python API and the
    loaded shared library are the same to avoid errors due to 
    incompatibilities.
    Raises a RuntimeError if the major version numbers are different.
    """
    vapi = ctl.api_version
    vlib = [int(i) for i in ctl.GetFullVersionString().split('.')]
    if vapi[0] != vlib[0]:
        raise RuntimeError("Incompatible SmarActCTL python api and library version.")

def transform(ch0, ch1, ch2):
    """
    Coordinate transformation from calibrator channel positions to X, Y, Z coordinates
    """
    RX = math.radians(-45)
    RZ = math.radians(-54.736)
    x = -ch1 * math.sin(RX) * math.sin(RZ) + ch0 * math.cos(RX) * math.sin(RZ) + ch2 * math.cos(RZ)
    y =  ch2 * math.sin(RZ) + ch1 * math.sin(RX) * math.cos(RZ) - ch0 * math.cos(RX) * math.cos(RZ)
    z = -ch0 * math.sin(RX) - ch1 * math.cos(RX)
    return x, y, z


def init_DMS():
    
    global d_handle
    
    version = ctl.GetFullVersionString()
    print("SmarActCTL library version: '{}'.".format(version))
    assert_lib_compatibility()

    locator = "usb:sn:MCS2-00006300"
    d_handle = None

    try:
        d_handle = ctl.Open(locator)
        print("MCS2 opened {}.".format(locator))
        sensorCheck = True
        for channel in range(3):
            
            state = ctl.GetProperty_i32(d_handle, channel, ctl.Property.CHANNEL_STATE)
            if not state & ctl.ChannelState.SENSOR_PRESENT:
                sensorCheck = False
        if not sensorCheck:
            print("MCS2 three channels with sensors are needed, abort.")
            sys.exit(1)
    
        # note: for completeness we set all relevant properties here. Generally, most of them are persistent
        # and need not be configured on every power-up
    
        # configure IO module (+-10V range)
        
        
        ctl.SetProperty_i32(d_handle, 0, ctl.Property.IO_MODULE_ANALOG_INPUT_RANGE, ctl.IOModuleAnalogInputRange.IO_MODULE_ANALOG_INPUT_RANGE_BI_10V)
    
        for channel in range(3):
            # sensors must be enabled (not in power save mode)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.SENSOR_POWER_MODE, ctl.SensorPowerMode.ENABLED)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.MAX_CL_FREQUENCY, 12000) # Hz
    
            # define input to be used as feedback
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.AUX_IO_MODULE_INPUT_INDEX, 0)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.AUX_INPUT_SELECT, ctl.AuxInputSelect.IO_MODULE)
            if channel == 2:
                ctl.SetProperty_i32(d_handle, channel, ctl.Property.AUX_DIRECTION_INVERSION, ctl.INVERTED)
            else:
                ctl.SetProperty_i32(d_handle, channel, ctl.Property.AUX_DIRECTION_INVERSION, ctl.NON_INVERTED)
                # route aux input into control loop
                ctl.SetProperty_i32(d_handle, channel, ctl.Property.CONTROL_LOOP_INPUT, ctl.ControlLoopInput.AUX_IN)
    
            # unlock write access to tune the (custom) positioner type
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_WRITE_PROTECTION, ctl.PosWriteProtection.KEY)
            # the endstop detection must be disabled
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_ESD_DIST_TH, 0)
            # set control loop gains
            # TODO tune to best performance
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_PID_SHIFT, 1)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_P_GAIN, 1000)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_D_GAIN, 100)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_I_GAIN, 0)
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_ANTI_WINDUP, 0)
    
            # optionally, the modified positioner type may be saved to a custom slot and
            # may be selected to be used for the aux input to define the control loop settings
            # this will be needed if switching between aux control and regular position feedback is needed, e.g.
            # to move tplot_data()he stage with regular closed-loop movement to a starting position
            #ctl.SetProperty_i32(d_handle, channel, ctl.Property.POS_SAVE, ctl.PositionerType.CUSTOM0)
            #ctl.SetProperty_i32(d_handle, channel, ctl.Property.AUX_POSITIONER_TYPE, ctl.PositionerType.CUSTOM0)
    
            # note: setting the position refers to the position calculation for the build-in sensors
            ctl.SetProperty_i64(d_handle, channel, ctl.Property.POSITION, 0)
    
            # note: the commanded target is in adc increments now and *not* in 'position'
            # the range is -131072 to 131071 corresponding to +-10.24V input voltage
            # the amplifier box should be offset trimmed to generate ca. ADC = 0 with unstressed force sensors
    
            # the current adc value may be read with 'Control Loop Input Aux Value'
            aux = ctl.GetProperty_i64(d_handle, channel, ctl.Property.CL_INPUT_AUX_VALUE)
            print("AUX value: {} (channel {})".format(aux, channel))
    
            target = 32000 #int(131071 / 4) # ca 25% of the (positive) control range to 'press' against the sensors
            ctl.SetProperty_i32(d_handle, channel, ctl.Property.MOVE_MODE, ctl.MoveMode.CL_ABSOLUTE)
            ctl.Move(d_handle, channel, target) # enable control loop

#         the position offset may now be read with:
#         pos = ctl.GetProperty_i64(d_handle, 0, ctl.Property.CL_INPUT_SENSOR_VALUE)
            
    except ctl.Error as e:
        print("MCS2 {}: {}, error: {} (0x{:04X}) in line: {}."
          .format(e.func, ctl.GetResultInfo(e.code), ctl.ErrorCode(e.code).name, e.code, (sys.exc_info()[-1].tb_lineno)))

    except Exception as ex:
        print("Unexpected error: {}, {} in line: {}".format(ex, type(ex), (sys.exc_info()[-1].tb_lineno)))
        raise

    time.sleep(3)

def move_to_ZERO_Start():
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
    
def move_to_ZERO_Stopp():
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
        
    ctl.Stop(d_handle,2)
    ctl.Stop(d_handle,1)
    ctl.Stop(d_handle,0)
    ctl.Close(d_handle)
    #get Position Feedback
    readbackSCS = json.loads(response.text)
    CHI_Cal=readbackSCS['CHI']
    PHI_Cal=readbackSCS['PHI']
    OmegaRDB=epics.caget("X06MX-ES-DF1:OMEGA-RBV")
    OmegaRDB=round(OmegaRDB, 2)
    print("Omega:", OmegaRDB,"    CHI: ",CHI_Cal,"    PHI: ",PHI_Cal)
    print("==Move to ZERO -> DONE============")
    print("")
    
    
def callback_LJU6_JointState(data):
    
    global Nsecs,Secs,Seq,Liste,CHI,PHI,Position,Counter,p_x,p_y,p_z,loop
    
    #if loop % 25:#nur jedeszweite Werte werden geschrieben (200Hz :2 )also 100Hz
        
#    start_time = time.time() 
    
    ch0 = ctl.GetProperty_i64(d_handle, 0, ctl.Property.CL_INPUT_SENSOR_VALUE)
    ch1 = ctl.GetProperty_i64(d_handle, 1, ctl.Property.CL_INPUT_SENSOR_VALUE)
    ch2 = ctl.GetProperty_i64(d_handle, 2, ctl.Property.CL_INPUT_SENSOR_VALUE)
    p_x, p_y, p_z = transform(ch0, ch1, ch2)
    
 
    
    Position=(data.position)
    Secs=(data.header.stamp.secs)
    Nsecs=(data.header.stamp.nsecs)
    Seq=(data.header.seq)
    Liste=list(Position) 
        
    #response = requests.get(smargopolo_server+"/readbackSCS")
    #readbackSCS = json.loads(response.text)
    #CHI=readbackSCS['CHI']
    #PHI=readbackSCS['PHI']
    CHI=readbackCHI
    PHI=readbackPHI
    
    Liste.extend([Seq,Secs,Nsecs,CHI,PHI,Counter,p_x,p_y,p_z])
#       
#        print(loop)

    with open(day+'SmargonError.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(Liste) 
    
#    looptime= time.time() - start_time
#    print(looptime)    
    #loop+=1

def callback_readbackSCS_JointState(data):   
    global readbackCHI, readbackPHI
    readbackCHI = data.postion[4];
    readbackPHI = data.postion[5];

        
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
                
        
    #////////////////////////////////////////////////
    
if __name__ == '__main__':   
    
#    smargopolo_server = "http://smargopolo:3000"
#    response = requests.get(smargopolo_server+"/readbackSCS")
#    readbackMCS = json.loads(response.text)
    
    init_Smargon()
    create_CSV_File()
    move_to_ZERO_Start()
    init_DMS()
    rospy.init_node('listener', anonymous=True)
    subs1=rospy.Subscriber("/LJU6_JointState", JointState, callback_LJU6_JointState)
    subs2=rospy.Subscriber("/readbackSCS", JointState, callback_readbackSCS_JointState)
    
    for loop in range(0,5,1):#0,5,1 = 0-40Grad
        start_Calibration_routine(loop)
    subs1.unregister()      
#   rospy.spin()
    move_to_ZERO_Stopp()

    


        
        
        




