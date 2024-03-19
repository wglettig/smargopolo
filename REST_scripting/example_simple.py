#!/usr/bin/env python3

# This Script is a simple demonstration of controlling SmarGonMCS2 
# via the RESTful API provided by the smargopolo server
#
# Basic functions are demonstrated like:
# 1) See if the server is running, and if SmarGon is ready to move.
# 2) Get current position in SCS (SmarGon Coordinate System)
# 3) Do a move to an absolute position
# 4) Do a relative move
#
# Wayne Glettig, 17.02.2021

import requests 
import json
import time
from sys import exit


# Set the smargopolo server address and port here:
smargopolo_server = "http://smargopolo:3000"

##########################################################################################################
# 1) See if the server is running and if SmarGon is ready to move:
print("Getting HTTP GET request from "+smargopolo_server+"/readbackMCS")
response = requests.get(smargopolo_server+"/readbackMCS") 

print("Ok, the server gives us the following response:")
json_txt = json.dumps(response.json(),indent = 4) # Pretty Print the JSON response
print (json_txt)

#Convert the JSON response into a python object for further use
readbackMCS = json.loads(response.text) 

#We can print the serial number of the SmarAct MCS2 Controller
print ('SmarAct MCS2 S/N: "'+ readbackMCS['serial_number'] +'"')

#We can print the last message was logged on ROS (/rosout)
print ('Last rosout Message: "'+ readbackMCS['rosout']['msg'] +'"')

#We can check if the loop counter is counting, so if the internal smargopolo feedback loop 
#in ROS is running correctly
loopcounter1 = readbackMCS['seq']
time.sleep(1)
response = requests.get(smargopolo_server+"/readbackMCS") 
readbackMCS = json.loads(response.text) 
loopcounter2 = readbackMCS['seq']
if (loopcounter2 > loopcounter1):
   print('Loopcounter counting correctly. Good.')
else:
   print('Loopcounter not counting. Not good. ROS control loop has an issue. Stopping script.')
   exit(0)

# A good way to see if the smargon is ready to be used is to check if it is in "MODE 2: READY (Follow Target)" 
print("Let's see in which MODE the smargopolo controller is:")
readbackMCS = json.loads(response.text) #Convert the JSON response into a python object
MODE = readbackMCS['mode']
if (MODE == 0):
    print ("MODE 0: UNINITIALZED -> Should we initialize? (y/n)")
    if (input()=="y"):
        print ("Running initialization (Setting MODE 1: INITIALIZING...)")
        response = requests.put(smargopolo_server+"/mode?mode=1")
        while (MODE != 2): #keep polling the server until MODE = 2
            time.sleep(1)
            response = requests.get(smargopolo_server+"/readbackMCS") 
            readbackMCS = json.loads(response.text) #Convert the JSON response into a python object
            MODE = readbackMCS['mode']
    else:
        exit(0)
if (MODE == 2):
    print ("MODE 2: READY (Follow Target) -> ready to roll!")

##########################################################################################################
# 2) Get current position in SCS (SmarGon Coordinate System)
print ("Get Current position in SCS (SmarGon Coordinate System):")
response = requests.get(smargopolo_server+"/readbackSCS") 
print (json.dumps(response.json(),indent = 4)) # Pretty Print the JSON response

# Again here, the JSON response can be converted into a python object for further use
readbackSCS = json.loads(response.text) 
SHZ = readbackSCS['SHZ']
print("The readback SHZ value is:" , SHZ)

##########################################################################################################
# 3) Do a move to an absolute position
print("Moving to Home Position")
response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=0&SHZ=18&CHI=0&PHI=0&OX=0&OY=0&OZ=180')

print (json.dumps(response.json(),indent = 4)) # optionally, pretty-print the JSON response
# Note that the JSON response is the confirmed target position, not the readback position.
# If you need a confirmation that SmarGon actually moved to a certain position,
# You will have to request the current readback position and check that.

time.sleep(2)

##########################################################################################################
# 4) Do a few relative moves
print("Doing relative moves")
response = requests.put(smargopolo_server+'/targetSCS_rel?SHX=0.1')
time.sleep(0.5)
response = requests.put(smargopolo_server+'/targetSCS_rel?SHY=0.1')
time.sleep(0.5)
response = requests.put(smargopolo_server+'/targetSCS_rel?SHX=-0.2')
time.sleep(0.5)
response = requests.put(smargopolo_server+'/targetSCS_rel?SHY=-0.2')
time.sleep(0.5)
response = requests.put(smargopolo_server+'/targetSCS_rel?SHX=0.2')
time.sleep(0.5)
response = requests.put(smargopolo_server+'/targetSCS_rel?SHY=0.1')
time.sleep(0.5)
response = requests.put(smargopolo_server+'/targetSCS_rel?SHX=-0.1')
time.sleep(0.5)

##########################################################################################################
time.sleep(1)
print("And now back home")
response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=0&SHZ=18&CHI=0&PHI=0&OX=0&OY=0&OZ=180')
time.sleep(2)

print("Ok, All done.")
