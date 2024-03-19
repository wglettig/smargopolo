#!/usr/bin/env python3


# This Script shall demonstrate communicating to SmarGonMCS2 via the RESTful API
# provided by the smargopolo server
# Basic functions are demonstrated like:
# 1) See if the server is running, and if SmarGon is ready to move.
# 2) Get Current position in SCS (SmarGon Coordinate System)
# 3) Do a move to Absolute Position
# 4) Do a relative move


import requests 
import json
import time


# Set the smargopolo server address and port here:
smargopolo_server = "http://smargopolo:3000"

readbackMCS = None

# See if the server is running:
def getStatus ():
    global readbackMCS
    # Send a HTTP GET request to the RESTful API:
    print("Running: getStatus()")
    try:
        print("Getting HTTP GET request from "+smargopolo_server+"/readbackMCS")
        response = requests.get(smargopolo_server+"/readbackMCS") 
    except Exception as e:
        print ("Exception: ",e.__class__,"Failed to connect to "+smargopolo_server+"/readbackMCS")
        return False
    print("Ok, the server gives us the following response:")
    print(response.text)
    
    try: 
        print("See if the response is a JSON object:")
        json_txt = json.dumps(response.json(),indent = 4) # Pretty Print the JSON response
    except Exception as e:
        print ("Exception: ",e.__class__,"Response is not JSON")
        return False
    print (json_txt)

    try:
        print("Convert the JSON response into a python object:")
        readbackMCS = json.loads(response.text)
    except Exception as e:
        print("Exception: ", e.__class__,"Failed to convert JSON to python object")
        return False
    print("Ok, readbackMCS python object ready to use.") 
    return True

# A good way to see if the smargon is ready to be used is to check if it is in "MODE 2: READY (Follow Target)" 
def getMODE ():
    global readbackMCS
    print("Running: getMODE()")
    print("Let's see in which MODE the smargopolo controller is:")
    MODE = readbackMCS['mode']
    if (MODE == 0):
        print ("MODE 0: UNINITIALZED -> Should we initialize? (y/n)")
        if (input()=="y"):
            print ("Running initialization (Setting MODE 1: INITIALIZING...)")
            initialize()
            time.sleep(5)
            getMODE()
            MODE = readbackMCS['mode']
    if (MODE == 2):
        print ("MODE 2: READY (Follow Target) -> ready to roll!")
    return True

def initialize ():
    print("Running: initialize()")
    try:
        print("Setting MODE to 1: INITIALIZING...")
        response = requests.put(smargopolo_server+"/mode?mode=1") 
    except Exception as e:
        print ("Exception: ",e.__class__,"Failed to connect to "+smargopolo_server+"/mode?mode=1")
        return False
    return True

def doSequence():
    response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=0')
    time.sleep(0.5)
    response = requests.put(smargopolo_server+'/targetSCS?SHX=1&SHY=0')
    time.sleep(0.5)
    response = requests.put(smargopolo_server+'/targetSCS?SHX=1&SHY=1')
    time.sleep(0.5)
    response = requests.put(smargopolo_server+'/targetSCS?SHX=-1&SHY=1')
    time.sleep(0.5)
    response = requests.put(smargopolo_server+'/targetSCS?SHX=-1&SHY=-1')
    time.sleep(0.5)
    response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=-1')
    time.sleep(0.5)
    response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=0')
    time.sleep(0.5)
    
def main():
   getStatus()
   getMODE()

if __name__ == "__main__":
    main()


# 
# url = smargopolo_server + '/targetSCS?CHI=0'
# response = requests.put(url)
# time.sleep(1)
# 
# url = smargopolo_server + '/targetSCS?CHI=20'
# response = requests.put(url)
# time.sleep(2)
# 
# url = smargopolo_server + '/targetSCS?CHI=0'
# response = requests.put(url)
# time.sleep(2)
# 
# url = smargopolo_server + '/targetSCS?CHI=45'
# response = requests.put(url)
# time.sleep(3)
# 
# response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=0')
# time.sleep(0.5)
# response = requests.put(smargopolo_server+'/targetSCS?SHX=1&SHY=0')
# time.sleep(0.5)
# response = requests.put(smargopolo_server+'/targetSCS?SHX=1&SHY=1')
# time.sleep(0.5)
# response = requests.put(smargopolo_server+'/targetSCS?SHX=-1&SHY=1')
# time.sleep(0.5)
# response = requests.put(smargopolo_server+'/targetSCS?SHX=-1&SHY=-1')
# time.sleep(0.5)
# response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=-1')
# time.sleep(0.5)
# response = requests.put(smargopolo_server+'/targetSCS?SHX=0&SHY=0')
# time.sleep(0.5)
# 
# url = smargopolo_server + '/targetSCS?CHI=0'
# response = requests.put(url)
