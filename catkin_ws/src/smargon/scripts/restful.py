#! /usr/bin/env python

# This example demonstrates ROS working with Flask, flask_restful
# and rospkg. How to install:
# pip install -U rospkg
################################################################################
# This script is based on the answers.ros.org post at:
# [1] https://answers.ros.org/question/234418/easiest-way-to-implement-http-server-that-can-send-ros-messages/
#
# Wayne Glettig 19.06.2021
################################################################################
# REST API USE: 
# Ressource /mode:
# GET: 127.0.0.1:3000/mode
# PUT: 127.0.0.1:3000/mode?mode=1
# 
# Ressource /corr_type:
# GET: 127.0.0.1:3000/corr_type
# PUT: 127.0.0.1:3000/corr_type?corr_type=1
#
# Ressource /targetBCS:
# GET: 127.0.0.1:3000/targetBCS
# PUT: 127.0.0.1:3000/targetBCS?BX=0.001&BY=0.002
#
# Ressource /targetBCS_rel:
# GET: 127.0.0.1:3000/targetBCS_rel
# PUT: 127.0.0.1:3000/targetBCS_rel?BX=0.01
#
# Ressource /targetSCS:
# GET: 127.0.0.1:3000/targetSCS
# PUT: 127.0.0.1:3000/targetSCS?SHX=0.001&SHY=0.002
#
# Ressource /targetSCS_rel:
# GET: 127.0.0.1:3000/targetSCS_rel
# PUT: 127.0.0.1:3000/targetSCS_rel?SHX=0.01
#
# Ressource /targetMCS:
# GET: 127.0.0.1:3000/targetMCS
# PUT: 127.0.0.1:3000/targetMCS?q1=0.001&q2=0.002
#
# Ressource /targetMCS_rel:
# GET: 127.0.0.1:3000/targetMCS_rel
# PUT: 127.0.0.1:3000/targetMCS_rel?q1=0.01
#
# Ressource /nudgeBCS:
# PUT: 127.0.0.1:3000/nudgeBCS?BX=0.001&BY=0.002
#
# Ressource /nudgeOBCS:
# PUT: 127.0.0.1:3000/nudgeOBCS?BX=0.001&BY=0.002
#
# Ressource /readbackBCS:
# GET: 127.0.0.1:3000/readbackBCS
#
# Ressource /readbackSCS:
# GET: 127.0.0.1:3000/readbackSCS
#
# Ressource /readbackMCS:
# GET: 127.0.0.1:3000/readbackMCS
#
################################################################################
# NOTES:
# The MODES of SmarGonMCS2 are:
#   0: UNINITIALIZED 
#   1: REFERENCING...
#   2: READY (Follow Target)
#  99: ERROR
#
# The ressources used to set axis values can address one or multiple of the 
# following axis names:
#   in BCS Frame: (Beamline Coordinates)
#     BX, BY, BZ
#   in SCS Frame: (SmarGonMCS2 Coordinates)
#     SHX, SHY, SHZ, OMEGA, CHI, PHI, OX, OY, OZ
#   in MCS Frame: (motor axes)
#     q1, q2, q3, q4, q5 
#
# Linear values are all in mm, and angles in deg.
# 
# Response status codes are usually 202:Accepted, as usually the request is 
# merely placed, and the API is not blocked until motion is completed.
#
# Response to a GET command is a JSON snippet with the current values.
# Response to a PUT command is a the updated values.
# Example of response from a PUT /targetMCS?SHX=0.1:
# {
#   "SHX": 0.1
#   "SHY": 0.0
#   "SHZ": 0.0
#   "OMEGA": 0.0
#   "CHI": 0.0
#   "PHI": 0.0
#   "OX": 0.0
#   "OY": 0.0
#   "OZ": 300.0
# }
# The Responses to the /target..._rel ressources will be the absolute values
#
################################################################################
# EXAMPLES:
#  PUT: 127.0.0.1:3000/mode?mode=1
#  * Starts Referencing (Sets the MODE to 1)
#
#  PUT: 127.0.0.1:3000/targetBCS?BX=0.01&BY=0.02
#  * Sets the target position of BX & BY (in Beamline Coordinate System (BCS)
#
#  PUT: 127.0.0.1:3000/targetSCS?CHI=20
#  * Sets the CHI to 20 [deg]
#
#
################################################################################
# ROS Topics:
# Subscribed to:
# /mode
# /readbackBCS
# /readbackSCS
# /readbackMCS
# /status
# Publishes to:
# /targetBCS_request
# /targetSCS_request
# /targetMCS_request
# /mode_request
################################################################################

#RESTful API related libraries:
import os
from copy import copy
import threading
from flask import Flask, request
from flask_restful import Api, Resource, reqparse
from flask_cors import CORS
try:
    import redis
except:
    print("restful.py: redis not found. Running without redis.")

#Redis setup
try:
    redis_handle = redis.Redis(host="x06sa-cons-705.psi.ch", db=3)
except:
    print("unable to connect to redis")

#ROS related libraries
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Log
##from wiimote.msg import State

#General libraries
import math
from datetime import datetime
import time

#Limit werkzeug Logging
import logging
logger = logging.getLogger('werkzeug')
logger.setLevel(logging.ERROR)


#ROS: This is the thread in which the ROS node is run (according to [1])
threading.Thread(target=lambda: rospy.init_node('restful', 
                                     disable_signals=True)).start()

#Global Variables (upon loading, read from ROS param server)
mode_request = 0
corr_type = rospy.get_param('default_active_correction')

#targets: (upon loading, read from ROS param server)
targetBCS = {
    "BX"   : 0.,
    "BY"   : 0.,
    "BZ"   : 0.,
    "OMEGA": 0.,
    "CHI"  : 0.,
    "PHI"  : 0.,
}
targetSCS = {
    "SHX"  :rospy.get_param('/axis/SHX/start'),
    "SHY"  :rospy.get_param('/axis/SHY/start'),
    "SHZ"  :rospy.get_param('/axis/SHZ/start'),
    "OMEGA":rospy.get_param('/axis/OMEGA/start'),
    "CHI"  :rospy.get_param('/axis/CHI/start'),
    "PHI"  :rospy.get_param('/axis/PHI/start'),
    "OX"   :rospy.get_param('/axis/OX/start'),
    "OY"   :rospy.get_param('/axis/OY/start'),
    "OZ"   :rospy.get_param('/axis/OZ/start'),
}
targetSCS_limpos = {
    "SHX"  :rospy.get_param('/axis/SHX/limpos'),
    "SHY"  :rospy.get_param('/axis/SHY/limpos'),
    "SHZ"  :rospy.get_param('/axis/SHZ/limpos'),
    "OMEGA":rospy.get_param('/axis/OMEGA/limpos'),
    "CHI"  :rospy.get_param('/axis/CHI/limpos'),
    "PHI"  :rospy.get_param('/axis/PHI/limpos'),
    "OX"   :rospy.get_param('/axis/OX/limpos'),
    "OY"   :rospy.get_param('/axis/OY/limpos'),
    "OZ"   :rospy.get_param('/axis/OZ/limpos'),
}
targetSCS_limneg = {
    "SHX"  :rospy.get_param('/axis/SHX/limneg'),
    "SHY"  :rospy.get_param('/axis/SHY/limneg'),
    "SHZ"  :rospy.get_param('/axis/SHZ/limneg'),
    "OMEGA":rospy.get_param('/axis/OMEGA/limneg'),
    "CHI"  :rospy.get_param('/axis/CHI/limneg'),
    "PHI"  :rospy.get_param('/axis/PHI/limneg'),
    "OX"   :rospy.get_param('/axis/OX/limneg'),
    "OY"   :rospy.get_param('/axis/OY/limneg'),
    "OZ"   :rospy.get_param('/axis/OZ/limneg'),
}
targetMCS = {
    "q1":rospy.get_param('/axis/q1/start'),
    "q2":rospy.get_param('/axis/q2/start'),
    "q3":rospy.get_param('/axis/q3/start'),
    "q4":rospy.get_param('/axis/q4/start'),
    "q5":rospy.get_param('/axis/q5/start'),
    "q6":rospy.get_param('/axis/q6/start'),
}
#readbacks:
readbackBCS = {
    "BX":0.0,
    "BY":0.0,
    "BZ":0.0,
    "OMEGA": 0.,
    "CHI"  : 0.,
    "PHI"  : 0.,
    "seq":0.0,
    "secs":0.0,
    "nsecs":0.0
}
readbackSCS = {
    "SHX":0.0,
    "SHY":0.0,
    "SHZ":0.0,
    "OMEGA":0.0,  #readbackSCS also contains OMEGA readback
    "CHI":0.0,
    "PHI":0.0,
    "OX":0.0,
    "OY":0.0,
    "OZ":0.0,
    "seq":0.0,
    "secs":0.0,
    "nsecs":0.0
}
readbackMCS = {
    "mode":0,  #readbackMCS also contains current MODE
    "corr_type":0,  #readback MCS also contains current corr_type
    "serial_number":"",
    "seq":0,
    "secs":0,
    "nsecs":0,
    "state":{
        "q1":"",
        "q2":"",
        "q3":"",
        "q4":"",
        "q5":"",
        "q6":""
    },
    "position":{
        "q1":0.0,
        "q2":0.0,
        "q3":0.0,
        "q4":0.0,
        "q5":0.0,
        "q6":0.0
    },
    "rosout":{
        "seq":0,
        "node":"",
        "msg":""
    }
}
readbackCAL = {
     "x":0,
     "y":0,
     "z":0,
     "seq":0.0,
     "secs":0.0,
     "nsecs":0.0
}



#ROS Callback function for the /mode subscriber
def modeCallback(msg):
    readbackMCS['mode'] = msg.data

#ROS Callback function for the /corr_type subscriber
def corr_typeCallback(msg):
    corr_type = msg.data
    readbackMCS['corr_type'] = corr_type

#ROS Callback functions for the /readback subscribers:
def readbackBCSCallback(msg):
    readbackBCS['BX']   = msg.position[0]
    readbackBCS['BY']   = msg.position[1]
    readbackBCS['BZ']   = msg.position[2]
    readbackBCS['OMEGA'] = msg.position[3]
    readbackBCS['CHI']   = msg.position[4]
    readbackBCS['PHI']   = msg.position[5]
    readbackBCS['seq']   = msg.header.seq
    readbackBCS['secs']  = msg.header.stamp.secs
    readbackBCS['nsecs'] = msg.header.stamp.nsecs
    try:
        redis_handle.hset("readbackBCS", mapping=readbackBCS)
    except:
        print("redis: unable to update readback BCS")
        pass

def readbackSCSCallback(msg):
    readbackSCS['SHX']   = msg.position[0]
    readbackSCS['SHY']   = msg.position[1]
    readbackSCS['SHZ']   = msg.position[2]
    readbackSCS['OMEGA'] = msg.position[3]
    readbackSCS['CHI']   = msg.position[4]
    readbackSCS['PHI']   = msg.position[5]
    readbackSCS['OX']    = msg.position[6]
    readbackSCS['OY']    = msg.position[7]
    readbackSCS['OZ']    = msg.position[8]
    readbackSCS['seq']   = msg.header.seq
    readbackSCS['secs']  = msg.header.stamp.secs
    readbackSCS['nsecs'] = msg.header.stamp.nsecs
    try:
        redis_handle.hset("readbackSCS", mapping=readbackSCS)
    except:
        pass
    
def readbackMCSCallback(msg):
    readbackMCS['serial_number'] = msg.header.frame_id;
    readbackMCS['secs'] = msg.header.stamp.secs;
    readbackMCS['nsecs'] = msg.header.stamp.nsecs;
    readbackMCS['seq'] = msg.header.seq;
    readbackMCS['state']['q1'] = msg.name[0]
    readbackMCS['state']['q2'] = msg.name[1]
    readbackMCS['state']['q3'] = msg.name[2]
    readbackMCS['state']['q4'] = msg.name[3]
    readbackMCS['state']['q5'] = msg.name[4]
    readbackMCS['state']['q6'] = msg.name[5]
    readbackMCS['position']['q1'] = msg.position[0]
    readbackMCS['position']['q2'] = msg.position[1]
    readbackMCS['position']['q3'] = msg.position[2]
    readbackMCS['position']['q4'] = msg.position[3]
    readbackMCS['position']['q5'] = msg.position[4]
    readbackMCS['position']['q6'] = msg.position[5]
    redis_msg = copy(readbackMCS)
    redis_msg.pop('state')
    redis_msg.pop('position')
    rout = redis_msg.pop("rosout")
    redis_msg['rosout'] = f"{rout['seq']}:{rout['node']}:{rout['msg']}"
    try:
        redis_handle.hset("readbackMCS", mapping=redis_msg)
    except:
        pass

def readbackCALCallback(msg):
    readbackCAL['x']  = msg.position[0]
    readbackCAL['y']  = msg.position[1]
    readbackCAL['z']  = msg.position[2]
    readbackCAL['seq']   = msg.header.seq
    readbackCAL['secs']  = msg.header.stamp.secs
    readbackCAL['nsecs'] = msg.header.stamp.nsecs

def rosoutCallback(msg):
    readbackMCS['rosout']['seq'] = msg.header.seq
    readbackMCS['rosout']['node'] = msg.name
    readbackMCS['rosout']['msg'] = msg.msg

def targetSCS_requestCallback(msg): #This is used to update targetSCS if it is set from elsewhere.
    targetSCS['SHX']   = msg.position[0]
    targetSCS['SHY']   = msg.position[1]
    targetSCS['SHZ']   = msg.position[2]
    targetSCS['OMEGA'] = msg.position[3]
    targetSCS['CHI']   = msg.position[4]
    targetSCS['PHI']   = msg.position[5]
    targetSCS['OX']    = msg.position[6]
    targetSCS['OY']    = msg.position[7]
    targetSCS['OZ']    = msg.position[8]
    try:
        redis_handle.hset("targetSCS", mapping=targetSCS)
    except:
        pass
# Here is where we parse the WiiMote input and translate it to a targetSCS_Request 
# def wiimoteCallback(inmsg):
#     wiiY = inmsg.linear_acceleration_zeroed.y
#     wiiZ = inmsg.linear_acceleration_zeroed.z
#     targetSCS['CHI'] = math.atan2(wiiZ,-wiiY)*180/math.pi
#     if targetSCS['CHI'] > 90:
#         targetSCS['CHI'] = 90;
#     if targetSCS['CHI'] < 0:
#         targetSCS['CHI'] = 0;
#     #Post Message on ROS topic
#     msg = JointState()
#     msg.name = ['SHX', 'SHY', 'SHZ', 'OMEGA', 'CHI', 'PHI', 'OX', 'OY', 'OZ']
#     msg.position = [targetSCS['SHX'],   targetSCS['SHY'], targetSCS['SHZ'], 
#                     targetSCS['OMEGA'], targetSCS['CHI'], targetSCS['PHI'],
#                     targetSCS['OX'],    targetSCS['OY'],  targetSCS['OZ']]
#     pub_targetSCS.publish(msg)

def initial_setup():
    time.sleep(1) #wait for subscribers and publishers to be started.
    #Post Message on ROS topic
    msg = JointState()
    msg.name = ['SHX', 'SHY', 'SHZ', 'OMEGA', 'CHI', 'PHI', 'OX', 'OY', 'OZ']
    msg.position = [targetSCS['SHX'],   targetSCS['SHY'], targetSCS['SHZ'], 
                    targetSCS['OMEGA'], targetSCS['CHI'], targetSCS['PHI'],
                    targetSCS['OX'],    targetSCS['OY'],  targetSCS['OZ']]
    pub_targetSCS.publish(msg)
    #Post Default Active Correction State (/corr_type)
    msg = Int16()
    msg.data = corr_type
    pub_corr_type_request.publish(msg)



#Flask Setup
app = Flask(__name__)
api = Api(app)
CORS(app)


#ROS: Here the Topics are registered (Subscribers & Publishers)
rospy.Subscriber("/mode", Int16, modeCallback)
rospy.Subscriber("/corr_type", Int16, corr_typeCallback)
rospy.Subscriber("/readbackBCS", JointState, readbackBCSCallback)
rospy.Subscriber("/readbackSCS", JointState, readbackSCSCallback)
rospy.Subscriber("/readbackMCS", JointState, readbackMCSCallback)
rospy.Subscriber("/readbackCAL", JointState, readbackCALCallback)
rospy.Subscriber("/rosout_agg", Log, rosoutCallback)
rospy.Subscriber("/targetSCS_request", JointState, targetSCS_requestCallback)
#rospy.Subscriber("/wiimote/state", State, wiimoteCallback)
pub_mode_request = rospy.Publisher('/mode_request', Int16, queue_size=100)
pub_corr_type_request = rospy.Publisher('/corr_type', Int16, queue_size=100)
pub_targetBCS = rospy.Publisher('/targetBCS_request', JointState, queue_size=100)
pub_targetSCS = rospy.Publisher('/targetSCS_request', JointState, queue_size=100)
pub_targetMCS = rospy.Publisher('/targetMCS', JointState, queue_size=100)
pub_nudgeBCS = rospy.Publisher('/nudgeBCS', JointState, queue_size=100)
pub_nudgeOBCS = rospy.Publisher('/nudgeOBCS', JointState, queue_size=100)


#Perform initial setup:
initial_setup()




# RESTful API Classes (what to do when a RESTful API Request comes in)
class Mode(Resource):
    def get(self):
        return readback['mode'], 200

    def post(self):
        return self.put()

    def put(self):
        parser = reqparse.RequestParser()
        parser.add_argument("mode", type=int, help="mode cannot be converted")
        args = parser.parse_args()
 
        for k, v in args.items():
            if k == "mode":
                mode_request = int(v)             
    
        #Post Message on topic
        msg = Int16()
        msg.data = mode_request
        pub_mode_request.publish(msg)

        return mode_request, 202 #200:OK, #202:Accepted for processing (not yet processed)


class CorrType(Resource):
    def get(self):
        return corr_type, 200

    def post(self):
        return self.put()

    def put(self):
        parser = reqparse.RequestParser()
        parser.add_argument("corr_type", type=int, help="mode cannot be converted")
        args = parser.parse_args()
 
        for k, v in args.items():
            if k == "corr_type":
                corr_type_request = int(v)             
    
        #Post Message on topic
        msg = Int16()
        msg.data = corr_type_request
        pub_corr_type_request.publish(msg)

        return corr_type, 202 #200:OK, #202:Accepted for processing (not yet processed)


class TargetBCS(Resource):
     def get(self):
         return targetBCS, 200

     def post(self):
         return self.put()

     def put(self):
         
         parser = reqparse.RequestParser()
         parser.add_argument("BX", type=float, 
                 help="BX: value cannot be converted")
         parser.add_argument("BY", type=float, 
                 help="BY: value cannot be converted")
         parser.add_argument("BZ", type=float, 
                 help="BZ: value cannot be converted")
         parser.add_argument("CHI", type=float, 
                 help="CHI: value cannot be converted")
         parser.add_argument("PHI", type=float, 
                 help="PHI: value cannot be converted")
         args = parser.parse_args()
  
         for k, v in args.items():
             if v is not None:
                 targetBCS[k] = v             

         #Post Message on ROS topic
         msg = JointState()
         msg.name = ['BX', 'BY', 'BZ', 'OMEGA', 'CHI', 'PHI']
         msg.position = [targetBCS['BX'], targetBCS['BY'], targetBCS['BZ'], targetBCS['OMEGA'], targetBCS['CHI'], targetBCS['PHI']] 
         pub_targetBCS.publish(msg)

         return targetBCS, 202 #200:OK, #202:Accepted for processing 
                                        #        (not yet processed)
class TargetBCS_Rel(Resource):
     def get(self):
         return targetBCS, 200

     def post(self):
         return self.put()

     def put(self):
         parser = reqparse.RequestParser()
         parser.add_argument("BX", type=float, 
                 help="BX: value cannot be converted")
         parser.add_argument("BY", type=float, 
                 help="BY: value cannot be converted")
         parser.add_argument("BZ", type=float, 
                 help="BY: value cannot be converted")
         parser.add_argument("CHI", type=float, 
                 help="CHI: value cannot be converted")
         parser.add_argument("PHI", type=float, 
                 help="PHI: value cannot be converted")
         args = parser.parse_args()
  
         for k, v in args.items():
             if v is not None:
                 targetBCS[k] = targetBCS[k] + v             
         
         #Post Message on ROS topic
         msg = JointState()
         msg.name = ['BX', 'BY', 'BZ', 'OMEGA', 'CHI', 'PHI']
         msg.position = [targetBCS['BX'], targetBCS['BY'], targetBCS['BZ'], targetBCS['OMEGA'], targetBCS['CHI'], targetBCS['PHI']] 
         pub_targetBCS.publish(msg)

         return targetBCS, 202 #200:OK, #202:Accepted for processing 
                                        #        (not yet processed)

class TargetSCS(Resource):
     def get(self):
         return targetSCS, 200

     def post(self):
         return self.put()

     def put(self):
         
         parser = reqparse.RequestParser()
         parser.add_argument("SHX", type=float, 
                 help="SHX: value cannot be converted")
         parser.add_argument("SHY", type=float, 
                 help="SHY: value cannot be converted")
         parser.add_argument("SHZ", type=float, 
                 help="SHZ: value cannot be converted")
         parser.add_argument("OMEGA", type=float, 
                 help="OMEGA: value cannot be converted")
         parser.add_argument("CHI", type=float, 
                 help="CHI: value cannot be converted")
         parser.add_argument("PHI", type=float, 
                 help="PHI: value cannot be converted")
         parser.add_argument("OX", type=float, 
                 help="OX: value cannot be converted")
         parser.add_argument("OY", type=float, 
                 help="OY: value cannot be converted")
         parser.add_argument("OZ", type=float, 
                 help="OZ: value cannot be converted")
         args = parser.parse_args()
  
         for k, v in args.items():
             if v is not None:
                 targetSCS[k] = v             
                 #bound the value based on limpos and limneg
                 if (targetSCS[k] > targetSCS_limpos[k]):
                     rospy.loginfo("%s out of bounds: Request: %.3f, POS Limit %.3f, Setting to %.3f.", k, targetSCS[k], targetSCS_limpos[k], targetSCS_limpos[k])
                     targetSCS[k] = targetSCS_limpos[k]
                 elif (targetSCS[k] < targetSCS_limneg[k]):
                     rospy.loginfo("%s out of bounds: Request: %.3f, NEG Limit %.3f, Setting to %.3f.", k, targetSCS[k], targetSCS_limneg[k], targetSCS_limneg[k])
                     targetSCS[k] = targetSCS_limneg[k]

         #Post Message on ROS topic
         msg = JointState()
         msg.name = ['SHX', 'SHY', 'SHZ', 'OMEGA', 'CHI', 'PHI', 'OX', 'OY', 'OZ']
         msg.position = [targetSCS['SHX'],   targetSCS['SHY'], targetSCS['SHZ'], 
                         targetSCS['OMEGA'], targetSCS['CHI'], targetSCS['PHI'],
                         targetSCS['OX'],    targetSCS['OY'],  targetSCS['OZ']]
         pub_targetSCS.publish(msg)

         return targetSCS, 202 #200:OK, #202:Accepted for processing 
                                        #        (not yet processed)
class TargetSCS_Rel(Resource):
     def get(self):
         return targetSCS, 200

     def post(self):
         return self.put()

     def put(self):
         parser = reqparse.RequestParser()
         parser.add_argument("SHX", type=float, 
                 help="SHX: value cannot be converted")
         parser.add_argument("SHY", type=float, 
                 help="SHY: value cannot be converted")
         parser.add_argument("SHZ", type=float, 
                 help="SHZ: value cannot be converted")
         parser.add_argument("OMEGA", type=float, 
                 help="OMEGA: value cannot be converted")
         parser.add_argument("CHI", type=float, 
                 help="CHI: value cannot be converted")
         parser.add_argument("PHI", type=float, 
                 help="PHI: value cannot be converted")
         parser.add_argument("OX", type=float, 
                 help="OX: value cannot be converted")
         parser.add_argument("OY", type=float, 
                 help="OY: value cannot be converted")
         parser.add_argument("OZ", type=float, 
                 help="OZ: value cannot be converted")
         args = parser.parse_args()
  
         for k, v in args.items():
             if v is not None:
                 targetSCS[k] = targetSCS[k] + v             
                 #bound the value based on limpos and limneg
                 if (targetSCS[k] > targetSCS_limpos[k]):
                     rospy.loginfo("%s out of bounds: Request: %.3f, POS Limit %.3f, Setting to %.3f.", k, targetSCS[k], targetSCS_limpos[k], targetSCS_limpos[k])
                     targetSCS[k] = targetSCS_limpos[k]
                 elif (targetSCS[k] < targetSCS_limneg[k]):
                     rospy.loginfo("%s out of bounds: Request: %.3f, NEG Limit %.3f, Setting to %.3f.", k, targetSCS[k], targetSCS_limneg[k], targetSCS_limneg[k])
                     targetSCS[k] = targetSCS_limneg[k]
         #Post Message on ROS topic
         msg = JointState()
         msg.name = ['SHX', 'SHY', 'SHZ', 'OMEGA', 'CHI', 'PHI', 'OX', 'OY', 'OZ']
         msg.position = [targetSCS['SHX'],   targetSCS['SHY'], targetSCS['SHZ'], 
                         targetSCS['OMEGA'], targetSCS['CHI'], targetSCS['PHI'],
                         targetSCS['OX'],    targetSCS['OY'],  targetSCS['OZ']]
         pub_targetSCS.publish(msg)

         return targetSCS, 202 #200:OK, #202:Accepted for processing 
                                        #        (not yet processed)

class TargetMCS(Resource):
    def get(self):
        return targetMCS, 200

    def post(self):
        return self.put()

    def put(self):
        parser = reqparse.RequestParser()
        parser.add_argument("q1", type=float, 
                help="q1: value cannot be converted")
        parser.add_argument("q2", type=float, 
                help="q2: value cannot be converted")
        parser.add_argument("q3", type=float, 
                help="q3: value cannot be converted")
        parser.add_argument("q4", type=float, 
                help="q4: value cannot be converted")
        parser.add_argument("q5", type=float, 
                help="q5: value cannot be converted")
        parser.add_argument("q6", type=float, 
                help="q6: value cannot be converted")
        args = parser.parse_args()
 
        for k, v in args.items():
            if v is not None:
                targetMCS[k] = v             
    
        #Post Message on topic
        msg = JointState()
        msg.name = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        msg.position = [targetMCS['q1'], targetMCS['q2'], targetMCS['q3'],
                        targetMCS['q4'], targetMCS['q5'], targetMCS['q6']]
        pub_targetMCS.publish(msg)

        return targetMCS, 202 #200:OK, #202:Accepted for processing 
                                    #        (not yet processed)
class TargetMCS_Rel(Resource):
    def get(self):
        return targetMCS, 200

    def post(self):
        return self.put()

    def put(self):
        parser = reqparse.RequestParser()
        parser.add_argument("q1", type=float, 
                help="q1: value cannot be converted")
        parser.add_argument("q2", type=float, 
                help="q2: value cannot be converted")
        parser.add_argument("q3", type=float, 
                help="q3: value cannot be converted")
        parser.add_argument("q4", type=float, 
                help="q4: value cannot be converted")
        parser.add_argument("q5", type=float, 
                help="q5: value cannot be converted")
        parser.add_argument("q6", type=float, 
                help="q6: value cannot be converted")
        args = parser.parse_args()
 
        for k, v in args.items():
            if v is not None:
                targetMCS[k] = targetMCS[k] + v
    
        #Post Message on topic
        msg = JointState()
        msg.name = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        msg.position = [targetMCS['q1'], targetMCS['q2'], targetMCS['q3'],
                        targetMCS['q4'], targetMCS['q5'], targetMCS['q6']]
        pub_targetMCS.publish(msg)

        return targetMCS, 202 #200:OK, #202:Accepted for processing 
                                    #        (not yet processed)

class ReadbackBCS(Resource):
    def get(self):
        return readbackBCS, 200

class ReadbackSCS(Resource):
    def get(self):
        return readbackSCS, 200

class ReadbackMCS(Resource):
    def get(self):
        return readbackMCS, 200

class ReadbackCAL(Resource):
    def get(self):
        return readbackCAL, 200

class NudgeBCS(Resource):
    def post(self):
        return self.put()

    def put(self):
        parser = reqparse.RequestParser()
        parser.add_argument("BX", type=float, 
                help="BX: value cannot be converted")
        parser.add_argument("BY", type=float, 
                help="BY: value cannot be converted")
        parser.add_argument("BZ", type=float, 
                help="BZ: value cannot be converted")
        args = parser.parse_args()
  
        nudgeBX = 0.
        nudgeBY = 0.
        nudgeBZ = 0.
        
        for k, v in args.items():
            if v is not None:
                if k == "BX":
                    nudgeBX = v
                if k == "BY":
                    nudgeBY = v
                if k == "BZ":
                    nudgeBZ = v
    
        #Post Message on topic
        msg = JointState()
        msg.name = ['BX', 'BY', 'BZ']
        msg.position = [nudgeBX, nudgeBY, nudgeBZ]
        pub_nudgeBCS.publish(msg)

        return [nudgeBX, nudgeBY, nudgeBZ], 202 #200:OK, #202:Accepted for processing 
                                                #        (not yet processed)
class NudgeOBCS(Resource):
    def post(self):
        return self.put()

    def put(self):
        parser = reqparse.RequestParser()
        parser.add_argument("BX", type=float, 
                help="BX: value cannot be converted")
        parser.add_argument("BY", type=float, 
                help="BY: value cannot be converted")
        parser.add_argument("BZ", type=float, 
                help="BZ: value cannot be converted")
        args = parser.parse_args()
  
        nudgeBX = 0.
        nudgeBY = 0.
        nudgeBZ = 0.
        
        for k, v in args.items():
            if v is not None:
                if k == "BX":
                    nudgeBX = v
                if k == "BY":
                    nudgeBY = v
                if k == "BZ":
                    nudgeBZ = v
    
        #Post Message on topic
        msg = JointState()
        msg.name = ['BX', 'BY', 'BZ']
        msg.position = [nudgeBX, nudgeBY, nudgeBZ]
        pub_nudgeOBCS.publish(msg)

        return [nudgeBX, nudgeBY, nudgeBZ], 202 #200:OK, #202:Accepted for processing 
                                                #        (not yet processed)


# List of RESTful API ressources:
api.add_resource(Mode, "/mode")
api.add_resource(CorrType, "/corr_type")
api.add_resource(TargetBCS, "/targetBCS")
api.add_resource(TargetBCS_Rel, "/targetBCS_rel")
api.add_resource(TargetSCS, "/targetSCS")
api.add_resource(TargetSCS_Rel, "/targetSCS_rel")
api.add_resource(TargetMCS, "/targetMCS")
api.add_resource(TargetMCS_Rel, "/targetMCS_rel")
api.add_resource(ReadbackBCS, "/readbackBCS")
api.add_resource(ReadbackSCS, "/readbackSCS")
api.add_resource(ReadbackMCS, "/readbackMCS")
api.add_resource(ReadbackCAL, "/readbackCAL")
api.add_resource(NudgeBCS, "/nudgeBCS")
api.add_resource(NudgeOBCS, "/nudgeOBCS")

@app.route('/', methods=['PUT','POST','GET','OPTIONS'])
def default_message():
    return 'Welcome to smargopolo!'

if __name__ == '__main__':
    #app.run(host="localhost", port=3000)
    app.debug = False
    app.run(host="0.0.0.0", port=3000)
