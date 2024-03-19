#! /usr/bin/env python

# This example demonstrates ROS working with Flask, flask_restl
# and rospkg. How to install:
# pip install -U rospkg
################################################################################
# This script is based on the answers.ros.org post at:
# [1] https://answers.ros.org/question/234418/easiest-way-to-implement-http-server-that-can-send-ros-messages/
#
# Wayne Glettig 19.06.2021
################################################################################
# REST API USE: 
# Ressource /enableCAL:
# GET: 127.0.0.1:3030/enableCAL
# PUT: 127.0.0.1:3030/enableCAL?enableCAL=1    //1: Feedback active 0: Feedback not active
#
# Ressource /readbackCAL:
# GET: 127.0.0.1:3030/readbackCAL
#
# Ressource /moveCAL:
# PUT: 127.0.0.1:3030/moveCAL?ch0=1&ch1=0.1&ch2=0.2   //(move in [mm])
#
#
################################################################################
# NOTES:
# Response status codes are usually 202:Accepted, as usually the request is 
# merely placed, and the API is not blocked until motion is completed.
#
# Response to a GET command is a JSON snippet with the current values.
# Response to a PUT command is a the updated values.
# Example of response from a GET /readbackCAL:
# {
#   "x": 0.1
#   "y": 0.0
#   "z": 0.0
#   "ch0": 0.0
#   "ch1": 0.0
#   "ch2": 0.0
# }
#
################################################################################
# ROS Topics:
# Subscribed to:
# /enableCAL      std_msgs::Int16
# /readbackCAL    sensor_msgs::JointState
#
# Publishes to:
# /enableCAL      std_msgs::Int16
# /moveCAL        sensor_msgs::JointState
################################################################################

#RESTful API related libraries:
import os
import threading
from flask import Flask 
from flask_restful import Api, Resource, reqparse
from flask_cors import CORS

#ROS related libraries
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Log
#General libraries
import math
from datetime import datetime
import time
#This reduces logging
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

#ROS: This is the thread in which the ROS node is run (according to [1])
threading.Thread(target=lambda: rospy.init_node('calib_restful', 
                                     disable_signals=True)).start()

#Global Variables (state)
enableCAL = 0

#targets: (upon loading, read from ROS param server)
moveCAL = {
    "ch0":0.0,
    "ch1":0.0,
    "ch2":0.0
}
#readbacks:
readbackCAL = {
    "x":0.0,
    "y":0.0,
    "z":0.0,
    "ch0":0.0,
    "ch1":0.0,
    "ch2":0.0,
    "adc0":0.0,
    "adc1":0.0,
    "adc2":0.0,
    "serial_number":"",
    "seq":0.0,
    "secs":0.0,
    "nsecs":0.0
}


#ROS Callback function for the /enableCAL subscriber
def enableCALCallback(msg):
    enableCAL = msg.data

#ROS Callback functions for the /readback subscribers:
def readbackCALCallback(msg):
    readbackCAL['x']     = msg.position[0]
    readbackCAL['y']     = msg.position[1]
    readbackCAL['z']     = msg.position[2]
    readbackCAL['ch0']   = msg.position[3]
    readbackCAL['ch1']   = msg.position[4]
    readbackCAL['ch2']   = msg.position[5]
    readbackCAL['adc0']   = msg.position[6]
    readbackCAL['adc1']   = msg.position[7]
    readbackCAL['adc2']   = msg.position[8]
    readbackCAL['serial_number'] = msg.header.frame_id
    readbackCAL['seq']   = msg.header.seq
    readbackCAL['secs']  = msg.header.stamp.secs
    readbackCAL['nsecs'] = msg.header.stamp.nsecs



#ROS: Here the Topics are registered (Subscribers & Publishers)
rospy.Subscriber("/enableCAL", Int16, enableCALCallback)
rospy.Subscriber("/readbackCAL", JointState, readbackCALCallback)
pub_enableCAL = rospy.Publisher('/enableCAL', Int16, queue_size=100)
pub_moveCAL = rospy.Publisher('/moveCAL', JointState, queue_size=100)


#Flask Setup
app = Flask(__name__)
api = Api(app)
CORS(app)


# RESTful API Classes (what to do when a RESTful API Request comes in)
class EnableCAL(Resource):
    def get(self):
        return enableCAL, 200

    def post(self):
        return self.put()

    def put(self):
        global enableCAL
        parser = reqparse.RequestParser()
        parser.add_argument("enableCAL", type=int, help="mode cannot be converted")
        args = parser.parse_args()
 
        for k, v in args.items():
            if k == "enableCAL":
                enableCAL = int(v)             
    
        #Post Message on topic
        msg = Int16()
        msg.data = enableCAL
        pub_enableCAL.publish(msg)

        return enableCAL, 202 #200:OK, #202:Accepted for processing (not yet processed)

class ReadbackCAL(Resource):
    def get(self):
        return readbackCAL, 200

class MoveCAL(Resource):
     def get(self):
         return moveCAL, 200

     def post(self):
         return self.put()

     def put(self):
         parser = reqparse.RequestParser()
         parser.add_argument("ch0", type=float, 
                 help="ch0: value cannot be converted")
         parser.add_argument("ch1", type=float, 
                 help="ch1: value cannot be converted")
         parser.add_argument("ch2", type=float, 
                 help="ch2: value cannot be converted")
         args = parser.parse_args()

         moveCAL['ch0'] = 0.0
         moveCAL['ch1'] = 0.0
         moveCAL['ch2'] = 0.0
  
         for k, v in args.items():
             if v is not None:
                 moveCAL[k] = v             
     
         #Post Message on ROS topic
         msg = JointState()
         msg.name = ['ch0', 'ch1', 'ch0']
         msg.position = [moveCAL['ch0'], moveCAL['ch1'], moveCAL['ch2']]
         pub_moveCAL.publish(msg)

         return moveCAL, 202 #200:OK, #202:Accepted for processing 
                                      #        (not yet processed)

# List of RESTful API ressources:
api.add_resource(EnableCAL, "/enableCAL")
api.add_resource(ReadbackCAL, "/readbackCAL")
api.add_resource(MoveCAL, "/moveCAL")

@app.route('/', methods=['PUT','POST','GET','OPTIONS'])
def default_message():
    return 'Welcome to smargopolo Calibration REST API!'

if __name__ == '__main__':
    #app.run(host="localhost", port=3000)
    app.run(host="0.0.0.0", port=3030)
