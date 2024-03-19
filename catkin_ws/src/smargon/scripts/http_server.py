#! /usr/bin/env python
import os 
from http.server import HTTPServer, CGIHTTPRequestHandler
import rospkg
import rospy



# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for smargon
directory = rospack.get_path('smargon')


# Make sure the server is created at current directory
os.chdir(directory+'/html')
# Create server object listening the port 80
server_object = HTTPServer(server_address=('', 8080), RequestHandlerClass=CGIHTTPRequestHandler)

rospy.loginfo("Starting a webserver. Point your browser to http://smargopolo:8080.")

#Start the web server
server_object.serve_forever()
