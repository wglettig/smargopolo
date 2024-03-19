#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 24 05:12:42 2021

@author: ros
"""

from caproto.sync.client import read
from caproto.sync.client import write


res = read('X06MX-ES-DF1:OMEGA-SETP')
print (res.data[0])

# Initialize Aerotech: ########################################################
# Press "Stop / Reset, Restart" Button:
res = write('X06MX-ES-AERO:TSK-STOP',1, notify='True')
print (res.status.description)

# Press "HOME ALL" button:
#res = write('X06MX-ES-DF1:HOME-ALL',1, notify='True')
#print (res.status.description)

# Read back Mode:
res = read('X06MX-ES-DF1:AXES-MODE')
print ('Mode: ', res.data[0])

# Read back A3200 Init:
res = read('X06MX-ES-AERO:CTRL-STAT')
print ('A3200 Init: ', res.data[0])

# Read back Library:
res = read('X06MX-ES-AERO:CTRL-LIB')
print ('Library: ', res.data[0])

# Read back SStatus:
res = read('X06MX-ES-AERO:STAT')
print ('SStatus: ', res.data[0])


# Check Omega Setup ###########################################################
# Omega positive in Clockwise direction (opposite to trigonometic direction)

# Check UsrOffset: At Robot cage this should be 60deg
res = read('X06MX-ES-DF1:OMEGA-OFF')
print (res.data[0])

# Check Current Position (this depends on UsrOffset)
res = read('X06MX-ES-DF1:OMEGA-RBV')
print (res.data[0])

# Raw Position Readback: GETP+OFF=RBV <=> GETP=RBV-OFF
res = read('X06MX-ES-DF1:OMEGA-GETP')
print (res.data[0])

# Setting of absolute raw position:
write('X06MX-ES-DF1:OMEGA-SETP',-60, notify='True')

# Check Current Velocity:
res = read('X06MX-ES-DF1:OMEGA-SETV')
print (res.data[0])

write('X06MX-ES-DF1:OMEGA-SETV', 45, notify='True')
