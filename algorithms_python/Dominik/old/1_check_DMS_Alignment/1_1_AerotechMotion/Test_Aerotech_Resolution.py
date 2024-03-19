#!/usr/bin/env python

# Dieses Script verfaehrt den Aerotech translativ mit GMX,GMY und GMZ.
# Der Ausgangswert des Scripts beschreibt den Winkelversatz zwischen Aerotech und
# dem Kalibrier Tool.

# Dominik Buntschu, 2.8.2021

import epics
import time

# Setzt die Geschwidigkeit der Aerotech Achsen
epics.caput("X06MX-ES-DF1:GMX-SETV",2)
epics.caput("X06MX-ES-DF1:GMZ-SETV",2)
epics.caput("X06MX-ES-DF1:GMY-SETV",2)

# Faehrt Aerotech Achsen in Startpositionen
epics.caput("X06MX-ES-DF1:GMX-SETP",85)
epics.caput("X06MX-ES-DF1:GMY-SETP",4)
epics.caput("X06MX-ES-DF1:GMZ-SETP",23)
time.sleep(3)
# Macht X Iterationen eines Rechteckes mit jeweils einem Versatz
# Veranschaulicht den Winkelversatz zwischen Aerotech und Kalibriertool 
Step1 = 5
Step3 = 3       
for i in range(0,Step1,1):
    print("Move GMY to +0.001")
    epics.caput("X06MX-ES-DF1:GMY-INCP",0.001)
    time.sleep(2.5)

for i in range(0,Step1*2,1):
    print("Move GMY to -0.001")
    epics.caput("X06MX-ES-DF1:GMY-INCP",-0.001)
    time.sleep(2.5)
    
for i in range(0,Step1,1):
    print("Move GMY to +0.001")
    epics.caput("X06MX-ES-DF1:GMY-INCP",0.001)
    time.sleep(2.5)

for i in range(0,Step3,1):
    print("Move GMY to +0.003")
    epics.caput("X06MX-ES-DF1:GMY-INCP",0.003)
    time.sleep(2.5)

for i in range(0,Step3*2,1):
    print("Move GMY to -0.003")
    epics.caput("X06MX-ES-DF1:GMY-INCP",-0.003)
    time.sleep(2.5)

for i in range(0,Step3,1):
    print("Move GMY to +0.003")
    epics.caput("X06MX-ES-DF1:GMY-INCP",0.003)
    time.sleep(2.5)



