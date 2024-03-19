#!/usr/bin/env python

# Dieses Script verfaehrt den Aerotech translativ mit GMX.
# Anhand des Scripts wir die Genauigkeit der Sensoren ermittelt.

# Dominik Buntschu, 2.8.2021

import epics
import time

# Setzt die Geschwidigkeit der Aerotech Achsen
epics.caput("X06MX-ES-DF1:GMX-SETV",2)
epics.caput("X06MX-ES-DF1:GMZ-SETV",2)
epics.caput("X06MX-ES-DF1:GMY-SETV",2)

# Faehrt Aerotech Achsen in Startpositionen
epics.caput("X06MX-ES-DF1:GMX-SETP",89)
epics.caput("X06MX-ES-DF1:GMY-SETP",4)
epics.caput("X06MX-ES-DF1:GMZ-SETP",46)
time.sleep(3)
# Macht X Iterationen eines Rechteckes mit jeweils einem Versatz
# Veranschaulicht den Winkelversatz zwischen Aerotech und Kalibriertool 
Step1 = 4
Step2 = 3
      
for i in range(0,Step1,1):
    print("Move GMX to +0.0005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",0.0005)
    time.sleep(2.5)

for i in range(0,Step1*2,1):
    print("Move GMX to -0.0005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",-0.0005)
    time.sleep(2.5)
    
for i in range(0,Step1,1):
    print("Move GMX to +0.0005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",0.0005)
    time.sleep(2.5)

for i in range(0,Step2,1):
    print("Move GMX to +0.005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",0.0005)
    time.sleep(2.5)

for i in range(0,Step2*2,1):
    print("Move GMX to -0.0005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",-0.0005)
    time.sleep(2.5)
    
for i in range(0,Step2,1):
    print("Move GMX to +0.0005")
    epics.caput("X06MX-ES-DF1:GMX-INCP",0.0005)
    time.sleep(2.5)

