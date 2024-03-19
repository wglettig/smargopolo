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
# For readback use GETP instead of RBV, RBV includes the ...-OFF offset
# to read back values into a variable, you can also use the form:
# val=epics.caget("X06MX-ES-DF1:OMEGA-RBV")


import epics

    
# Get current GETP
epics.caget("X06MX-ES-DF1:GMX-GETP")
epics.caget("X06MX-ES-DF1:GMY-GETP")
epics.caget("X06MX-ES-DF1:GMZ-GETP")
epics.caget("X06MX-ES-DF1:OMEGA-GETP")

#Home:
86.89399995422363
-0.005
1.72
0


#Move to Safe Position:
epics.caput("X06MX-ES-DF1:GMX-SETV",5)
epics.caput("X06MX-ES-DF1:GMX-SETP",50)

epics.caput("X06MX-ES-DF1:GMY-SETV",5)
epics.caput("X06MX-ES-DF1:GMY-SETP",3)

epics.caput("X06MX-ES-DF1:GMZ-SETV",5)
epics.caput("X06MX-ES-DF1:GMZ-SETP",30)

#Move to HOT position:
epics.caput("X06MX-ES-DF1:GMX-SETV",5)
epics.caput("X06MX-ES-DF1:GMX-SETP",80)

epics.caput("X06MX-ES-DF1:GMY-SETV",5)
epics.caput("X06MX-ES-DF1:GMY-SETP",3)

epics.caput("X06MX-ES-DF1:GMZ-SETV",5)
epics.caput("X06MX-ES-DF1:GMZ-SETP",30)





# Get current Position: (uses: GMX-OFF offset) 
epics.caget("X06MX-ES-DF1:GMX-RBV")
epics.caget("X06MX-ES-DF1:GMY-RBV")
epics.caget("X06MX-ES-DF1:GMZ-RBV")
epics.caget("X06MX-ES-DF1:OMEGA-RBV")

#Home:
-59.10599969482422
-0.0050880000000000005
1.719998782348633
2.108328683035714e-06






# Go home:
        
epics.caput("X06MX-ES-DF1:OMEGA-SETP",50)
epics.caput("X06MX-ES-DF1:OMEGA-SETP",50)
epics.caput("X06MX-ES-DF1:OMEGA-SETP",50)
        
        





        
        
        
        
        
        


        
        
        




