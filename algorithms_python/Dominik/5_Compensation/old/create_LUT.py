#!/usr/bin/env python

# Bedingungen: 
# 1. Ausrichtung des DMS Tools zu Aerotech mit Script: 210827_measure_DMS_Tool_linearity.py
# 2. Omega Achse ausgereichtet mit Script:210827_measure_Omega_Offset_to_Smargon.py
# 3. Die Kalibrierpin Vektoren wurden durch das Script: measure_PinVector_SHX_SHY.py ermittelt
# 4. Der Kalibrierpin Vektor wurde durch das Script: measure_PinVector_SHZ_2.py ermittelt
# 5. Smargon Koordinatensystem OX und OY wurde auf die Omega Achse Korrigiert
# 6. KOmplettes Fehlerbild wurde in einer csv Datei geschrieben Chi 0-40 Grad / Omega jeweils 0/360 Grad

# Dieses Scgript generiert eine LUT, Look up table aufgrund der gemessenen Fehler des Smargons.

# Dominik Buntschu, 8.9.2021

# Uses pandas, because pandas is fast, and needs less CPU & RAM usage than regular numpy
import pandas as pd
from pandas import read_csv
import matplotlib.pyplot as plt

# Read in CSV File into DataFrame rawdf:
rawdf = read_csv('2021_09_08_14:40:28_measure_Smargon_Error_OXOY_CHI0_40.csv')
rawdf['time']=rawdf['DMS_Secs']+rawdf['DMS_Nsecs']*1e-9 #Joins s & ns columns to one float
rawdf['X']=(rawdf['DMS_X']-6006337873)*1e-9  #Set X Offset and scale
rawdf['Y']=(rawdf['DMS_Y']+43285290)*1e-9   #Set Y Offset and scale
rawdf['Z']=(rawdf['DMS_Z']+169185962)*1e-9   #Set Z Offset and scale

# Select Data Window and save to new DataFrame df:
df = rawdf[500:20750]

# Create LUT: Average values per omega window
lut = pd.DataFrame(columns=['CHI','OMEGA', 'X', 'Y', 'Z'])

chistep = 5 # degrees
omegastep = 10 # degrees

for chi in range(0,40, chistep):
    for om in range(0,360, omegastep):
    	Xavg = df[(df.OMEGA>om)&(df.OMEGA<om+omegastep)]['X'].mean()#*1e-9
    	Yavg = df[(df.OMEGA>om)&(df.OMEGA<om+omegastep)]['Y'].mean()#*1e-9
    	Zavg = df[(df.OMEGA>om)&(df.OMEGA<om+omegastep)]['Z'].mean()#*1e-9
    	#lut = lut.append({'CHI':chi+chistep,'OMEGA':om+omegastep/2,'X':Xavg, 'Y':Yavg, 'Z':Zavg}, ignore_index=True)
    	lut = lut.append({'CHI':chi+chistep,'OMEGA':om+omegastep/2,'X':Xavg, 'Y':Yavg, 'Z':Zavg})

# Plot:
ax1 = df.plot(x='time', y=['Z'])
ax1.set_title("Sample Window of Raw Signals")
ax2 = df.plot(x='OMEGA', y=['Z'])
lut.plot(x='OMEGA', y=['Z'], ax1=ax2)
ax2.set_title("Fitted LUT curve over measured curve")

# Plot:
#ax1 = df.plot(x='time', y=['X','Y','Z'])
#ax1.set_title("Sample Window of Raw Signals")
#ax2 = df.plot(x='OMEGA', y=['X','Y','Z'])
#lut.plot(x='OMEGA', y=['X','Y','Z'], ax1=ax2)
#ax2.set_title("Fitted LUT curve over measured curve")

# Check edge overlap of omega range:
ax3 = df.plot(x='OMEGA', y=['X','Y','Z'])
lut.plot(x='OMEGA', y=['X','Y','Z'], ax=ax3)
lutbef=lut.copy(deep=True)
lutbef['OMEGA']=lut['OMEGA']-360
lutaft=lut.copy(deep=True)
lutaft['OMEGA']=lut['OMEGA']+360
lutbef.plot(x='OMEGA', y=['X','Y','Z'], ax=ax3)
lutaft.plot(x='OMEGA', y=['X','Y','Z'], ax=ax3)
ax3.set_title("Check overlap at 0 & 360 deg")

plt.show()


