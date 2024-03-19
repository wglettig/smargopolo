#!/usr/bin/env python

# This Script creates a lookup table from the measured error CSV
# Wayne Glettig, 16.7.2021

# Uses pandas, because pandas is fast, and needs less CPU & RAM usage than regular numpy
import pandas as pd
from pandas import read_csv
import matplotlib.pyplot as plt

# Read in CSV File into DataFrame rawdf:
rawdf = read_csv('measure_omega_rot_OUTPUT.csv')
rawdf['time']=rawdf['DMS_Secs']+rawdf['DMS_Nsecs']*1e-9 #Joins s & ns columns to one float
rawdf['X']=(rawdf['DMS_X']+0.00)  #Set X Offset and scale
rawdf['Y']=(rawdf['DMS_Y']+0.0005)   #Set Y Offset and scale
rawdf['Z']=(rawdf['DMS_Z']-0.00)   #Set Z Offset and scale

# Select Data Window and save to new DataFrame df:
#df = rawdf[615:2219]
df = rawdf

# Create LUT: Average values per omega window
lut = pd.DataFrame(columns=['OMEGA', 'X', 'Y', 'Z'])
window = 10 # degrees
for om in range(0,360, window):
    Xavg = df[(df.OMEGA>om)&(df.OMEGA<om+window)]['X'].mean()#*1e-9
    Yavg = df[(df.OMEGA>om)&(df.OMEGA<om+window)]['Y'].mean()#*1e-9
    Zavg = df[(df.OMEGA>om)&(df.OMEGA<om+window)]['Z'].mean()#*1e-9
    lut = lut.append({'OMEGA':om+window/2,'X':Xavg, 'Y':Yavg, 'Z':Zavg}, ignore_index=True)

lut

# Plot:
ax1 = df.plot(x='time', y=['X','Y','Z'])
ax1.set_title("Sample Window of Raw Signals")
ax2 = df.plot(x='OMEGA', y=['X','Y','Z'])
lut.plot(x='OMEGA', y=['X','Y','Z'], ax=ax2)
ax2.set_title("Fitted LUT curve over measured curve")

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

# Output to CSV
lut.to_csv("create_LUT_omega_OUTPUT.csv")
