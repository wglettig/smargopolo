#!/usr/bin/env python

# This Script creates a lookup table from the measured error CSV
# Wayne Glettig, 16.7.2021

# Uses pandas, because pandas is fast, and needs less CPU & RAM usage than regular numpy
import pandas as pd
from pandas import read_csv
import matplotlib.pyplot as plt

# Read in CSV File into DataFrame rawdf:
rawdf = read_csv('measure_chi_rot_OUTPUT.csv')
rawdf['time']=rawdf['DMS_Secs']+rawdf['DMS_Nsecs']*1e-9 #Joins s & ns columns to one float
#rawdf['X']=(rawdf['DMS1']-6006337873)*1e-9  #Set X Offset and scale
#rawdf['Y']=(rawdf['DMS2']+43285290)*1e-9   #Set Y Offset and scale
#rawdf['Z']=(rawdf['DMS3']+169185962)*1e-9   #Set Z Offset and scale

# Select Data Window and save to new DataFrame df:
#df = rawdf[615:2219]
df = rawdf

# Create LUT: Average values per omega window
lut = pd.DataFrame(columns=['CHI', 'LUT_X', 'LUT_Y', 'LUT_Z'])
window = 3 # degrees
for om in range(0,90, window):
    Xavg = df[(df.SCS_CHI>om)&(df.SCS_CHI<om+window)]['DMS_X'].mean()#*1e-9
    Yavg = df[(df.SCS_CHI>om)&(df.SCS_CHI<om+window)]['DMS_Y'].mean()#*1e-9
    Zavg = df[(df.SCS_CHI>om)&(df.SCS_CHI<om+window)]['DMS_Z'].mean()#*1e-9
    lut = lut.append({'CHI':om+window/2,'LUT_X':Xavg, 'LUT_Y':Yavg, 'LUT_Z':Zavg}, ignore_index=True)

# Plot:
ax1 = df.plot(x='time', y=['DMS_X','DMS_Y','DMS_Z'])
ax1.set_title("Sample Window of Raw Signals")
ax2 = df.plot(x='SCS_CHI', y=['DMS_X','DMS_Y','DMS_Z'])
lut.plot(x='CHI', y=['LUT_X','LUT_Y','LUT_Z'], ax=ax2)
ax2.set_title("Fitted LUT curve over measured curve")

plt.show()


