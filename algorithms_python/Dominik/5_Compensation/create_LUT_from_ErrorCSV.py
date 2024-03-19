#!/usr/bin/env python

# Bedingung: 
# 1_1.Ausrichtung des DMS Tools zu Aerotech mit Script: 	measure_DMS_Tool_angleError.py
# 1_2.Omega Achse ausgereichtet mit Script:			measure_Omega_Offset_to_Smargon.py
# 2_1.Die Kalibrierpin Vektoren ermittelt mit Script: 	measure_PinVector_SHX_SHY.py
# 2_2.Der Kalibrierpin Vektor ermittelt mit Script: 		measure_Offset_Q4_AND_PinVector_SHZ.py
# 3.  Der Smargon Offset ermittelt mit Script:		measure_Smargon_Error_OXOY_CHI0.py
# 4.  KOmplettes Fehlerbild ermittelt mit Script:		measure_Smargon_Error_OXOY_CHI0_40.py

# Dieses Scgript generiert eine LUT (Look up table) aufgrund des gemessenen Fehler des Smargons.

# Dominik Buntschu, 13.11.2021

import pandas as pd
from pandas import read_csv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

#Daten von Error Datei in Panda DataFrame
rawdf = read_csv('2022_01_19_21:24:31_measure_Smargon_Error_OXOY_CHI0_40.csv')

#Aufbereiten der Daten
rawdf['time']=rawdf['DMS_Secs']+rawdf['DMS_Nsecs']*1e-9
rawdf['OMEGA_Cont']=(rawdf['OMEGA_Cont'])
rawdf['X']=(rawdf['DMS_X']-0.008198765) #mit Offset zu 0 Referenz
rawdf['Y']=(rawdf['DMS_Y']+0.003291426) #mit Offset zu 0 Referenz
rawdf['Z']=(rawdf['DMS_Z']+0.001798504) #mit Offset zu 0 Referenz

# Relevante Daten deklarieren
df = rawdf[1200:28060] # Bewegung startet ab Zeile 1200 (CHI= 0 und OMEGA = 0) und endet mit Zeile 28100 (CHI= 40 und OMEGA = 360)

# Generiert LUT mit Header
lut = pd.DataFrame(columns=['CHI','OMEGA', 'X_LUT', 'Y_LUT', 'Z_LUT'])

# Definition Groesse der LUT
chistep = 10 # degrees (muss gleich gross sein wie Schrittweite von CHI im Script: measure_Smargon_Error_OXOY_CHI0_40.py)
omegastep = 10 # degrees (kann veraendert werden.-> beeinflusst Groesse der LUT)
Counter = 0
chi_tolerance = 1

#generiert Mittelwert mit CHI Schritten von chistep (10 Grad)und  Omega Schritten von omegastep (10 Grad)
for chi in range(0,50, chistep):
    for om in range(0,360, omegastep):
    	Xavg = df[(df.CHI>chi-chi_tolerance)&(df.CHI<chi+chi_tolerance)&(df.OMEGA>om)&(df.OMEGA<om+omegastep)]['X'].mean()#*1e-9
    	Yavg = df[(df.CHI>chi-chi_tolerance)&(df.CHI<chi+chi_tolerance)&(df.OMEGA>om)&(df.OMEGA<om+omegastep)]['Y'].mean()#*1e-9
    	Zavg = df[(df.CHI>chi-chi_tolerance)&(df.CHI<chi+chi_tolerance)&(df.OMEGA>om)&(df.OMEGA<om+omegastep)]['Z'].mean()#*1e-9
    	
    	if Counter == 0:
    	   lut = lut.append({'CHI':chi,'OMEGA':om+omegastep/2,'OMEGA_Cont':(om+omegastep/2)+(Counter*360),'X_LUT':Xavg, 'Y_LUT':Yavg, 'Z_LUT':Zavg}, ignore_index=True)
    	else:
    	   lut = lut.append({'CHI':chi,'OMEGA':om+omegastep/2,'OMEGA_Cont':(om+omegastep/2)+(Counter*360),'X_LUT':Xavg, 'Y_LUT':Yavg, 'Z_LUT':Zavg}, ignore_index=True)
    Counter = Counter+1

#generiert CSV Datei mit den Spalten CHI,OMEGA,X_LUT,Y_LUT,Z_LUT (ohne Index)
lut.to_csv('LUT_Smargon_Error_corrected.csv', columns=['CHI','OMEGA','X_LUT','Y_LUT','Z_LUT'], index=False)
	
# 2D Plots:
ax1 = df.plot(x='OMEGA_Cont', y=['X','Y','Z'])
ax1.set_title("2D PLot: Origin Error (CHI 0 -40 degree with Omega 0 - 360 degree)")
ax1.set_xlabel('Omega [degree]')
ax1.set_ylabel('X,Y,Z Error [mm]')

ax2 = lut.plot(x='OMEGA_Cont', y=['X_LUT','Y_LUT','Z_LUT'])
ax2.set_title("2D PLot: Calculated Error in LUT")
ax2.set_xlabel('Omega [degree]')
ax2.set_ylabel('Calculated X,Y,Z Error [mm]')

ax3 = df.plot(x='OMEGA_Cont', y=['X','Y','Z'])
lut.plot(x='OMEGA_Cont', y=['X_LUT','Y_LUT','Z_LUT'],ax=ax3)
ax3.set_title("2D PLot: Overlapping LUT with origin Error Signal")
ax3.set_xlabel('Omega [degree]')
ax3.set_ylabel('X_LUT, Y_LUT, Z_LUT, X,Y,Z [mm]')

# 3D Plots: Error Visualisierung
chi_plot = lut.CHI
omega_plot = lut.OMEGA
x_plot = lut.X_LUT
y_plot = lut.Y_LUT
z_plot = lut.Z_LUT
my_cmap = plt.get_cmap('hot')

ax5 = plt.figure()
ax5 = plt.axes(projection='3d')
ax5.plot_trisurf(omega_plot, chi_plot,x_plot,cmap=cm.jet, linewidth=0.5)
ax5.set_title('Trisurf Plot X Error')
ax5.set_xlabel('Omega [degree]')
ax5.set_ylabel('Chi [degree]')
ax5.set_zlabel('X_Offset [mm]')

ax6 = plt.figure()
ax6 = plt.axes(projection='3d')
ax6.plot_trisurf(omega_plot, chi_plot,y_plot,cmap=cm.jet, linewidth=0.5)
ax6.set_title('Trisurf Plot Y Error ')
ax6.set_xlabel('Omega [degree]')
ax6.set_ylabel('Chi [degree]')
ax6.set_zlabel('Y_Offset [mm]')

ax7 = plt.figure()
ax7 = plt.axes(projection='3d')
ax7.plot_trisurf(omega_plot, chi_plot,z_plot,cmap=cm.jet, linewidth=0.5)
ax7.set_title('Trisurf Plot Z Error')
ax7.set_xlabel('Omega [degree]')
ax7.set_ylabel('Chi [degree]')
ax7.set_zlabel('Z_Offset [mm]')

plt.show()


