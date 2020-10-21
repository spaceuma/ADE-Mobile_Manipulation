# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 12:41:45 2020

@author: Richi
"""

from scipy.optimize import fsolve
from scipy.optimize import least_squares
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

#beta = math.radians(30.0) #Counter-clockwise rotation from heading vector to sampling vector
#offL = 0.5
offx = 0.384
offy = 0.969

offL = np.sqrt(pow(offx,2)+pow(offy,2))

x0Array = np.linspace(-0.5,offx-0.01,60)
y0Array = np.linspace(offy+0.01,2.3,80)

Rmatrix = np.zeros((y0Array.size,x0Array.size))
omegamatrix = np.zeros((y0Array.size,x0Array.size))

for j in range(y0Array.size):
    for i in range(x0Array.size):
        x0 = x0Array[i]
        y0 = y0Array[j]
        #print(x0)
        #print(y0)
        #print(' ')
        def equations(i):
            R,omega = i
            gamma = math.asin(offy/R)
            f1 = - math.cos(omega) * math.sqrt(pow(R,2)-pow(offy,2)) + math.sin(omega)*offy + math.sqrt(pow(R,2)-pow(offy,2)) - offx - x0
            f2 = math.sin(omega) * math.sqrt(pow(R,2)-pow(offy,2)) + math.cos(omega)*offy - y0
            #f1 = -(R+offx)*math.cos(omega) + offy*math.sin(omega) + R + x0
            #f2 = (R+offx)*math.sin(omega) + offy*math.cos(omega) - y0
            return (f1,f2)
        for k in range(10):
            try:
            #R,omega = least_squares(equations,(5.0, math.radians(30.0)), bounds = ((offy,0.0),(None,3.1416)))
                R,omega = fsolve(equations,(k, math.radians(30.0)))
                Rmatrix[j,i] = 1.0/R
                omegamatrix[j,i] = omega
                break
            except ValueError:
                Rmatrix[j,i] = 0.0
                omegamatrix[j,i] = 0.0


#print(equations((R,omega)))
#print(R)
#print(math.degrees(omega))

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
XX,YY = np.meshgrid(x0Array,y0Array)

surf = ax.plot_surface(XX,YY,Rmatrix, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

import pandas as pd
import csv

RmatrixToCSV = pd.DataFrame(data=Rmatrix.astype(float))
RmatrixToCSV.to_csv('LSC_TurningCurvature.txt', sep=' ', header=False, float_format='%.3f', index=False)

OmegaToCSV = pd.DataFrame(data=omegamatrix.astype(float))
OmegaToCSV.to_csv('LSC_TurningAngle.txt', sep=' ', header=False, float_format='%.3f', index=False)

x0ArrayT = x0Array[np.newaxis]

x0ArrayT = x0ArrayT.T
print(x0ArrayT.shape)

XToCSV = pd.DataFrame(columns = x0Array)
XToCSV.loc[0] = x0Array.astype(float)
XToCSV.to_csv('LSC_X0.txt', sep=' ', header=False, float_format='%.3f', index=False)
#with open('LSC_X0.txt', 'w') as csvfile:
#    xWriter = csv.writer(csvfile,delimiter = ' ', lineterminator = '\n')
#    xWriter.writerows(map(lambda x: [x], x0Array))

YToCSV = pd.DataFrame(columns = y0Array)
YToCSV.loc[0] = y0Array.astype(float)
YToCSV.to_csv('LSC_Y0.txt', sep=' ', header=False, float_format='%.3f', index=False)

#ax.set_zlim(0.0, 2.0)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
XX,YY = np.meshgrid(x0Array,y0Array)

surf = ax.plot_surface(XX,YY,omegamatrix, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

plt.show()
