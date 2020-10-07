# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 12:41:45 2020

@author: Richi
"""

from scipy.optimize import fsolve
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

#beta = math.radians(30.0) #Counter-clockwise rotation from heading vector to sampling vector
#offL = 0.5
offx = 0.969
offy = 0.384

offL = np.sqrt(pow(offx,2)+pow(offy,2))

x0Array = np.linspace(-0.5,offx-0.01,20)
y0Array = np.linspace(offL+0.1,2.0,30)

Rmatrix = np.zeros((y0Array.size,x0Array.size))
omegamatrix = np.zeros((y0Array.size,x0Array.size))

for j in range(y0Array.size):
    for i in range(x0Array.size):
        x0 = x0Array[i]
        y0 = y0Array[j]
        def equations(i):
            R,omega = i
            f1 = -(R+offx)*math.cos(omega) + offy*math.sin(omega) + R + x0
            f2 = (R+offx)*math.sin(omega) + offy*math.cos(omega) - y0
            return (f1,f2)
        R,omega = fsolve(equations,(1.0, math.radians(30.0)))
        Rmatrix[j,i] = 1.0/R
        omegamatrix[j,i] = omega

print(equations((R,omega)))
print(R)
print(math.degrees(omega))

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
XX,YY = np.meshgrid(x0Array,y0Array)

surf = ax.plot_surface(XX,YY,Rmatrix, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

import pandas as pd
RmatrixToCSV = pd.DataFrame(data=Rmatrix.astype(float))
RmatrixToCSV.to_csv('LSC_TurningRadius.txt', sep=' ', header=False, float_format='%.3f', index=False)

OmegaToCSV = pd.DataFrame(data=omegamatrix.astype(float))
OmegaToCSV.to_csv('LSC_TurningAngle.txt', sep=' ', header=False, float_format='%.3f', index=False)

XToCSV = pd.DataFrame(data=x0Array.astype(float))
XToCSV.to_csv('LSC_X0.txt', sep=' ', header=False, float_format='%.3f', index=False)

YToCSV = pd.DataFrame(data=y0Array.astype(float))
YToCSV.to_csv('LSC_Y0.txt', sep=' ', header=False, float_format='%.3f', index=False)

#ax.set_zlim(0.0, 2.0)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
XX,YY = np.meshgrid(x0Array,y0Array)

surf = ax.plot_surface(XX,YY,omegamatrix, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
