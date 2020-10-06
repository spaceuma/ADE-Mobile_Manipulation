# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 10:04:42 2020

@author: Richi
"""

from scipy.optimize import fsolve
import math
import numpy as np
import matplotlib.pyplot as plt

beta = math.radians(30.0) #Counter-clockwise rotation from heading vector to sampling vector
L2 = 0.5
x0 = 0.0 
y0 = 1.5

def equations(i):
    L1,alpha = i
    f1 = -x0 + L1*math.cos(alpha) + L2*(math.cos(alpha)*math.cos(beta) - math.sin(alpha)*math.sin(beta))
    f2 = -y0 + L1 + L1*math.sin(alpha) + L2*(math.sin(alpha)*math.cos(beta) + math.sin(beta)*math.cos(alpha))
    return (f1,f2)

LL,alpha1 = fsolve(equations,(10.0, math.radians(30.0)))

print(equations((LL,alpha1)))

alpha4 = math.atan2(LL + LL*math.sin(alpha1), LL*math.cos(alpha1))
alpha3 = math.pi - 2*alpha4
R = 0.5*math.sqrt(math.pow(LL*math.cos(alpha1),2) + math.pow(LL + LL*math.sin(alpha1),2))/math.sin(alpha3/2.0)

plt.style.use('default')
plt.rcParams["font.family"] = "Constantia"
plt.rcParams['mathtext.fontset'] = 'cm'
plt.rcParams['mathtext.rm'] = 'serif'
fig, ax = plt.subplots(figsize=(8, 8), \
      nrows = 1, ncols = 1, \
      sharex = 'all', sharey = 'all')


V = np.array([[LL*math.cos(alpha1),LL + LL*math.sin(alpha1)],[-2,2],[4,-7]])
origin = [0], [0] # origin point

ax.quiver([0,R,R,0], 
          [0,0,0,0], 
          [LL*math.cos(alpha1),-R, -R + LL*math.cos(alpha1), -math.cos(math.pi/2 - beta)*L2], 
          [LL + LL*math.sin(alpha1),0, LL + LL*math.sin(alpha1), math.sin(math.pi/2 - beta)*L2], 
          angles='xy', scale_units='xy', scale=1)
ax.scatter(x0, y0, facecolor = 'r', edgecolor='black', s=60)
ax.set_aspect('equal')
fig.tight_layout()
plt.show()