import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

resultsFile = "../data/results/"
dateFile = "2021-1-27_11-27-15"


# Loading TXT files
elevationMap = np.loadtxt(open("./extractedLog/elevationMap.txt",'r'), skiprows=0)
validityMap = np.loadtxt(open("./extractedLog/validityMap.txt",'r'), skiprows=0)
slopeMap = np.loadtxt(open("./extractedLog/slopeMap.txt",'r'), skiprows=0)
sdMap = np.loadtxt(open("./extractedLog/sdMap.txt",'r'), skiprows=0)
offset = np.loadtxt(open("./extractedLog/offset.txt",'r'), skiprows=0)

# Creating X and Y coordinates
res = 0.1 # This is a hardcoded resolution!
xMap, yMap = np.meshgrid(np.linspace(0,elevationMap.shape[1]-1, elevationMap.shape[1]), np.linspace(0,elevationMap.shape[0]-1, elevationMap.shape[0]))
xMap = xMap*res + offset[0]
yMap = yMap*res + offset[1]

# Elevation Map
fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, elevationMap, 100, cmap = cm.gist_earth, extend = 'both')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Elevation (m)')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')

# Slope Map
fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, slopeMap, 100, cmap = 'nipy_spectral')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Slope (deg)')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')

# SD (Roughness) Map
fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, sdMap, 100, cmap = 'nipy_spectral')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Spherical Deviation (deg)')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')

# Validity Map
fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, validityMap, alpha = 0.5)
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Validity [bool]')
plt.minorticks_on()
plt.grid(b=True, which='minor', color='w', linestyle='--')
plt.grid(b=True, which='major', color='w', linestyle='-')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')

plt.show()
