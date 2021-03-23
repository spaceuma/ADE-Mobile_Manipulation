import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

resultsFile = "../data/results/"
dateFile = "2021-1-27_11-27-15"

# Loading TXT files
traversabilityMap = np.loadtxt(open("./extractedLog/traversabilityMap.txt",'r'), skiprows=0)
costMap = np.loadtxt(open("./extractedLog/costMap.txt"), skiprows=0)
path = np.loadtxt(open("./extractedLog/roverPath.txt",'r'), skiprows=0)
offset = np.loadtxt(open("./extractedLog/offset.txt",'r'), skiprows=0)

# Creating X and Y coordinates
res = 0.1 # This is a hardcoded resolution!
xMap, yMap = np.meshgrid(np.linspace(0,traversabilityMap.shape[1]-1, traversabilityMap.shape[1]), np.linspace(0,traversabilityMap.shape[0]-1, traversabilityMap.shape[0]))
xMap = xMap*res + offset[0]
yMap = yMap*res + offset[1]

# Cost Map
fig1, ax1 = plt.subplots(constrained_layout=True)
plot1 = ax1.contourf(xMap, yMap, costMap, 40, cmap = 'Reds')
ax1.plot(path[:,0] + offset[0], path[:,1] + offset[1], color = 'm')
ax1.plot(path[:,0] + offset[0], path[:,1] + offset[1], 'o', color = 'm')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis (m)')
ax1.set_ylabel('Y-axis (m)')
ax1.set_title('Cost Map')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
ax1.set_facecolor('k')

#Traversability Map
fig2, ax2 = plt.subplots(constrained_layout=True)
plot2 = ax2.scatter(xMap[np.where(traversabilityMap == 0)],
        yMap[np.where(traversabilityMap == 0)],
        c = 'r',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 1)],
        yMap[np.where(traversabilityMap == 1)],
        c = 'orange',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 2)],
        yMap[np.where(traversabilityMap == 2)],
        c = 'yellow',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 3)],
        yMap[np.where(traversabilityMap == 3)],
        c = 'green',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 4)],
        yMap[np.where(traversabilityMap == 4)],
        c = 'c',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 5)],
        yMap[np.where(traversabilityMap == 5)],
        c = 'lime',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 6)],
        yMap[np.where(traversabilityMap == 6)],
        c = 'grey',
        s = 20
        )
ax2.scatter(xMap[np.where(traversabilityMap == 7)],
        yMap[np.where(traversabilityMap == 7)],
        c = 'black',
        s = 20
        )
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_title('Traversability Map')
ax2.set_xlim([xMap[0,0],xMap[0,-1]])
ax2.set_ylim([yMap[0,0],yMap[-1,0]])
ax2.plot(path[:,0] + offset[0], path[:,1] + offset[1], color = 'm')
ax2.plot(path[:,0] + offset[0], path[:,1] + offset[1], 'o', color = 'm')
plt.show()
