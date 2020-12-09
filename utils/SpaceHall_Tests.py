import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

### SPACE HALL ###
exrSpaceHall_elevationMap = np.loadtxt(open("spaceHallTest01/spacehall_elevationMap.txt"), skiprows=0)
exrSpaceHall_traversabilityMap = np.loadtxt(open("spaceHallTest01/spacehall_traversabilityMap.txt"), skiprows=0)
exrSpaceHall_validityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_validityMap.txt"), skiprows=0)

res = 0.1
xMap, yMap = np.meshgrid(np.linspace(0,300,301), np.linspace(0,300,301))

xMap = xMap*res - 19.0688738623660825
yMap = yMap*res - 10.9856070496854024

fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, exrSpaceHall_elevationMap, 100, cmap = cm.gist_earth, extend = 'both')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Elevation')

fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, exrSpaceHall_validityMap, alpha = 0.5)
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
s2 = ax.plot(-2.5, 4.0, 'o', color = 'lime') #Sample Position
s2 = ax.plot(-3.67, 2.95, 'or') #Rover Position
ax.add_artist(plt.Circle((-3.67,2.95),1.6, color = 'orange',alpha = 0.4))
cbb.ax.set_title('execution_data_2020-12-03_12-16-36 - Validity')
plt.minorticks_on()
plt.grid(b=True, which='minor', color='w', linestyle='--')
plt.grid(b=True, which='major', color='w', linestyle='-')

fig2, ax2 = plt.subplots(constrained_layout=True)
plot2 = ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 0)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 0)],
        c = 'r',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 1)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 1)],
        c = 'orange',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 2)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 2)],
        c = 'yellow',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 3)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 3)],
        c = 'green',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 4)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 4)],
        c = 'c',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 5)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 5)],
        c = 'lime',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 6)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 6)],
        c = 'grey',
        s = 20
        )
ax2.scatter(xMap[np.where(exrSpaceHall_traversabilityMap == 7)],
        yMap[np.where(exrSpaceHall_traversabilityMap == 7)],
        c = 'black',
        s = 20
        )
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_title('execution_data_2020-12-03_12-16-36 - Traversability Map')
s2 = ax2.plot(-2.5, 4.0, 'o', color = 'lime') #Sample Position
#s2 = ax2.plot(3.9, 9.0, 'ob')
ax2.set_xlim([xMap[0,0],xMap[0,-1]])
ax2.set_ylim([yMap[0,0],yMap[-1,0]])
ax2.add_artist(plt.Circle((-3.67,2.95),1.6, color = 'b',alpha = 0.4))

plt.show()
