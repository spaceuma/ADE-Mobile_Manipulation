import sys
import numpy as np
import matplotlib.pyplot as plt

costMap_shadowing = np.loadtxt(open("../test/unit/data/results/MMMapTest/costMap_Shadowing.txt"), skiprows=0)
costMap_no_shadowing = np.loadtxt(open("../test/unit/data/results/MMMapTest/costMap_noShadowing.txt"), skiprows=0)
costMap_splitted = np.loadtxt(open("../test/unit/data/results/MMMapTest/costMap_splittedMap.txt"), skiprows=0)
costMap_shadowing[np.where(costMap_shadowing==np.inf)] = np.nan
costMap_no_shadowing[np.where(costMap_no_shadowing==np.inf)] = np.nan
costMap_splitted[np.where(costMap_splitted==np.inf)] = np.nan

res = 0.1

xMap, yMap = \
          np.meshgrid(np.linspace(0,costMap_shadowing.shape[1]-1,costMap_shadowing.shape[1]), \
                      np.linspace(0,costMap_shadowing.shape[0]-1,costMap_shadowing.shape[0]))

xMap = xMap*res
yMap = yMap*res

sample = np.loadtxt(open("../test/unit/data/input/MMMapTest/sample_pos.txt"), skiprows = 0)

fig1, (ax1,ax2) = plt.subplots(1,2,constrained_layout=True)
fig1.suptitle('Unit Test 01 - MMMap Class', fontsize=16)

plot1 = ax1.contourf(xMap, yMap, costMap_shadowing, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis (m)')
ax1.set_ylabel('Y-axis (m)')
ax1.set_title('Cost Map')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
ax1.set_facecolor('k')
s1 = ax1.plot(sample[0], sample[1], 'ob')

plot2 = ax2.contourf(costMap_no_shadowing, 40, cmap = 'Reds')
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_title('Cost Map without Shadowing')
cb2 = fig1.colorbar(plot2, ax = ax2, orientation = 'horizontal')
cb2.ax.set_title('Cost')
s2 = ax2.plot(sample[0]*10, sample[1]*10, 'ob')



fig2, ax3 = plt.subplots(constrained_layout=True)
plot3 = ax3.contourf(xMap, yMap, costMap_splitted, 40, cmap = 'Reds')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis (m)')
ax3.set_ylabel('Y-axis (m)')
ax3.set_title('Cost Map')
cb3 = fig2.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb3.ax.set_title('Cost')
ax3.set_facecolor('k')
#s3 = ax3.plot(sample[0], sample[1], 'ob')

plt.show()
