import sys
import numpy as np
import matplotlib.pyplot as plt

costMap_shadowing = np.loadtxt(open("../test/unit/data/results/costMap_Shadowing.txt"), skiprows=0)
costMap_no_shadowing = np.loadtxt(open("../test/unit/data/results/costMap_noShadowing.txt"), skiprows=0)
costMap_shadowing[np.where(costMap_shadowing==np.inf)] = np.nan

fig1, (ax1,ax2) = plt.subplots(1,2,constrained_layout=True)

plot1 = ax1.contourf(costMap_shadowing, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
ax1.set_title('Cost Map with Shadowing')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')

plot2 = ax2.contourf(costMap_no_shadowing, 40, cmap = 'Reds')
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_title('Cost Map without Shadowing')
cb2 = fig1.colorbar(plot2, ax = ax2, orientation = 'horizontal')
cb2.ax.set_title('Cost')

plt.show()
