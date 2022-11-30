
import sys
import numpy as np
import matplotlib.pyplot as plt

costMap = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_costMap_01.txt"), skiprows=0)
costMap[np.where(costMap==np.inf)] = np.nan

path = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_01.txt"), skiprows=0)


fig1, ax1 = plt.subplots(tight_layout=True)
plot1 = ax1.contourf(costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.plot(10.0*path[:,0],10.0*path[:,1],'b')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')

plt.show()
