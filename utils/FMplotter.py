
import sys
import numpy as np
import matplotlib.pyplot as plt

costMap = np.loadtxt(open("../test/unit/data/results/costMap.txt"), skiprows=0)
costMap[np.where(costMap==np.inf)] = np.nan

total_cost_G = np.loadtxt(open("../test/unit/data/results/TMapGoal.txt"), skiprows=0)
total_cost_G[np.where(total_cost_G==-1)] = np.amax(total_cost_G)

total_cost_S = np.loadtxt(open("../test/unit/data/results/TMapStart.txt"), skiprows=0)
total_cost_S[np.where(total_cost_S==-1)] = np.amax(total_cost_S)

path = np.loadtxt(open("../test/unit/data/results/path.txt"), skiprows=0)


fig1, (ax1, ax2, ax3) = plt.subplots(1, 3, tight_layout=True)
plot1 = ax1.contourf(costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.plot(path[:,0],path[:,1],'b')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')

plot2 = ax2.contourf(total_cost_G, 100, cmap = 'viridis', alpha = .5)
ax2.contour(total_cost_G, 100, cmap = 'viridis')
ax2.plot(path[:,0],path[:,1],'r')
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')

plot3 = ax3.contourf(total_cost_S, 100, cmap = 'viridis', alpha = .5)
ax3.contour(total_cost_S, 100, cmap = 'viridis')
ax3.plot(path[:,0],path[:,1],'r')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis')
ax3.set_ylabel('Y-axis')
plt.show()
