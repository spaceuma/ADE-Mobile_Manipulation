import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

res = 0.1
elevation_map = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/ColmenarRocks_smaller_10cmDEM.csv"), skiprows=0)
cost_map = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/costMap_Shadowing.txt"), skiprows=0)

path = np.loadtxt(open("../test/unit/data/input/MMExecutorTest/path.txt"), skiprows=0)
xMap, yMap = \
          np.meshgrid(np.linspace(0,elevation_map.shape[1]-1,elevation_map.shape[1]), \
                      np.linspace(0,elevation_map.shape[0]-1,elevation_map.shape[0]))

xMap = xMap*res
yMap = yMap*res

rover_pos = np.loadtxt(open("../test/unit/data/results/MMExecutorTest/roverRealPos.txt"), skiprows=0)
rover_loc_pos = np.loadtxt(open("../test/unit/data/results/MMExecutorTest/roverEstimatedPos.txt"), skiprows=0)

fig, ax = plt.subplots(constrained_layout = True)
fig.suptitle('Unit Test 03 - MMExecutor', fontsize=16)
plot3 = ax.contourf(xMap, yMap, elevation_map, 100, cmap = cm.gist_earth, extend='both')
plot3k = ax.contour(xMap, yMap, elevation_map, 50, colors = 'k', alpha = .3)
ax.set_aspect('equal')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')
cb3 = fig.colorbar(plot3, ax = ax, orientation = 'vertical')
cb3.ax.set_title('Elevation (m)')
for i in range(path.shape[0]):
     c1 = ax.add_artist(plt.Circle((path[i,0],path[i,1]),0.3, color = 'b'))
p1 = ax.plot(path[:,0],path[:,1],'oc')
p3 = ax.plot(rover_loc_pos[:,0], rover_loc_pos[:,1], 'or')
p2 = ax.plot(rover_pos[:,0], rover_pos[:,1], 'oy')
#q1 = ax.quiver(rover_pos[-1,0],rover_pos[-1,1], np.cos(rover_pos[-1,3]), np.sin(rover_pos[-1,3]))
#ax.scatter(rover_pos[-1,0],rover_pos[-1,1], color = 'orange', s = 100)
ax.legend((c1, p1[0],p2[0],p3[0]),('Corridor Area', 'Computed Path',\
        'Rover Groundtruth Positions', 'Rover Localization Positions'), loc = 'lower right')

plt.show()

