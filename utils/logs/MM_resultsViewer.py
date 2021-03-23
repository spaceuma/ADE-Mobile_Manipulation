import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import pandas as pd

res = 0.1
elevation_map = np.loadtxt(open("./extractedLog/elevationMap.txt"), skiprows=0)
cost_map = np.loadtxt(open("./extractedLog/costMap.txt"), skiprows=0)
cost_map[np.where(cost_map==np.inf)] = np.nan

path = np.loadtxt(open("./extractedLog/roverPath.txt",'r'), skiprows=0)

xMap, yMap = \
          np.meshgrid(np.linspace(0,elevation_map.shape[1]-1,elevation_map.shape[1]), \
                      np.linspace(0,elevation_map.shape[0]-1,elevation_map.shape[0]))

xMap = xMap*res + 492664.435 - 492697.82
yMap = yMap*res + 5880515.055 - 5880516.06

rover_pos = path[:,0]

fig, ax = plt.subplots(constrained_layout = True)
fig.suptitle('Galopprennbahn Test', fontsize=16)
plot3 = ax.contourf(xMap, yMap, elevation_map, 100, cmap = cm.gist_earth, extend='both')
plot3k = ax.contour(xMap, yMap, elevation_map, 50, colors = 'k', alpha = .3)
ax.set_aspect('equal')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')
cb3 = fig.colorbar(plot3, ax = ax, orientation = 'vertical')
cb3.ax.set_title('Elevation (m)')
for i in range(path.shape[0]):
     c1 = ax.add_artist(plt.Circle((path[i,0] + 492664.435 - 492697.82,path[i,1] + 5880515.055 - 5880516.06),0.3, color = 'b'))
p1 = ax.plot(path[:,0] + 492664.435 - 492697.82,path[:,1] + 5880515.055 - 5880516.06,'oc', markersize=1)
p2 = ax.plot(rover_pos[2], rover_pos[5], 'oy', markersize=1)
#q1 = ax.quiver(rover_pos[-1,0],rover_pos[-1,1], np.cos(rover_pos[-1,3]), np.sin(rover_pos[-1,3]))
#ax.scatter(rover_pos[-1,0],rover_pos[-1,1], color = 'orange', s = 100)
ax.legend((c1, p1[0],p2[0]),('Corridor Area', 'Computed Path',\
        'Rover Positions'), loc = 'lower right')

fig, ax = plt.subplots(constrained_layout = True)
fig.suptitle('Galopprennbahn Test', fontsize=16)
plot3 = ax.contourf(xMap, yMap, cost_map, 100, cmap = 'Oranges')
#plot3k = ax.contour(xMap, yMap, cost_map, 50, colors = 'k', alpha = .3)
ax.set_aspect('equal')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')
ax.set_facecolor('k')
cb3 = fig.colorbar(plot3, ax = ax, orientation = 'vertical')
cb3.ax.set_title('Cost')
for i in range(path.shape[0]):
     c1 = ax.add_artist(plt.Circle((path[i,0] + 492664.435 - 492697.82,path[i,1] + 5880515.055 - 5880516.06),0.3, color = 'b'))
p1 = ax.plot(path[:,0] + 492664.435 - 492697.82,path[:,1] + + 5880515.055 - 5880516.06,'oc', markersize=1)
p2 = ax.plot(rover_pos[2], rover_pos[5], 'oy', markersize=1)
#q1 = ax.quiver(rover_pos[-1,0],rover_pos[-1,1], np.cos(rover_pos[-1,3]), np.sin(rover_pos[-1,3]))
#ax.scatter(rover_pos[-1,0],rover_pos[-1,1], color = 'orange', s = 100)
ax.legend((c1, p1[0],p2[0]),('Corridor Area', 'Computed Path',\
        'Rover Positions'), loc = 'lower right')

plt.show()

