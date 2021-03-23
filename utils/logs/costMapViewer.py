
import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

costMap = np.loadtxt(open("./extractedLog/costMap.txt"), skiprows=0)
costMap[np.where(costMap==np.inf)] = np.nan

res = 0.1
elevation_map = np.loadtxt(open("./extractedLog/elevationMap.txt"), skiprows=0)
path = np.loadtxt(open("./extractedLog/roverPath.txt",'r'), skiprows=0)


xMap, yMap = \
          np.meshgrid(np.linspace(0,elevation_map.shape[1]-1,elevation_map.shape[1]), \
                      np.linspace(0,elevation_map.shape[0]-1,elevation_map.shape[0]))

xMap = xMap*res
yMap = yMap*res


fig1, (ax1,ax2) = plt.subplots(figsize = (9,6),nrows = 1,ncols = 2,constrained_layout=True)
plot1 = ax1.contourf(xMap, yMap, costMap, 40, cmap = 'Oranges')
ax1.set_title('Cost Map')
#s1 = ax1.plot(5.6,4.2, 'or')
#s11 = ax1.plot(1.0, 1.0, 'og')
p31 = ax1.plot(path[:,0],path[:,1],'c')
ax1.set_facecolor('k')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis [m]')
ax1.set_ylabel('Y-axis [m]')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
ax2.set_title('Elevation Map')
plot3 = ax2.contourf(xMap, yMap, elevation_map, 100, cmap = cm.gist_earth, extend='both')
plot3k = ax2.contour(xMap, yMap, elevation_map, 50, colors = 'k', alpha = .3)
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis [m]')
ax2.set_ylabel('Y-axis [m]')
cb3 = fig1.colorbar(plot3, ax = ax2, orientation = 'horizontal')
cb3.ax.set_title('Elevation above sea level (0m) + 1000 [m]')
#p31 = ax2.plot(path[:,0],path[:,1],'c')
#s3 = ax2.plot(5.6,4.2,'or')
#s31 = ax2.plot(1.0, 1.0, 'og')
#c1 = ax2.add_artist(plt.Circle((path[-1,0],path[-1,1]),1.7, color = 'b',alpha = .5))
#ax2.legend((s3[0]),'Sample Position', loc = 'upper left')
#ax1.text(5.5,3.65,'Sample', bbox=dict(facecolor='red', alpha=0.5))
#ax2.text(5.5,3.65,'Sample', bbox=dict(facecolor='red', alpha=0.5))
plt.show()
