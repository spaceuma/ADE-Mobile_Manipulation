import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

### SPACE HALL ###
exrSpaceHall_elevationMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_elevationMap.txt"), skiprows=0)
exrSpaceHall_slopeMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_slopeMap.txt"), skiprows=0)
exrSpaceHall_sdMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_sdMap.txt"), skiprows=0)
exrSpaceHall_validityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_validityMap.txt"), skiprows=0)
exrSpaceHall_costMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_costMap.txt"), skiprows=0)
exrSpaceHall_traversabilityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/spacehall_traversabilityMap.txt"), skiprows=0)

res = 0.1
xMap, yMap = np.meshgrid(np.linspace(0,300,301), np.linspace(0,300,301))

xMap = xMap*res - 11.05
yMap = yMap*res - 6.05

fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, exrSpaceHall_validityMap)
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Validity')

fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, exrSpaceHall_slopeMap, 100, cmap = 'nipy_spectral')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Slope')

fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, exrSpaceHall_sdMap, 100, cmap = 'nipy_spectral')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Spherical Deviation')


fig, ax = plt.subplots(constrained_layout=True)
plott = ax.contourf(xMap, yMap, exrSpaceHall_elevationMap, 100, cmap = cm.gist_earth, extend = 'both')
cbb = fig.colorbar(plott, ax = ax, orientation = 'horizontal')
cbb.ax.set_title('Elevation')


fig1, ax1 = plt.subplots(constrained_layout=True)
plot1 = ax1.contourf(xMap, yMap, exrSpaceHall_costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis (m)')
ax1.set_ylabel('Y-axis (m)')
ax1.set_title('Cost Map')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
ax1.set_facecolor('k')


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
ax2.set_title('Traversability Map')
#s2 = ax2.plot(sample[0], sample[1], 'ob')
s2 = ax2.plot(6.0, 7.0, 'ob')
ax2.set_xlim([xMap[0,0],xMap[0,-1]])
ax2.set_ylim([yMap[0,0],yMap[-1,0]])


### EXR COLMENAR ###
exrColmenar_elevationMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/exrColmenar.txt"), skiprows=0)
exrColmenar_slopeMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/exrcolmenar_slopeMap.txt"), skiprows=0)
exrColmenar_sdMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/exrcolmenar_sdMap.txt"), skiprows=0)
exrColmenar_costMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/exrcolmenar_costMap.txt"), skiprows=0)
res = 0.1
xMap, yMap = np.meshgrid(np.linspace(0,300,301), np.linspace(0,300,301))

fig3, ax6 = plt.subplots(constrained_layout=True)
xMap = xMap*res
yMap = yMap*res
plot6 = ax6.contourf(xMap, yMap, exrColmenar_elevationMap, 100, cmap = cm.gist_earth, extend = 'both')
plot6k = ax6.contour(xMap, yMap, exrColmenar_elevationMap, 50, colors = 'k', alpha = .3)
ax6.set_aspect('equal')
ax6.set_xlabel('X-axis (m)')
ax6.set_ylabel('Y-axis (m)')
ax6.set_title('Elevation Map - exrColmenar')
cb6 = fig3.colorbar(plot6, ax = ax6, orientation = 'horizontal')
cb6.ax.set_title('Elevation')


fig2, (ax3,ax4) = plt.subplots(1,2,constrained_layout=True)

plot3 = ax3.contourf(xMap, yMap, exrColmenar_slopeMap, 40, vmax = 20.0, cmap = 'Reds')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis (m)')
ax3.set_ylabel('Y-axis (m)')
ax3.set_title('Slope Map')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'horizontal')
cb3.ax.set_title('Slope')

plot4 = ax4.contourf(xMap, yMap, exrColmenar_sdMap, 40, vmax = 16.82, cmap = 'Oranges')
ax4.set_aspect('equal')
ax4.set_xlabel('X-axis (m)')
ax4.set_ylabel('Y-axis (m)')
ax4.set_title('Spherical Deviation')
cb4 = fig2.colorbar(plot4, ax = ax4, orientation = 'horizontal')
cb4.ax.set_title('Spherical Deviation')

fig1, ax1 = plt.subplots(constrained_layout=True)
fig1.suptitle('EXR Colmenar Cost Map', fontsize=16)

plot1 = ax1.contourf(xMap, yMap, exrColmenar_costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis (m)')
ax1.set_ylabel('Y-axis (m)')
ax1.set_title('Cost Map')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
ax1.set_facecolor('k')



costMap_shadowing = np.loadtxt(open("../test/unit/data/results/MMMapTest/costMap.txt"), skiprows=0)
costMap_mag = np.loadtxt(open("../test/unit/data/results/MMMapTest/MAGcostMap.txt"), skiprows=0)
costMap_Globalmag = np.loadtxt(open("../test/unit/data/results/MMMapTest/MAGGlobalcostMap.txt"), skiprows=0)
#elevationMap = np.loadtxt(open("../test/unit/data/input/MMMapTest/RG_Colmenar_10cmDEM.csv"), skiprows=0)
#elevationMap = np.loadtxt(open("../test/unit/data/input/MMMapTest/RH1_Zone1_10cmDEM.csv"), skiprows=0)
elevationMap = np.loadtxt(open("../test/unit/data/input/MMMapTest/GalopprennbahnWest_Zone01blurred_10cmDEM.csv"), skiprows=0)
#elevationMap = np.loadtxt(open("../test/unit/data/input/MMMapTest/ColmenarRocks_Nominal_10cmDEM.csv"), skiprows=0)
slopeMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/slopeMap.txt"), skiprows=0)
slopeMap_mag = np.loadtxt(open("../test/unit/data/results/MMMapTest/MAGslopeMap.txt"), skiprows=0)
validityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/validityMap.txt"), skiprows=0)
sdMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/sdMap.txt"), skiprows=0)
sdMap_mag = np.loadtxt(open("../test/unit/data/results/MMMapTest/MAGsdMap.txt"), skiprows=0)
traversabilityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/traversabilityMap.txt"), skiprows=0)
traversabilityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/traversabilityMap.txt"), skiprows=0)
costMap_splitted = np.loadtxt(open("../test/unit/data/results/MMMapTest/costMap_splittedMap.txt"), skiprows=0)
costMap_shadowing[np.where(costMap_shadowing==np.inf)] = np.nan
traversabilityMap[np.where(traversabilityMap==np.inf)] = np.nan
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
#s1 = ax1.plot(sample[0], sample[1], 'ob')
s1 = ax1.plot(6.0, 7.0, 'ob')

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
#s2 = ax2.plot(sample[0], sample[1], 'ob')
s2 = ax2.plot(6.0, 7.0, 'ob')
ax2.set_xlim([xMap[0,0],xMap[0,-1]])
ax2.set_ylim([yMap[0,0],yMap[-1,0]])


fig2, (ax3,ax4) = plt.subplots(1,2,constrained_layout=True)

plot3 = ax3.contourf(xMap, yMap, slopeMap, 40, vmax = 20.0, cmap = 'Reds')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis (m)')
ax3.set_ylabel('Y-axis (m)')
ax3.set_title('Slope Map')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'horizontal')
cb3.ax.set_title('Slope')

plot4 = ax4.contourf(xMap, yMap, sdMap, 40, vmax = 16.82, cmap = 'Oranges')
ax4.set_aspect('equal')
ax4.set_xlabel('X-axis (m)')
ax4.set_ylabel('Y-axis (m)')
ax4.set_title('Spherical Deviation')
cb4 = fig2.colorbar(plot4, ax = ax4, orientation = 'horizontal')
cb4.ax.set_title('Spherical Deviation')


fig3, (ax5,ax6) = plt.subplots(1,2,constrained_layout=True)
plot5 = ax5.contourf(xMap, yMap, validityMap)
ax5.set_aspect('equal')
ax5.set_xlabel('X-axis (m)')
ax5.set_ylabel('Y-axis (m)')
ax5.set_title('Validity Map')
cb5 = fig3.colorbar(plot5, ax = ax5, orientation = 'horizontal')
cb5.ax.set_title('Validity')

plot6 = ax6.contourf(xMap, yMap, elevationMap, 100, cmap = cm.gist_earth, extend = 'both')
plot6k = ax6.contour(xMap, yMap, elevationMap, 50, colors = 'k', alpha = .3)
ax6.set_aspect('equal')
ax6.set_xlabel('X-axis (m)')
ax6.set_ylabel('Y-axis (m)')
ax6.set_title('Elevation Map')
cb6 = fig3.colorbar(plot6, ax = ax6, orientation = 'horizontal')
cb6.ax.set_title('Elevation')

plt.show()
