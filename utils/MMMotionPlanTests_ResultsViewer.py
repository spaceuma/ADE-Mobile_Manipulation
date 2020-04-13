import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

res = 0.1

elevation_map = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/ColmenarRocks_smaller_10cmDEM.csv"), skiprows=0)

costMap_shadowing = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/costMap_Shadowing.txt"), skiprows=0)
costMap_no_shadowing = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/costMap_noShadowing.txt"), skiprows=0)
costMap_shadowing[np.where(costMap_shadowing==np.inf)] = np.nan
costMap_no_shadowing[np.where(costMap_no_shadowing==np.inf)] = np.nan

xMap, yMap = \
          np.meshgrid(np.linspace(0,elevation_map.shape[1]-1,elevation_map.shape[1]), \
                      np.linspace(0,elevation_map.shape[0]-1,elevation_map.shape[0]))

xMap = xMap*res
yMap = yMap*res

path_shadowing_01 = np.loadtxt(open("../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_01.txt"), skiprows=0)
path_shadowing_02 = np.loadtxt(open("../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_02.txt"), skiprows=0)
path_no_shadowing_01 = np.loadtxt(open("../test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_path_01.txt"), skiprows=0)
path_no_shadowing_02 = np.loadtxt(open("../test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_path_02.txt"), skiprows=0)
#path_no_shadowing_02 = np.loadtxt(open("../test/unit/data/results/MMMotionPlanTest/sample_outoftunnel_path_01.txt"), skiprows=0)

sample = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/sample_pos.txt"), skiprows = 0)
rover = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/rover_pos_01.txt"), skiprows = 0)
rover_02 = np.loadtxt(open("../test/unit/data/input/MMMotionPlanTest/rover_pos_02.txt"), skiprows = 0)

fig1, (ax1,ax2) = plt.subplots(figsize = (9,6),nrows = 1,ncols = 2,constrained_layout=True)
fig1.suptitle('Unit Test 02 - MMMotionPlan Cost Maps', fontsize=16)

plot1 = ax1.contourf(xMap, yMap, costMap_shadowing, 40, cmap = 'Oranges')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
ax1.set_title('Cost Map Rectified\n(Rover Facing to Sample)')
ax1.set_facecolor('k')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
#p12 = ax1.plot(path_no_shadowing_01[:,0],path_no_shadowing_01[:,1],'lime', linewidth = 2)
#p12 = ax1.plot(path_no_shadowing_02[:,0],path_no_shadowing_02[:,1],'lime', linewidth = 2)
p1 = ax1.plot(path_shadowing_01[:,0],path_shadowing_01[:,1],'c', linewidth = 2)
p1 = ax1.plot(path_shadowing_02[:,0],path_shadowing_02[:,1],'c', linewidth = 2)
s1 = ax1.plot(sample[0], sample[1], 'or')
circ1 = ax1.add_artist(plt.Circle((sample[0],sample[1]),1.0, color='m', fill = False))
circ2 = ax1.add_artist(plt.Circle((sample[0],sample[1]),1.5, color='y', fill = False))
circ3 = ax1.add_artist(plt.Circle((sample[0],sample[1]),1.8, color='b', fill = False))
s12 = ax1.plot(rover[0], rover[1], 'oy')
s12 = ax1.plot(rover_02[0], rover_02[1], 'oy')
#s13 = ax1.plot(path_no_shadowing_01[-1,0], path_no_shadowing_01[-1,1], 'o',color = 'lime')
#s13 = ax1.plot(path_no_shadowing_02[-1,0], path_no_shadowing_02[-1,1], 'o',color = 'lime')
s14 = ax1.plot(path_shadowing_01[-1,0], path_shadowing_01[-1,1], 'oc')
s14 = ax1.plot(path_shadowing_02[-1,0], path_shadowing_02[-1,1], 'oc')
ax1.legend((p1[0],s1[0],s12[0],circ1,circ2,circ3),('Facing Sample Path', 'Sample Position', 'Initial Rover Position','Straight Area', 'Entry Area - Maneuvering', 'Entry Area - Access'), loc = 'lower right')

plot2 = ax2.contourf(xMap, yMap, costMap_no_shadowing, 40, cmap = 'Oranges')
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_title('Cost Map non Rectified')
ax2.set_facecolor('k')
cb2 = fig1.colorbar(plot2, ax = ax2, orientation = 'horizontal')
cb2.ax.set_title('Cost')
#p21 = ax2.plot(path_shadowing_01[:,0],path_shadowing_01[:,1],'c')
#p21 = ax2.plot(path_shadowing_02[:,0],path_shadowing_02[:,1],'c')
p2 = ax2.plot(path_no_shadowing_01[:,0],path_no_shadowing_01[:,1],'lime')
p2 = ax2.plot(path_no_shadowing_02[:,0],path_no_shadowing_02[:,1],'lime')
s2 = ax2.plot(sample[0], sample[1], 'or')
s21 = ax2.plot(rover[0], rover[1], 'oy')
s21 = ax2.plot(rover_02[0], rover_02[1], 'oy')
s23 = ax2.plot(path_no_shadowing_01[-1,0], path_no_shadowing_01[-1,1], 'o',color = 'lime')
s33 = ax2.plot(path_no_shadowing_02[-1,0], path_no_shadowing_02[-1,1], 'o',color = 'lime')
#s34 = ax2.plot(path_no_shadowing_01[-1,0], path_no_shadowing_01[-1,1], 'o',color = 'lime')
#s34 = ax2.plot(path_shadowing_02[-1,0], path_shadowing_02[-1,1], 'oc')
ax2.legend((p2[0],s2[0],s21[0]),('Non Rectified Path', 'Sample Position', 'Initial Rover Position'), loc = 'lower right')



fig2, ax3 = plt.subplots(constrained_layout = True)
fig2.suptitle('Unit Test 02 - MMMotionPlan Elevation Map', fontsize=16)
plot3 = ax3.contourf(xMap, yMap, elevation_map, 100, cmap = cm.gist_earth, extend='both')
plot3k = ax3.contour(xMap, yMap, elevation_map, 50, colors = 'k', alpha = .3)
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis (m)')
ax3.set_ylabel('Y-axis (m)')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'vertical')
cb3.ax.set_title('Elevation (m)\nabove sea level')
p3 = ax3.plot(path_no_shadowing_01[:,0],path_no_shadowing_01[:,1],'lime')
p3 = ax3.plot(path_no_shadowing_02[:,0],path_no_shadowing_02[:,1],'lime')
p31 = ax3.plot(path_shadowing_01[:,0],path_shadowing_01[:,1],'c')
p31 = ax3.plot(path_shadowing_02[:,0],path_shadowing_02[:,1],'c')
s3 = ax3.plot(sample[0], sample[1], 'or')
s31 = ax3.plot(rover[0], rover[1], 'oy')
s31 = ax3.plot(rover_02[0], rover_02[1], 'oy')
s14 = ax3.plot(path_shadowing_01[-1,0], path_shadowing_01[-1,1], 'oc')
s14 = ax3.plot(path_shadowing_02[-1,0], path_shadowing_02[-1,1], 'oc')
s33 = ax3.plot(path_no_shadowing_01[-1,0], path_no_shadowing_01[-1,1], 'o',color = 'lime')
s33 = ax3.plot(path_no_shadowing_02[-1,0], path_no_shadowing_02[-1,1], 'o',color = 'lime')
s34 = ax3.plot(path_shadowing_01[-1,0], path_shadowing_01[-1,1], 'oc')
ax3.legend((p31[0],p3[0],s2[0],s21[0]),('Rectified Path', 'Non-Rectified Path', 'Sample Position', 'Initial Rover Position'), loc = 'lower right')


plt.show()



