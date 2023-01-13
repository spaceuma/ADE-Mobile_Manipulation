import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

if len(sys.argv) != 2:
  print('ERROR. The script needs one additional input:')
  print('0 for END deployment')
  print('1 for PROGRESSIVE deployment')
  print('2 for BEGINNING deployment')
  print('3 for DECOUPLED solution')
  sys.exit()

representationNumber = str(int(sys.argv[1])+1)

if (int(representationNumber) < 1) or (int(representationNumber) > 4):
  print('ERROR. The provided input is not valid:')
  print('0 for END deployment')
  print('1 for PROGRESSIVE deployment')
  print('2 for BEGINNING deployment')
  print('3 for DECOUPLED solution')
  sys.exit()

if int(representationNumber) == 1:
  print('Case END deployment')
elif int(representationNumber) == 2:
  print('Case PROGRESSIVE deployment')
elif int(representationNumber) == 3:
  print('Case BEGINNING deployment')
elif int(representationNumber) == 4:
  print('Case DECOUPLED deployment')

res = 0.1

elevation_map = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/GalopprennbahnWest_Zone01_10cmDEM.csv"), skiprows=0)
#traversabilityMap = np.loadtxt(open("../test/unit/data/results/MMMapTest/traversabilityMap.txt"), skiprows=0)

costMap = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_costMap_0"+representationNumber+".txt",'r'), skiprows=0)
costMap[np.where(costMap==np.inf)] = np.nan

path = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_path_0"+representationNumber+".txt",'r'), skiprows=0)

sample = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/sample_pos_01.txt"), skiprows = 0)
rover = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/rover_pos_01.txt"), skiprows = 0)

traversabilityMap = costMap

xMap, yMap = \
          np.meshgrid(np.linspace(0,elevation_map.shape[1]-1,elevation_map.shape[1]), \
                      np.linspace(0,elevation_map.shape[0]-1,elevation_map.shape[0]))

xMap = xMap*res
yMap = yMap*res


fig1, (ax2,ax1) = plt.subplots(figsize = (9,6),nrows = 1,ncols = 2,constrained_layout=True)
fig1.suptitle('Unit Test - MMMotionPlan Cost Maps', fontsize=16)

plot1 = ax1.contourf(xMap, yMap, costMap, 40, cmap = 'Oranges')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
ax1.set_title('Cost Map Rectified')
ax1.set_facecolor('k')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
p1 = ax1.plot(path[:,0],path[:,1],'c', linewidth = 2)
s1 = ax1.plot(sample[0], sample[1], 'or')
circ1 = ax1.add_artist(plt.Circle((sample[0],sample[1]),1.0, color='m', fill = False))
circ2 = ax1.add_artist(plt.Circle((sample[0],sample[1]),2.3, color='y', fill = False))
circ3 = ax1.add_artist(plt.Circle((sample[0],sample[1]),2.3+0.142, color='b', fill = False))
s12 = ax1.plot(rover[0], rover[1], 'oy')
s14 = ax1.plot(path[-1,0], path[-1,1], 'oc')
ax1.legend((p1[0],s1[0],s12[0],circ1,circ2,circ3),('Facing Sample Path', 'Sample Position', 'Initial Rover Position','Straight Area', 'Entry Area - Maneuvering', 'Entry Area - Access'), loc = 'lower right')

plot2 = ax2.scatter(xMap, yMap,c = traversabilityMap, cmap = 'Set1', s = 40)
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_title('Traversability Map')
ax2.set_facecolor('k')
cb2 = fig1.colorbar(plot2, ax = ax2, orientation = 'horizontal')
cb2.ax.set_title('Traversability')
p21 = ax2.plot(path[:,0],path[:,1],'c')
s2 = ax2.plot(sample[0], sample[1], 'or')
s21 = ax2.plot(rover[0], rover[1], 'oy')
ax2.legend((p21[0],s2[0],s21[0]),('Facing Sample, Path', 'Sample Position', 'Initial Rover Position'), loc = 'lower right')
ax2.text(0.1,6.0,'Traversable Area')
ax2.text(3.2,6.0,'Dilation 2')
ax2.text(4.8,4.8,'Dilation 1')
ax2.text(5.0,3.6,'Sampling Area')
ax2.text(6.0,6.0,'Obstacles')
ax2.set_xlim([xMap[0,0],xMap[0,-1]])
ax2.set_ylim([yMap[0,0],yMap[-1,0]])

fig2, ax3 = plt.subplots(constrained_layout = True)
fig2.suptitle('Unit Test 02 - MMMotionPlan Elevation Map', fontsize=16)
plot3 = ax3.contourf(xMap, yMap, elevation_map, 100, cmap = cm.gist_earth, extend='both')
plot3k = ax3.contour(xMap, yMap, elevation_map, 50, colors = 'k', alpha = .3)
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis (m)')
ax3.set_ylabel('Y-axis (m)')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'vertical')
cb3.ax.set_title('Elevation (m)\nabove sea level')
p31 = ax3.plot(path[:,0],path[:,1],'c')
s3 = ax3.plot(sample[0], sample[1], 'or')
s31 = ax3.plot(rover[0], rover[1], 'oy')
#c1 = ax3.add_artist(plt.Circle((rover[0],rover[1]),1.7, color = 'b',alpha = .5))
c1 = ax3.add_artist(plt.Circle((path[-1,0],path[-1,1]),1.7, color = 'b',alpha = .5))
s14 = ax3.plot(path[-1,0], path[-1,1], 'oc')
s34 = ax3.plot(path[-1,0], path[-1,1], 'oc')
ax3.legend((p31[0],s2[0],s21[0]),('Rectified Path', 'Sample Position', 'Initial Rover Position'), loc = 'lower right')


plt.show()



