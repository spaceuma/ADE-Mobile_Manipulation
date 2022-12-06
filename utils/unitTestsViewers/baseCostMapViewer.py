import sys
import numpy as np
import matplotlib.pyplot as plt

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

costMap = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_costMap_0"+representationNumber+".txt",'r'), skiprows=0)
costMap[np.where(costMap==np.inf)] = np.nan

xMap, yMap = \
          np.meshgrid(np.linspace(0,costMap.shape[1]-1,costMap.shape[1]), \
                      np.linspace(0,costMap.shape[0]-1,costMap.shape[0]))

res = 0.1

xMap = xMap*res
yMap = yMap*res

fig1, ax1 = plt.subplots()
plot1 = ax1.contourf(xMap, yMap, costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
plt.show()
