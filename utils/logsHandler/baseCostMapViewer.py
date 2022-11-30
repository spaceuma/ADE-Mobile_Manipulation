
import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

costMap = np.loadtxt(open("./extractedLog/costMap.txt"), skiprows=0)
costMap[np.where(costMap==np.inf)] = np.nan

fig1, ax1 = plt.subplots()
plot1 = ax1.contourf(costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')
plt.show()
