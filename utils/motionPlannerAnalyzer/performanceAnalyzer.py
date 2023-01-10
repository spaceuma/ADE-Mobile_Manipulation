import sys
import numpy as np
from math import sqrt
from numpy import dot
import os
import matplotlib.pyplot as plt

# Function to obtain the index of a particular approach
def getApproach(string):
    if string == "END":
        return 0
    elif string == "PROGRESSIVE":
        return 1
    elif string == "BEGINNING":
        return 2
    elif string == "DECOUPLED":
        return 3
    return 4

# Check that the log file exists
if not os.path.exists("motionPlanResultsLog.txt"):
    print("ERROR: log file not found")
    sys.exit()

# Initialize variables
computation_time = []
std_devs_self_collision_distance = []
means_self_collision_distance = []
arm_required_time = []
base_required_time = []
arm_moving_time = []
base_moving_time = []
total_required_time = []

# Different approaches
approaches = ["END", "PROGRESSIVE", "BEGINNING", "DECOUPLED"]

# Extract data from log
with open("motionPlanResultsLog.txt", "r") as file_object:
    file_text = file_object.readlines()
    i = 0
    for line in file_text:
        data = line.split(" ")
        filtered_line = []
        for value in data:
            value = float(value.rstrip("\n"))
            filtered_line.append(value)

        if i == 0:
            computation_time.append(filtered_line)
        elif i == 1:
            std_devs_self_collision_distance.append(filtered_line)
        elif i == 2:
            means_self_collision_distance.append(filtered_line)
        elif i == 3:
            arm_required_time.append(filtered_line)
        elif i == 4:
            base_required_time.append(filtered_line)
        elif i == 5:
            arm_moving_time.append(filtered_line)
        elif i == 6:
            base_moving_time.append(filtered_line)
        elif i == 7:
            total_required_time.append(filtered_line)

        i += 1
        if i >= 8:
            i = 0


mean_computation_time = np.empty(np.size(approaches))
mean_self_collision_distance = np.empty(np.size(approaches))
mean_arm_required_time = np.empty(np.size(approaches))
mean_base_required_time = np.empty(np.size(approaches))
mean_arm_moving_time = np.empty(np.size(approaches))
mean_base_moving_time = np.empty(np.size(approaches))
mean_total_required_time = np.empty(np.size(approaches))

std_dev_computation_time = np.zeros(np.size(approaches))
std_dev_self_collision_distance = np.zeros(np.size(approaches))
std_dev_arm_required_time = np.zeros(np.size(approaches))
std_dev_base_required_time = np.zeros(np.size(approaches))
std_dev_arm_moving_time = np.zeros(np.size(approaches))
std_dev_base_moving_time = np.zeros(np.size(approaches))
std_dev_total_required_time = np.zeros(np.size(approaches))

for approach in approaches:
    index = getApproach(approach)
    num_data = np.size(computation_time[index])
    print("Approach "+approach)

    '''print("Num data: "+str(np.size(computation_time[index])))
    print("Num data: "+str(np.size(std_devs_self_collision_distance[index])))
    print("Num data: "+str(np.size(means_self_collision_distance[index])))
    print("Num data: "+str(np.size(arm_required_time[index])))
    print("Num data: "+str(np.size(base_required_time[index])))
    print("Num data: "+str(np.size(arm_moving_time[index])))
    print("Num data: "+str(np.size(base_moving_time[index])))
    print("Num data: "+str(np.size(total_required_time[index])))'''

    mean_computation_time[index] = np.cumsum(computation_time[index])[-1]/num_data
    mean_self_collision_distance[index] = np.cumsum(means_self_collision_distance[index])[-1]/num_data
    mean_arm_required_time[index] = np.cumsum(arm_required_time[index])[-1]/num_data
    mean_base_required_time[index] = np.cumsum(base_required_time[index])[-1]/num_data
    mean_arm_moving_time[index] = np.cumsum(arm_moving_time[index])[-1]/num_data
    mean_base_moving_time[index] = np.cumsum(base_moving_time[index])[-1]/num_data
    mean_total_required_time[index] = np.cumsum(total_required_time[index])[-1]/num_data

    for i in range(0, num_data):
        std_dev_computation_time[index] += pow(computation_time[index][i] - mean_computation_time[index], 2)
        std_dev_self_collision_distance[index] += pow(std_devs_self_collision_distance[index][i], 2)
        std_dev_arm_required_time[index] += pow(arm_required_time[index][i] - mean_arm_required_time[index], 2)
        std_dev_base_required_time[index] += pow(base_required_time[index][i] - mean_base_required_time[index], 2)
        std_dev_arm_moving_time[index] += pow(arm_moving_time[index][i] - mean_arm_moving_time[index], 2)
        std_dev_base_moving_time[index] += pow(base_moving_time[index][i] - mean_base_moving_time[index], 2)
        std_dev_total_required_time[index] += pow(total_required_time[index][i] - mean_total_required_time[index], 2)

    std_dev_computation_time[index] = sqrt(std_dev_computation_time[index]/num_data)
    std_dev_self_collision_distance[index] = sqrt(std_dev_self_collision_distance[index]/num_data)
    std_dev_arm_required_time[index] = sqrt(std_dev_arm_required_time[index]/num_data)
    std_dev_base_required_time[index] = sqrt(std_dev_base_required_time[index]/num_data)
    std_dev_arm_moving_time[index] = sqrt(std_dev_arm_moving_time[index]/num_data)
    std_dev_base_moving_time[index] = sqrt(std_dev_base_moving_time[index]/num_data)
    std_dev_total_required_time[index] = sqrt(std_dev_total_required_time[index]/num_data)

#proportional_arm_time = arm_moving_time
#proportional_base_time = base_moving_time
proportional_total_time = total_required_time

#mean_proportional_arm_time = np.empty(np.size(approaches))
#mean_proportional_base_time = np.empty(np.size(approaches))
mean_proportional_total_time = np.empty(np.size(approaches))

#std_dev_proportional_arm_time = np.zeros(np.size(approaches))
#std_dev_proportional_base_time = np.zeros(np.size(approaches))
std_dev_proportional_total_time = np.zeros(np.size(approaches))

for approach in approaches:
    index = getApproach(approach)
    num_data = np.size(computation_time[index])

    for i in range(0, num_data):
        #proportional_arm_time[index][i] = arm_moving_time[index][i]/total_required_time[getApproach("DECOUPLED")][i]
        #proportional_base_time[index][i] = base_moving_time[index][i]/total_required_time[getApproach("DECOUPLED")][i]
        proportional_total_time[index][i] = total_required_time[index][i]/total_required_time[getApproach("DECOUPLED")][i]

    #mean_proportional_arm_time[index] = np.cumsum(proportional_arm_time[index])[-1]/num_data
    #mean_proportional_base_time[index] = np.cumsum(proportional_base_time[index])[-1]/num_data
    mean_proportional_total_time[index] = np.cumsum(proportional_total_time[index])[-1]/num_data

    for i in range(0, num_data):
        #std_dev_proportional_arm_time[index] += pow(proportional_arm_time[index][i] - mean_proportional_arm_time[index], 2)
        #std_dev_proportional_base_time[index] += pow(proportional_base_time[index][i] - mean_proportional_base_time[index], 2)
        std_dev_proportional_total_time[index] += pow(proportional_total_time[index][i] - mean_proportional_total_time[index], 2)

    #std_dev_proportional_arm_time[index] = sqrt(std_dev_proportional_arm_time[index]/num_data)
    #std_dev_proportional_base_time[index] = sqrt(std_dev_proportional_base_time[index]/num_data)
    std_dev_proportional_total_time[index] = sqrt(std_dev_proportional_total_time[index]/num_data)

plt.rcParams.update({'font.size': 18})

# Interchange position of the approaches (for better visualization)
approaches[0],approaches[3] = approaches[3],approaches[0]
approaches[1],approaches[3] = approaches[3],approaches[1]

mean_computation_time[0],mean_computation_time[3] = mean_computation_time[3],mean_computation_time[0]
mean_computation_time[1],mean_computation_time[3] = mean_computation_time[3],mean_computation_time[1]

std_dev_computation_time[0],std_dev_computation_time[3] = std_dev_computation_time[3],std_dev_computation_time[0]
std_dev_computation_time[1],std_dev_computation_time[3] = std_dev_computation_time[3],std_dev_computation_time[1]

mean_self_collision_distance[0],mean_self_collision_distance[3] = mean_self_collision_distance[3],mean_self_collision_distance[0]
mean_self_collision_distance[1],mean_self_collision_distance[3] = mean_self_collision_distance[3],mean_self_collision_distance[1]

std_dev_self_collision_distance[0],std_dev_self_collision_distance[3] = std_dev_self_collision_distance[3],std_dev_self_collision_distance[0]
std_dev_self_collision_distance[1],std_dev_self_collision_distance[3] = std_dev_self_collision_distance[3],std_dev_self_collision_distance[1]

mean_proportional_total_time[0],mean_proportional_total_time[3] = mean_proportional_total_time[3],mean_proportional_total_time[0]
mean_proportional_total_time[1],mean_proportional_total_time[3] = mean_proportional_total_time[3],mean_proportional_total_time[1]

std_dev_proportional_total_time[0],std_dev_proportional_total_time[3] = std_dev_proportional_total_time[3],std_dev_proportional_total_time[0]
std_dev_proportional_total_time[1],std_dev_proportional_total_time[3] = std_dev_proportional_total_time[3],std_dev_proportional_total_time[1]

mean_total_required_time[0],mean_total_required_time[3] = mean_total_required_time[3],mean_total_required_time[0]
mean_total_required_time[1],mean_total_required_time[3] = mean_total_required_time[3],mean_total_required_time[1]

mean_base_moving_time[0],mean_base_moving_time[3] = mean_base_moving_time[3],mean_base_moving_time[0]
mean_base_moving_time[1],mean_base_moving_time[3] = mean_base_moving_time[3],mean_base_moving_time[1]

mean_arm_moving_time[0],mean_arm_moving_time[3] = mean_arm_moving_time[3],mean_arm_moving_time[0]
mean_arm_moving_time[1],mean_arm_moving_time[3] = mean_arm_moving_time[3],mean_arm_moving_time[1]

fig1, (ax1, ax2) = plt.subplots(1, 2, sharey=False)
plot1 = ax1.errorbar(approaches, mean_computation_time, std_dev_computation_time, linestyle='None', ecolor= 'black', marker='s', mfc='y', mec = 'black', ms=8, capsize = 8)
ax1.set_ylabel('Time (s)')
ax1.set_ylim(bottom = 0)
ax1.grid()

plot4 = ax2.errorbar(approaches, mean_self_collision_distance, std_dev_self_collision_distance, linestyle='None', ecolor= 'black', marker='s', mfc='y', mec = 'black', ms=8, capsize = 8)
ax2.set_ylabel('Distance (m)')
ax2.set_ylim(bottom = 0)
ax2.grid()
#ax2.legend(["Min","Mean","Max"])

fig2, (ax3, ax4) = plt.subplots(1, 2, sharey=False)
#plot5 = ax3.errorbar(approaches, mean_proportional_arm_time, std_dev_proportional_arm_time, linestyle='None', marker='o', capsize = 5)
#plot6 = ax3.errorbar(approaches, mean_proportional_base_time, std_dev_proportional_base_time, linestyle='None', marker='x', capsize = 5)
plot5 = ax3.errorbar(approaches, mean_proportional_total_time, std_dev_proportional_total_time, linestyle='None',  ecolor= 'black', marker='s', mfc='y', mec = 'black', ms=8, capsize = 8)
ax3.set_ylabel('Motion execution time w.r.t. DECOUPLED')
ax3.set_ylim(bottom = 0)
ax3.grid()

x = np.arange(len(approaches))
width = 0.12

plt.rcParams["hatch.linewidth"] = 4

#plot8 = ax4.bar(x+width, mean_total_required_time, width, edgecolor='black', label='Total')
#plot7 = ax4.bar(x, mean_base_moving_time, width, edgecolor='black', label='Base')
#plot6 = ax4.bar(x-width, mean_arm_moving_time, width, edgecolor='black', label='Arm')
plot9 = ax4.bar(approaches[0], mean_total_required_time[0], width, edgecolor='limegreen', label='Arm', facecolor='limegreen')
plot8 = ax4.bar(approaches[0], mean_base_moving_time[0], width, edgecolor='blue', label='Base', facecolor='blue')
plot7 = ax4.bar(approaches[1:], mean_total_required_time[1:], width, facecolor='limegreen', label='Coupled', edgecolor='blue', hatch=r"\\" )
plot6 = ax4.bar(approaches[1:], mean_base_moving_time[1:]-mean_arm_moving_time[1:], width, edgecolor='blue', label='Base', facecolor='blue')
ax4.set_ylabel('Time (s)')
ax4.set_ylim(bottom = 0)
ax4.grid()
ax4.legend(["Arm","Base"])
#ax4.legend(bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0)
#ax4.set_xticks(x, approaches)
#ax4.set_axisbelow(True)

#ax4.bar_label(plot8, padding=3)
#fig2.tight_layout()

plt.show()
