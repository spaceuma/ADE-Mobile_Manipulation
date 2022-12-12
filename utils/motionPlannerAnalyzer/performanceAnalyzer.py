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
min_self_collision_distance = []
max_self_collision_distance = []
avg_self_collision_distance = []
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
            min_self_collision_distance.append(filtered_line)
        elif i == 2:
            max_self_collision_distance.append(filtered_line)
        elif i == 3:
            avg_self_collision_distance.append(filtered_line)
        elif i == 4:
            arm_required_time.append(filtered_line)
        elif i == 5:
            base_required_time.append(filtered_line)
        elif i == 6:
            arm_moving_time.append(filtered_line)
        elif i == 7:
            base_moving_time.append(filtered_line)
        elif i == 8:
            total_required_time.append(filtered_line)

        i += 1
        if i >= 9:
            i = 0


mean_computation_time = np.empty(np.size(approaches))
mean_min_self_collision_distance = np.empty(np.size(approaches))
mean_max_self_collision_distance = np.empty(np.size(approaches))
mean_avg_self_collision_distance = np.empty(np.size(approaches))
mean_arm_required_time = np.empty(np.size(approaches))
mean_base_required_time = np.empty(np.size(approaches))
mean_arm_moving_time = np.empty(np.size(approaches))
mean_base_moving_time = np.empty(np.size(approaches))
mean_total_required_time = np.empty(np.size(approaches))

std_dev_computation_time = np.zeros(np.size(approaches))
std_dev_min_self_collision_distance = np.zeros(np.size(approaches))
std_dev_max_self_collision_distance = np.zeros(np.size(approaches))
std_dev_avg_self_collision_distance = np.zeros(np.size(approaches))
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
    print("Num data: "+str(np.size(min_self_collision_distance[index])))
    print("Num data: "+str(np.size(max_self_collision_distance[index])))
    print("Num data: "+str(np.size(avg_self_collision_distance[index])))
    print("Num data: "+str(np.size(arm_required_time[index])))
    print("Num data: "+str(np.size(base_required_time[index])))
    print("Num data: "+str(np.size(arm_moving_time[index])))
    print("Num data: "+str(np.size(base_moving_time[index])))
    print("Num data: "+str(np.size(total_required_time[index])))'''

    mean_computation_time[index] = np.cumsum(computation_time[index])[-1]/num_data
    mean_min_self_collision_distance[index] = np.cumsum(min_self_collision_distance[index])[-1]/num_data
    mean_max_self_collision_distance[index] = np.cumsum(max_self_collision_distance[index])[-1]/num_data
    mean_avg_self_collision_distance[index] = np.cumsum(avg_self_collision_distance[index])[-1]/num_data
    mean_arm_required_time[index] = np.cumsum(arm_required_time[index])[-1]/num_data
    mean_base_required_time[index] = np.cumsum(base_required_time[index])[-1]/num_data
    mean_arm_moving_time[index] = np.cumsum(arm_moving_time[index])[-1]/num_data
    mean_base_moving_time[index] = np.cumsum(base_moving_time[index])[-1]/num_data
    mean_total_required_time[index] = np.cumsum(total_required_time[index])[-1]/num_data

    for i in range(0, num_data):
        std_dev_computation_time[index] += pow(computation_time[index][i] - mean_computation_time[index], 2)
        std_dev_min_self_collision_distance[index] += pow(min_self_collision_distance[index][i] - mean_min_self_collision_distance[index], 2)
        std_dev_max_self_collision_distance[index] += pow(max_self_collision_distance[index][i] - mean_max_self_collision_distance[index], 2)
        std_dev_avg_self_collision_distance[index] += pow(avg_self_collision_distance[index][i] - mean_avg_self_collision_distance[index], 2)
        std_dev_arm_required_time[index] += pow(arm_required_time[index][i] - mean_arm_required_time[index], 2)
        std_dev_base_required_time[index] += pow(base_required_time[index][i] - mean_base_required_time[index], 2)
        std_dev_arm_moving_time[index] += pow(arm_moving_time[index][i] - mean_arm_moving_time[index], 2)
        std_dev_base_moving_time[index] += pow(base_moving_time[index][i] - mean_base_moving_time[index], 2)
        std_dev_total_required_time[index] += pow(total_required_time[index][i] - mean_total_required_time[index], 2)

    std_dev_computation_time[index] = sqrt(std_dev_computation_time[index]/num_data)
    std_dev_min_self_collision_distance[index] = sqrt(std_dev_min_self_collision_distance[index]/num_data)
    std_dev_max_self_collision_distance[index] = sqrt(std_dev_max_self_collision_distance[index]/num_data)
    std_dev_avg_self_collision_distance[index] = sqrt(std_dev_avg_self_collision_distance[index]/num_data)
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

fig1, ax1 = plt.subplots()
plot1 = ax1.errorbar(approaches, mean_computation_time, std_dev_computation_time, linestyle='None', ecolor= 'black', marker='s', mfc='y', mec = 'black', ms=8, capsize = 8)
ax1.set_ylabel('Computation time (s)')
ax1.set_ylim(bottom = 0)
ax1.grid()

fig2, ax2 = plt.subplots()
plot2 = ax2.errorbar(approaches, mean_min_self_collision_distance, std_dev_min_self_collision_distance, linestyle='None', ecolor= 'black', marker='s', mfc='b', mec = 'black', ms=8, capsize = 8)
plot3 = ax2.errorbar(approaches, mean_avg_self_collision_distance, std_dev_avg_self_collision_distance, linestyle='None', ecolor= 'black', marker='s', mfc='y', mec = 'black', ms=8, capsize = 8)
plot4 = ax2.errorbar(approaches, mean_max_self_collision_distance, std_dev_max_self_collision_distance, linestyle='None', ecolor= 'black', marker='s', mfc='r', mec = 'black', ms=8, capsize = 8)
ax2.set_ylabel('Distance to self collisions (m)')
ax2.set_ylim(bottom = 0)
ax2.grid()
ax2.legend(["Minimum","Mean","Maximum"])

fig3, ax3 = plt.subplots()
#plot5 = ax3.errorbar(approaches, mean_proportional_arm_time, std_dev_proportional_arm_time, linestyle='None', marker='o', capsize = 5)
#plot6 = ax3.errorbar(approaches, mean_proportional_base_time, std_dev_proportional_base_time, linestyle='None', marker='x', capsize = 5)
plot5 = ax3.errorbar(approaches, mean_proportional_total_time, std_dev_proportional_total_time, linestyle='None',  ecolor= 'black', marker='s', mfc='y', mec = 'black', ms=8, capsize = 8)
ax3.set_ylabel('Proportion of motion execution time w.r.t. DECOUPLED solution')
ax3.set_ylim(bottom = 0)
ax3.grid()

fig4, ax4 = plt.subplots()
plot6 = ax4.errorbar(approaches, mean_arm_moving_time, linestyle='None', marker='s', mec = 'black', ms=10, capsize = 8)
plot7 = ax4.errorbar(approaches, mean_base_moving_time, linestyle='None', marker='o', mec = 'black', ms=10, capsize = 8)
plot8 = ax4.errorbar(approaches, mean_total_required_time, linestyle='None', marker='x', mec = 'black', ms=11, capsize = 8)
ax4.set_ylabel('Avg. motion execution time (s)')
ax4.set_ylim(bottom = 0)
ax4.grid()
ax4.legend(["Arm","Base","Total"])

plt.show()
