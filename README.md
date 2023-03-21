<h1 align="center">ADE-Mobile_Manipulation</h1>

<h3 align="center">Combined path and motion planner for workspace restricted mobile manipulators in planetary exploration</h3>


<h4 align="center">C++ library used in H2020 EC Project Autonomous DEcision making in very long traverses (ADE)</h4>

![image](https://user-images.githubusercontent.com/37618448/226581695-0f960cca-62d7-466f-a9d4-4c4b6082c896.png)


*Author*: [Gonzalo Jesús Paz Delgado](https://github.com/gonzalopd96), gonzalopd96@uma.es

*Supervisor*: [Carlos J. Pérez del Pulgar](https://github.com/carlibiri), carlosperez@uma.es

*Organization*: [Space Robotics Lab, University of Malaga](https://www.uma.es/space-robotics)


## Table of Contents
  * [Description](#description)
  * [Installation](#installation)
  * [Examples](#examples)
  * [Dependencies](#dependencies)
  * [File tree](#file-tree)
  * [Citation](#citation)


## Description
Coupled path and motion planner for Mobile Manipulation (MM), focused on rovers with a restricted arm workspace. 
- First, a Fast Marching Method (FMM) based path planner generates a safe trajectory to reach the goal vicinity, avoiding obstacles and non-traversable areas in the scenario. The path planner is able to control the final rover orientation to ensure that the goal is finally reachable by the arm.
- Second, a FMM 3D based motion planner generates the arm joints motion profile, by creating a 3D tunnel-like cost volume surrounding the already computed rover base trajectory. This tunnel makes use of an offline-computed safe workspace of the manipulator, thus ensuring that no self-collision will occur during the planned motion.

Check the [Simulation and field tests video](https://youtu.be/I-cEbNgtQ9c).

## Installation

In order to get this repository together with its submodules, use the following command ([Source](https://www.vogella.com/tutorials/GitSubmodules/article.html)):

```
git clone --recursive ADE-Mobile_Manipulation
```

Then, to install the component and the required packages:

```
cd ~/ADE-Mobile_Manipulation
chmod u+x install.sh
sudo ./install.sh
```

## Examples
There are several unit tests that can be used as examples. To run them:

```
./runUnitTests
```

Then, the results will be saved in:
```
test/unit/data/results
```

The input for the unit tests (rover and goal poses, maps...) can be modified in:
```
test/unit/data/input
```

The unit test results can be viewed using the provided python3 utils. For example, to represent the evolution of the arm during the motion plan:
```
cd utils/unitTestsViewers/
python3 armFullPlanViewer.py <approach>
```

Or to represent the base motion plan:

```
python3 baseMotionPlanViewer.py <approach>
```

Where ```<approach>``` is:
- ```0``` for end deployment within the coupled approach
- ```1``` for progressive deployment within the coupled approach
- ```2``` for beginning deployment within the coupled approach
- ```3``` for decoupled solution

## Dependencies

### Required (the indicated version is the one used)

- OpenCV 3.2.0

- [DART 6.9.2](https://dartsim.github.io/install_dart_on_ubuntu.html)

### Optional (mainly for results visualization)

- python3-pip

- mayavi

- PyQt5

- python3-matplotlib

## File Tree


| Directory         |       Description                             |
| ----------------- | ------------------------                      |
| data/             | Data required as input by the planner         |
| deps/             | External dependencies                         |
| ├── ESA-Trajectory_Control/ | (Git submodule) Libraries for path tracking|
| ├── UMA-PathAndMotion_Planning/ | Libraries for planning rover and arm paths|
| doc/              | Documentation                                 |
| spike/            | Individual portions of code to try new things |
| src/              | Source files                                  |
| ├── MobileManipMotionPlanner.h | Main Interface Class             |
| ├── MobileManipMap.h | Class to handle Map Information            |
| ├── MotionPlan.h | The Plan to move both arm and rover            |
| ├── MobileManipExecutor.h | Class to provide commands during execution|
| ├── mmFileManagerLib/ | Library to read external files|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  mmFileManager.h | Functions to read from external text files|
| ├── types/ | All types used by the component|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ├──  MMError.h | MM Error Codes|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  MMStatus.h | MM Status Codes|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  RoverGuidance_Dem.h | DEM type used by Rover Guidance|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  base/ | Types using base namespace|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  ├──  proxy_library/ | Types using proxy_library namespace|
| test/             | Test files                                    |
| UML/              | UML Project files                             |
| utils/            | Utilities                                     |
| ├── armCollisionsViewer/ | Check and view if the arm collides for a given config |
| ├── armReachabilityComputer/ | Compute and represent the arm reachability volume |
| ├── armSingleSweepingComputer/ | Example computation of a simple sweeping movement for the end effector |
| ├── logsHandler/ | Logs extraction and different pythonr utils for representation |
| ├── motionPlannerAnalyzer/ | Process unit tests log data to obtain results about performance |
| ├── unitTestsViewers/ | Python scripts to represent the results of the unit tests |

## Citation

If this work was helpful for your research, please consider citing the following BibTeX entry:

@article{author = {G. J. Paz-Delgado and J. R. Sánchez-Ibáñez and R. Domínguez and C. J. Pérez-del-Pulgar and F. Kirchner and A. García-Cerezo},
   title = {Combined path and motion planning for workspace restricted mobile manipulators in planetary exploration},
   year = {2023}
}
