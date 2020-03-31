# ADE_Mobile-Manipulation
C++ code for European H2020 Project ADE

## Installation

In order to get this repository together with its submodules, use the following command ([Source](https://www.vogella.com/tutorials/GitSubmodules/article.html)):

```
git clone --recursive ADE-Mobile_Manipulation
```

Then, to install the component and the required packages:

```
cd ~/ADE-Mobile_Manipulation
chmod u+x install.sh
```

## External packages

### Required (the indicated version is the one used)

- OpenCV 3.2.0

- DART 6.9.2

### Optional (mainly for results visualization)

- python3-pip

- mayavi

- PyQt5

- python3-matplotlib

## Folder Structure


| Folder            |       Description                             |
| ----------------- | ------------------------                      |
| deps/             | External dependencies                         |
| ├── ESA-Trajectory_Control/ | (Git submodule) Libraries for path tracking|
| ├── UMA-Coupled_Control/ | (Git submodule) Libraries for arm-rover coordinated motion|
| ├── UMA-PathAndMotion_Planning/ | Libraries for planning rover and arm paths|
| doc/              | Documentation                                 |
| spike/            | Individual portions of code to try new things |
| src/              | Source files                                  |
| ├── MobileManipMotionPlanner.h | Main Interface Class             |
| ├── MobileManipMap.h | Class to handle Map Information            |
| ├── MotionPlan.h | The Plan to move both arm and rover            |
| ├── MobileManipExecutor.h | Class to provide commands during execution|
| ├── mmFileManagerLib/ | Library to read external files|
| │   ├──  mmFileManager.h | Functions to read from external text files|
| ├── types/ | All types used by the component|
| │   ├──  MMError.h | MM Error Codes|
| │   ├──  MMStatus.h | MM Status Codes|
| │   ├──  RoverGuidance_Dem.h | DEM type used by Rover Guidance|
| │   ├──  base/ | Types using base namespace|
| │   ├──  proxy_library/ | Types using proxy_library namespace|
| test/             | Test files                                    |
| UML/              | UML Project files                             |
