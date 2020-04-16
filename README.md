# ADE_Mobile-Manipulation
C++ code for European H2020 Project ADE

## Table of Contents
  * [Installation](#installation)
  * [External Packages](#external-packages)
  * [Folder Structure](#installation)

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

## External Packages

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
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  mmFileManager.h | Functions to read from external text files|
| ├── types/ | All types used by the component|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ├──  MMError.h | MM Error Codes|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  MMStatus.h | MM Status Codes|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  RoverGuidance_Dem.h | DEM type used by Rover Guidance|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   ├──  base/ | Types using base namespace|
| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  ├──  proxy_library/ | Types using proxy_library namespace|
| test/             | Test files                                    |
| UML/              | UML Project files                             |

