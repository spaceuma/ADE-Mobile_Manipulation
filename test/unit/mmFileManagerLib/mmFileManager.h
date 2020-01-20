#ifndef __MM_FILE_MANAGER__
#define __MM_FILE_MANAGER__

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "RoverGuidance_Dem.h"
#include "Waypoint.hpp"

using namespace base;

void readMatrixFile(std::string map_file,
                    std::vector<std::vector<double>> &vector_elevationData);
void readPath(std::string s_path_file,
	      std::vector<Waypoint> &vw_path);
Waypoint getWaypoint(std::string s_path_file);
void savePath(std::vector<base::Waypoint> *roverPath, std::string s_path_file);
void saveProfile(std::vector<std::vector<double>> *pvvd_arm_motion_profile, std::string s_path_file);
#endif
