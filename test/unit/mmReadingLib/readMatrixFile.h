#ifndef __READ_MATRIX_FILE__
#define __READ_MATRIX_FILE__

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
#endif
