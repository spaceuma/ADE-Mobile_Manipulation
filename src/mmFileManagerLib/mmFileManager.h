// MIT License
// -----------
//
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Authors: J. Ricardo Sánchez Ibáñez, Gonzalo J. Paz Delgado, Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __MM_FILE_MANAGER__
#define __MM_FILE_MANAGER__

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "RoverGuidance_InputDataStruct.h"
#include "Waypoint.hpp"

using namespace base;

void readVectorFile(std::string vector_file, std::vector<double> & vector);
void writeMatrixFile(std::string s_matrix_dir, std::vector<std::vector<double>> & vvd_inputmatrix);
void writeMatrixFile(std::string s_matrix_dir, std::vector<std::vector<int>> & vvd_inputmatrix);
void writeMatrixFile(std::string s_matrix_dir, std::vector<std::vector<int8_t>> & vvd_inputmatrix);
void readMatrixFile(std::string map_file, std::vector<std::vector<double>> & vector_elevationData);
void readMatrixFileCommas(std::string map_file,
                          std::vector<std::vector<double>> & vector_elevationData);
void readReachabilityMap(std::string map_file,
                         std::vector<std::vector<std::vector<double>>> * reachabilityMap,
                         std::vector<double> * resolutions,
                         std::vector<double> * minValues);
void readPath(std::string s_path_file, std::vector<Waypoint> & vw_path);
Waypoint getWaypoint(std::string s_path_file);
void savePath(std::vector<base::Waypoint> * roverPath, std::string s_path_file);
void saveValue(double d_value, std::string s_path_file);
void saveVector(std::vector<double> * pvd_vector, std::string s_path_file);
void saveProfile(std::vector<std::vector<double>> * pvvd_arm_motion_profile,
                 std::string s_path_file);
void saveVolume(std::vector<std::vector<std::vector<double>>> * pvvvd_volume,
                std::string s_path_file);
void saveVolume(std::vector<std::vector<std::vector<double>>> * pvvvd_volume,
                std::vector<double> * resolutions,
                std::vector<double> * minValues,
                std::string s_path_file);
#endif
