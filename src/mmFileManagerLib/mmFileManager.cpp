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

#include "mmFileManager.h"

void readVectorFile(std::string vector_file, std::vector<double> & vector)
{
    std::ifstream e_file(vector_file.c_str(), std::ios::in);

    vector.clear();
    if(e_file.is_open())
    {
        std::string cell;
        while(std::getline(e_file, cell, ' '))
        {
            double val;
            std::stringstream numeric_value(cell);
            numeric_value >> val;
            vector.push_back(val);
        }
        e_file.close();
    }
    else
    {
        std::cout << "[readVectorFile]  Problem opening the path file " << vector_file << std::endl;
        throw std::exception();
    }
}

void writeMatrixFile(std::string s_matrix_dir, std::vector<std::vector<double>> & vvd_inputmatrix)
{
    std::ofstream of_targetfile;

    of_targetfile.open(s_matrix_dir);
    for(int j = 0; j < vvd_inputmatrix.size(); j++)
    {
        for(int i = 0; i < vvd_inputmatrix[0].size(); i++)
        {
            of_targetfile << (double)vvd_inputmatrix[j][i] << " ";
        }
        of_targetfile << "\n";
    }
    of_targetfile.close();
}

void writeMatrixFile(std::string s_matrix_dir, std::vector<std::vector<int>> & vvd_inputmatrix)
{
    std::ofstream of_targetfile;

    of_targetfile.open(s_matrix_dir);
    for(int j = 0; j < vvd_inputmatrix.size(); j++)
    {
        for(int i = 0; i < vvd_inputmatrix[0].size(); i++)
        {
            of_targetfile << (double)vvd_inputmatrix[j][i] << " ";
        }
        of_targetfile << "\n";
    }
    of_targetfile.close();
}

void writeMatrixFile(std::string s_matrix_dir, std::vector<std::vector<int8_t>> & vvd_inputmatrix)
{
    std::ofstream of_targetfile;

    of_targetfile.open(s_matrix_dir);
    for(int j = 0; j < vvd_inputmatrix.size(); j++)
    {
        for(int i = 0; i < vvd_inputmatrix[0].size(); i++)
        {
            of_targetfile << (double)vvd_inputmatrix[j][i] << " ";
        }
        of_targetfile << "\n";
    }
    of_targetfile.close();
}

void readMatrixFile(std::string map_file, std::vector<std::vector<double>> & vector_elevationData)
{
    std::string line;
    std::ifstream e_file(map_file.c_str(), std::ios::in);

    double n_row = 0;
    double n_col = 0;
    vector_elevationData.clear();
    std::vector<double> row;
    if(e_file.is_open())
    {
        while(std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while(std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            vector_elevationData.push_back(row);
            row.clear();
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
    }
    else
    {
        std::cout << "[readMatrixFile] Problem opening the path file " << map_file << std::endl;
        throw std::exception();
    }
}

void readMatrixFileCommas(std::string map_file,
                          std::vector<std::vector<double>> & vector_elevationData)
{
    std::string line;
    std::ifstream e_file(map_file.c_str(), std::ios::in);

    double n_row = 0;
    double n_col = 0;
    vector_elevationData.clear();
    std::vector<double> row;
    if(e_file.is_open())
    {
        while(std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while(std::getline(ss, cell, ','))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            vector_elevationData.push_back(row);
            row.clear();
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
    }
    else
    {
        std::cout << "[readMatrixFile] Problem opening the path file " << map_file << std::endl;
        throw std::exception();
    }
}

void readReachabilityMap(std::string map_file,
                         std::vector<std::vector<std::vector<double>>> * reachabilityMap,
                         std::vector<double> * resolutions,
                         std::vector<double> * minValues)
{
    std::vector<double> * sizes = new std::vector<double>;

    std::string line;
    std::ifstream e_file(map_file.c_str(), std::ios::in);

    if(e_file.is_open())
    {
        for(int i = 0; i < 3; i++)
        {
            std::getline(e_file, line);
            std::stringstream ss(line);
            std::string cell;

            while(std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                switch(i)
                {
                    case 0:
                        sizes->push_back(val);
                        break;
                    case 1:
                        resolutions->push_back(val);
                        break;
                    case 2:
                        minValues->push_back(val);
                        break;
                    default:
                        break;
                }
            }
        }

        reachabilityMap->resize(
            (*sizes)[0],
            std::vector<std::vector<double>>((*sizes)[1], std::vector<double>((*sizes)[2])));

        int layer = 0, column = 0;
        for(int row = 0; row < (*sizes)[0]; row++)
        {
            layer = 0;
            column = 0;
            std::getline(e_file, line);
            std::stringstream ss(line);
            std::string cell;
            for(int i = 0; i < (*sizes)[1] * (*sizes)[2]; i++)
            {
                std::getline(ss, cell, ' ');
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                (*reachabilityMap)[row][column][layer] = val;
                layer++;
                if(layer > (*sizes)[2] - 1)
                {
                    layer = 0;
                    column++;
                }
            }
        }
        e_file.close();
    }
    else
    {
        std::cout << "[readReachabilityMap] Problem opening the path file " << map_file
                  << std::endl;
        throw std::exception();
    }
}

void readPath(std::string s_path_file, std::vector<Waypoint> & vw_path)
{
    std::string line;
    std::ifstream e_file(s_path_file.c_str(), std::ios::in);
    vw_path.clear();
    Waypoint w_current;
    int n_row = 0;
    if(e_file.is_open())
    {
        while(std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            double val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value(cell);
            numeric_value >> val;
            w_current.position[0] = val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value1(cell);
            numeric_value1 >> val;
            w_current.position[1] = val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value2(cell);
            numeric_value2 >> val;
            w_current.position[2] = val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value3(cell);
            numeric_value3 >> val;
            w_current.heading = val;

            vw_path.push_back(w_current);
            n_row++;
        }
        e_file.close();
    }
    else
    {
        std::cout << "Problem opening the map file" << std::endl;
    }
}

Waypoint getWaypoint(std::string s_path_file)
{
    std::string line;
    std::ifstream e_file(s_path_file.c_str(), std::ios::in);

    Waypoint w_current;

    std::getline(e_file, line);
    std::stringstream ss(line);
    std::string cell;
    double val;
    std::getline(ss, cell, ' ');
    std::stringstream numeric_value(cell);
    numeric_value >> val;
    w_current.position[0] = val;

    std::getline(ss, cell, ' ');
    std::stringstream numeric_value1(cell);
    numeric_value1 >> val;
    w_current.position[1] = val;

    std::getline(ss, cell, ' ');
    std::stringstream numeric_value2(cell);
    numeric_value2 >> val;
    w_current.position[2] = val;

    e_file.close();
    return w_current;
}

void savePath(std::vector<base::Waypoint> * roverPath, std::string s_path_file)
{
    std::ofstream pathFile;
    pathFile.open(s_path_file);
    for(int j = 0; j < roverPath->size(); j++)
    {
        pathFile << roverPath->at(j).position[0] << " " << roverPath->at(j).position[1] << " "
                 << roverPath->at(j).position[2] << " " << roverPath->at(j).heading << "\n";
    }
    pathFile.close();
}

void saveVector(std::vector<double> * pvd_vector, std::string s_path_file)
{
    std::ofstream f_vector;
    f_vector.open(s_path_file);
    for(int i = 0; i < (*pvd_vector).size(); i++)
    {
        f_vector << (*pvd_vector)[i] << " ";
    }
    f_vector.close();
}

void saveProfile(std::vector<std::vector<double>> * pvvd_arm_motion_profile,
                 std::string s_path_file)
{
    std::ofstream f_arm_motion;
    f_arm_motion.open(s_path_file);
    for(int j = 0; j < pvvd_arm_motion_profile->size(); j++)
    {
        for(int i = 0; i < (*pvvd_arm_motion_profile)[0].size(); i++)
        {
            f_arm_motion << (*pvvd_arm_motion_profile)[j][i] << " ";
        }
        f_arm_motion << "\n";
    }
    f_arm_motion.close();
}

void saveVolume(std::vector<std::vector<std::vector<double>>> * pvvvd_volume,
                std::string s_path_file)
{
    std::ofstream f_volume;
    f_volume.open(s_path_file);

    f_volume << (*pvvvd_volume).size() << " " << (*pvvvd_volume)[0].size() << " "
             << (*pvvvd_volume)[0][0].size() << "\n";
    for(int j = 0; j < pvvvd_volume->size(); j++)
    {
        for(int i = 0; i < (*pvvvd_volume)[0].size(); i++)
        {
            for(int k = 0; k < (*pvvvd_volume)[0][0].size(); k++)
            {
                f_volume << (*pvvvd_volume)[j][i][k] << " ";
            }
        }
        f_volume << "\n";
    }

    f_volume.close();
}

void saveVolume(std::vector<std::vector<std::vector<double>>> * pvvvd_volume,
                std::vector<double> * resolutions,
                std::vector<double> * minValues,
                std::string s_path_file)
{
    std::ofstream f_volume;
    f_volume.open(s_path_file);

    f_volume << (*pvvvd_volume).size() << " " << (*pvvvd_volume)[0].size() << " "
             << (*pvvvd_volume)[0][0].size() << "\n";
    f_volume << (*resolutions)[0] << " " << (*resolutions)[1] << " " << (*resolutions)[2] << "\n";
    f_volume << (*minValues)[0] << " " << (*minValues)[1] << " " << (*minValues)[2] << "\n";
    for(int j = 0; j < pvvvd_volume->size(); j++)
    {
        for(int i = 0; i < (*pvvvd_volume)[0].size(); i++)
        {
            for(int k = 0; k < (*pvvvd_volume)[0][0].size(); k++)
            {
                f_volume << (*pvvvd_volume)[j][i][k] << " ";
            }
        }
        f_volume << "\n";
    }

    f_volume.close();
}
