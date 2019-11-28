#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include "RoverGuidance_Dem.h"
#include "readMatrixFile.h"

void readMatrixFile(std::string map_file, double res, double &n_row, double &n_col, std::vector<double>& vector_elevationData)
{
    std::cout << "Reading map " << map_file << std::endl;
    std::string line;
    std::ifstream e_file(map_file.c_str(), std::ios::in);
    n_row = 0;
    n_col = 0;
    vector_elevationData.clear();
    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                vector_elevationData.push_back(val);
                n_col++;
            }
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
        std::cout << "Map of " << n_col << " x " << n_row << " loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the map file" << std::endl;
    }
    //double elevationData[rgDem->v_elevationData_m.size()];
        //double elevationData[vector_elevationData.size()];
        //rgDem->p_heightData_m = elevationData;
    /*for (uint j = 0; j<rgDem->rows; j++)
    {
        for (uint i = 0; i<rgDem->cols; i++)
        {
            std::cout << "Initial Value is " << rgDem->p_heightData_m[i+j*rgDem->cols] << std::endl;
        }
    }*/
}





