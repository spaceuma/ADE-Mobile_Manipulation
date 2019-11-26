#include "ArmPlanner.h"
#include <ctime>
#include <fstream>
#include <gtest/gtest.h>
#include <math.h>

std::vector<std::vector<double>> readMatrixFile(std::string cost_map_file)
{
    std::cout << "Reading cost_map " << cost_map_file << std::endl;
    std::vector<std::vector<double>> cost_map_matrix;
    std::string line;
    std::ifstream e_file(cost_map_file.c_str(), std::ios::in);
    double n_row = 0, n_col = 0;
    std::vector<double> row;

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
                row.push_back(val);
                n_col++;
            }
            cost_map_matrix.push_back(row);
            row.clear();
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
        std::cout << "Cost map of " << n_col << " x " << n_row << " loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the cost_map file" << std::endl;
        return cost_map_matrix;
    }
    return cost_map_matrix;
}

std::vector<std::vector<std::vector<double>>> readMatrixFile3D(std::string cost_map_file)
{
    std::cout << "Reading cost_map " << cost_map_file << std::endl;
    std::vector<std::vector<std::vector<double>>> cost_map_cube;
    std::string line;
    std::ifstream e_file(cost_map_file.c_str(), std::ios::in);
    std::vector<std::vector<double>> matrix;
    std::vector<double> row;

    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            int i = 0;
            while (std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                i++;
                if (i >= 100)
                {
                    i = 0;
                    matrix.push_back(row);
                    row.clear();
                }
            }
            cost_map_cube.push_back(matrix);
            matrix.clear();
        }
        e_file.close();

        std::cout << "Cost map of " << cost_map_cube.size() << " x " << cost_map_cube[0].size() << " x "
                  << cost_map_cube[0][0].size() << " loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the cost_map file" << std::endl;
        return cost_map_cube;
    }
    return cost_map_cube;
}

TEST(ArmPlannerTests, planningEEPath)
{
    FastMarching_lib::BiFastMarching dummyFM;

    std::vector<std::vector<double>> *costMap = new std::vector<std::vector<double>>;
    std::string costMapFile = "test/unit/data/dummyCostMap.txt";
    (*costMap) = readMatrixFile(costMapFile);
    for (int i = 0; i < costMap->size(); i++)
        for (int j = 0; j < (*costMap)[0].size(); j++)
            if (i == 0 || j == 0 || i == costMap->size() - 1 || j == (*costMap)[0].size() - 1)
                (*costMap)[i][j] = INFINITY;

    clock_t begin = clock();
    double mapResolution = 0.10;
    base::Waypoint roverPos, samplePos;

    roverPos.position[0] = 3;
    roverPos.position[1] = 0.5;
    roverPos.heading = 0;

    samplePos.position[0] = 6.51;
    samplePos.position[1] = 9.4;
    samplePos.position[2] = 1.2;
    samplePos.heading = 0;

    std::vector<base::Waypoint> *roverPath = new std::vector<base::Waypoint>;
    dummyFM.planPath(costMap, mapResolution, roverPos, samplePos, roverPath);
    roverPath->erase(roverPath->end() - 60, roverPath->end());

    std::vector<std::vector<double>> *DEM
        = new std::vector<std::vector<double>>(costMap->size(), std::vector<double>((*costMap)[0].size(), 1));
    double zResolution = 0.10;
    std::vector<std::vector<double>> *endEffectorPath = new std::vector<std::vector<double>>;
    std::vector<int> *pathsAssignment = new std::vector<int>;

    ArmPlanner_lib::ArmPlanner dummyArmPlanner;
    dummyArmPlanner.planEndEffectorPath(
        roverPath, DEM, mapResolution, zResolution, samplePos, endEffectorPath, pathsAssignment);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning: " << elapsed_secs << std::endl;

    // Printing results into .txt files
    std::ofstream pathFile;

    pathFile.open("test/unit/data/results/roverPath.txt");

    for (int j = 0; j < roverPath->size(); j++)
    {
        pathFile << (*roverPath)[j].position[0] << " " << (*roverPath)[j].position[1] << "\n";
    }

    pathFile.close();

    std::ofstream path3DFile;
    path3DFile.open("test/unit/data/results/EEPath.txt");

    for (int j = 0; j < endEffectorPath->size(); j++)
    {
        path3DFile << (*endEffectorPath)[j][0] << " " << (*endEffectorPath)[j][1] << " " << (*endEffectorPath)[j][2]
                   << "\n";
    }

    path3DFile.close();

    std::ofstream assignmentFile;
    assignmentFile.open("test/unit/data/results/assignment.txt");

    for (int j = 0; j < pathsAssignment->size(); j++)
    {
        assignmentFile << (*pathsAssignment)[j] << "\n";
    }

    assignmentFile.close();
}
}
