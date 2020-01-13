#include "ArmPlanner.h"
#include "FetchingPoseEstimator.h"
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

    double mapResolution = 0.10;
    base::Waypoint roverPos, samplePos;

    roverPos.position[0] = 2;
    roverPos.position[1] = 1;
    roverPos.heading = 0;

    samplePos.position[0] = 7.5;
    samplePos.position[1] = 9.4;
    samplePos.position[2] = 1.1;
    samplePos.heading = 0;

    std::vector<base::Waypoint> *roverPath = new std::vector<base::Waypoint>;

    clock_t ini2D = clock();
    dummyFM.planPath(costMap, mapResolution, roverPos, samplePos, roverPath);
    clock_t end2D = clock();
    double t = double(end2D - ini2D) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning 2D: " << t << std::endl;

    // Decide where to stop the rover to fetch optimally
    FetchingPoseEstimator_lib::FetchingPoseEstimator dummyFetchPosePlanner;
    int endWaypoint = dummyFetchPosePlanner.getFetchWaypointIndex(roverPath);
    roverPath->erase(roverPath->begin() + endWaypoint + 1, roverPath->end());

    std::vector<std::vector<double>> *DEM
        = new std::vector<std::vector<double>>(costMap->size(), std::vector<double>((*costMap)[0].size(), 1));
    double zResolution = 0.08;
    std::vector<std::vector<double>> *armJoints = new std::vector<std::vector<double>>;
    std::vector<int> *pathsAssignment = new std::vector<int>;

    clock_t begin = clock();
    ArmPlanner_lib::ArmPlanner dummyArmPlanner;
    dummyArmPlanner.planEndEffectorPath(
        roverPath, DEM, mapResolution, zResolution, samplePos, armJoints, pathsAssignment);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning 3D: " << elapsed_secs << std::endl;
}

