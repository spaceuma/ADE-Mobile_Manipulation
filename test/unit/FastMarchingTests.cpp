#include "FastMarching.h"
#include "Waypoint.hpp"
#include "WaypointNavigation.hpp"
#include <ctime>
#include <fstream>
#include <gtest/gtest.h>
#include <math.h>

using namespace FastMarching_lib;

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

TEST(FastMarchingTests, computingTMap)
{
    BiFastMarching dummyFM;

    std::vector<std::vector<double>> *costMap = new std::vector<std::vector<double>>;
    std::string costMapFile = "test/unit/data/dummyCostMap.txt";
    (*costMap) = readMatrixFile(costMapFile);
    for (int i = 0; i < costMap->size(); i++)
        for (int j = 0; j < (*costMap)[0].size(); j++)
            if (i == 0 || j == 0 || i == costMap->size() - 1 || j == (*costMap)[0].size() - 1)
                (*costMap)[i][j] = INFINITY;

    clock_t begin = clock();

    std::vector<int> goal;
    std::vector<int> start;

    goal.push_back(30);
    goal.push_back(5);

    start.push_back(65);
    start.push_back(94);

    std::vector<std::vector<double>> *TMapGoal = new std::vector<std::vector<double>>;
    std::vector<std::vector<double>> *TMapStart = new std::vector<std::vector<double>>;

    std::vector<int> *nodeJoin = new std::vector<int>;

    dummyFM.computeTMap(costMap, goal, start, TMapGoal, TMapStart, nodeJoin);

    std::cout << "Node join: [" << (*nodeJoin)[0] << ", " << (*nodeJoin)[1] << "]" << std::endl;
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time computeT: " << elapsed_secs << std::endl;

    std::ofstream TMapGoalFile;
    std::ofstream TMapStartFile;

    TMapGoalFile.open("test/unit/data/results/TMapGoal.txt");
    TMapStartFile.open("test/unit/data/results/TMapStart.txt");

    for (int j = 0; j < costMap->size(); j++)
    {
        for (int i = 0; i < (*costMap)[0].size(); i++)
        {
            TMapGoalFile << (*TMapGoal)[j][i] << " ";
            TMapStartFile << (*TMapStart)[j][i] << " ";
        }
        TMapGoalFile << "\n";
        TMapStartFile << "\n";
    }

    TMapGoalFile.close();
    TMapStartFile.close();

    // EXPECT_EQ(INFINITY,dummyFM.getEikonal(a,b,c));
}

TEST(FastMarchingTests, planningRoverPath)
{
    BiFastMarching dummyFM;

    std::vector<std::vector<double>> *costMap = new std::vector<std::vector<double>>;
    std::string costMapFile = "test/unit/data/dummyCostMap.txt";
    (*costMap) = readMatrixFile(costMapFile);
    for (int i = 0; i < costMap->size(); i++)
        for (int j = 0; j < (*costMap)[0].size(); j++)
            if (i == 0 || j == 0 || i == costMap->size() - 1 || j == (*costMap)[0].size() - 1)
                (*costMap)[i][j] = INFINITY;

    clock_t begin = clock();
    double mapResolution = 1;
    base::Waypoint roverPos, samplePos;

    roverPos.position[0] = 30;
    roverPos.position[1] = 5;
    roverPos.heading = 0;

    samplePos.position[0] = 65;
    samplePos.position[1] = 94;
    samplePos.heading = 0;

    std::vector<base::Waypoint> *roverPath = new std::vector<base::Waypoint>;

    dummyFM.planPath(costMap, mapResolution, roverPos, samplePos, roverPath);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning: " << elapsed_secs << std::endl;

    std::ofstream pathFile;

    pathFile.open("test/unit/data/results/path.txt");

    for (int j = 0; j < roverPath->size(); j++)
    {
        pathFile << (*roverPath)[j].position[0] << " " << (*roverPath)[j].position[1] << "\n";
    }

    pathFile.close();

}

TEST(FastMarchingTests, computingTMap3D)
{
    BiFastMarching3D dummyFM3D;

    std::vector<std::vector<std::vector<double>>> *costMap = new std::vector<std::vector<std::vector<double>>>;
    std::string costMapFile = "test/unit/data/dummyCostMap3D.txt";
    (*costMap) = readMatrixFile3D(costMapFile);

    for (int i = 0; i < costMap->size(); i++)
        for (int j = 0; j < (*costMap)[0].size(); j++)
            for (int k = 0; k < (*costMap)[0][0].size(); k++)
            {
                if (i == 0 || j == 0 || k == 0 || i == costMap->size() - 1 || j == (*costMap)[0].size() - 1
                    || k == (*costMap)[0][0].size() - 1)
                    (*costMap)[i][j][k] = INFINITY;
                if (i == 0 || j == 6 || k == 6 || i == 24 || j == 54 || k == 94) (*costMap)[i][j][k] = INFINITY;
            }

    clock_t begin = clock();

    std::vector<int> goal;
    std::vector<int> start;

    goal.push_back(10);
    goal.push_back(5);
    goal.push_back(10);

    start.push_back(50);
    start.push_back(15);
    start.push_back(90);

    std::vector<std::vector<std::vector<double>>> *TMapGoal = new std::vector<std::vector<std::vector<double>>>;
    std::vector<std::vector<std::vector<double>>> *TMapStart = new std::vector<std::vector<std::vector<double>>>;

    std::vector<int> *nodeJoin = new std::vector<int>;

    dummyFM3D.computeTMap(costMap, goal, start, TMapGoal, TMapStart, nodeJoin);

    std::cout << "Node join: [" << (*nodeJoin)[0] << ", " << (*nodeJoin)[1] << ", " << (*nodeJoin)[2] << "]"
              << std::endl;
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time computeT: " << elapsed_secs << std::endl;

    std::ofstream TMapGoalFile;
    std::ofstream TMapStartFile;

    TMapGoalFile.open("test/unit/data/results/TMapGoal3D.txt");
    TMapStartFile.open("test/unit/data/results/TMapStart3D.txt");

    for (int j = 0; j < costMap->size(); j++)
    {
        for (int i = 0; i < (*costMap)[0].size(); i++)
        {
            for (int k = 0; k < (*costMap)[0][0].size(); k++)
            {
                TMapGoalFile << (*TMapGoal)[j][i][k] << " ";
                TMapStartFile << (*TMapStart)[j][i][k] << " ";
            }
            // TMapGoalFile << ", ";
            // TMapStartFile << ", ";
        }
        TMapGoalFile << "\n";
        TMapStartFile << "\n";
    }

    TMapGoalFile.close();
    TMapStartFile.close();
}

TEST(FastMarchingTests, planningSimple3Dpath)
{
    BiFastMarching3D dummyFM3D;

    std::vector<std::vector<std::vector<double>>> *costMap = new std::vector<std::vector<std::vector<double>>>;
    std::string costMapFile = "test/unit/data/dummyCostMap3D.txt";
    (*costMap) = readMatrixFile3D(costMapFile);

    for (int i = 0; i < costMap->size(); i++)
        for (int j = 0; j < (*costMap)[0].size(); j++)
            for (int k = 0; k < (*costMap)[0][0].size(); k++)
            {
                if (i == 0 || j == 0 || k == 0 || i == costMap->size() - 1 || j == (*costMap)[0].size() - 1
                    || k == (*costMap)[0][0].size() - 1)
                    (*costMap)[i][j][k] = INFINITY;
                if (i == 0 || j == 6 || k == 6 || i == 24 || j == 54 || k == 94) (*costMap)[i][j][k] = INFINITY;
            }

    clock_t begin = clock();

    double mapResolution = 1;
    double zResolution = 1;
    base::Waypoint iniPos, samplePos;

    iniPos.position[0] = 10;
    iniPos.position[1] = 5;
    iniPos.position[2] = 10;

    samplePos.position[0] = 50;
    samplePos.position[1] = 15;
    samplePos.position[2] = 90;

    std::vector<base::Waypoint> *endEffectorPath = new std::vector<base::Waypoint>;

    dummyFM3D.planPath(costMap, mapResolution, zResolution, iniPos, samplePos, endEffectorPath);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning: " << elapsed_secs << std::endl;

    std::ofstream pathFile;

    pathFile.open("test/unit/data/results/3Dpath.txt");

    for (int j = 0; j < endEffectorPath->size(); j++)
    {
        pathFile << (*endEffectorPath)[j].position[0] << " " << (*endEffectorPath)[j].position[1] << " "
                 << (*endEffectorPath)[j].position[2] << "\n";
    }

    pathFile.close();
}
