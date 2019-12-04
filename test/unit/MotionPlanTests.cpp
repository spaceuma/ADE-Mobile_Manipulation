#include "FastMarching.h"
#include "MobileManipMap.h"
#include "MotionPlan.h"
#include "readMatrixFile.h"
#include <ctime>
#include <fstream>
#include <gtest/gtest.h>
#include "ArmPlanner.h"
#include "FetchingPoseEstimator.h"

using namespace FastMarching_lib;

TEST(MMMotionPlanTest, roverbaseplanning)
{
    // Reading the DEM 
    double res = 0.04; // meters
    double n_row, n_col;
    std::vector<double> vector_elevationData;
    readMatrixFile("test/unit/data/ColmenarRocks_smaller_4cmDEM.csv", res, n_row, n_col, vector_elevationData);
   
    // Creating the Rover Guidance DEM 
    RoverGuidance_Dem dummyDem;
    double dummyArray[(int)n_row * (int)n_col];
    dummyDem.p_heightData_m = dummyArray;
    for (uint i = 0; i < vector_elevationData.size(); i++)
    {
        dummyDem.p_heightData_m[i] = vector_elevationData[i];
    }
    dummyDem.cols = n_col;
    dummyDem.rows = n_row;
    dummyDem.nodeSize_m = res;

    // Introducing RG-DEM into MMMap
    MobileManipMap dummyMap;
    dummyMap.setRGDem(dummyDem);
    base::Waypoint roverPos, samplePos;

    clock_t begin = clock();

    roverPos.position[0] = 1.0;
    roverPos.position[1] = 1.2;
    roverPos.heading = 0;

    samplePos.position[0] = 1.0;
    samplePos.position[1] = 6.4;
    samplePos.position[2] = 1.1;
    samplePos.heading = 0;


    MotionPlan dummyPlan;
    clock_t ini2D = clock();
    dummyPlan.executeRoverBasePathPlanning(&dummyMap, roverPos, samplePos);
    int numWaypoints = dummyPlan.shortenPathForFetching();
    std::vector<base::Waypoint> roverPath = dummyPlan.getPath();
    clock_t end2D = clock();
    double t = double(end2D - ini2D) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning 2D: " << t << std::endl;

    std::ofstream pathFile;

    pathFile.open("test/unit/data/results/path.txt");

    for (int j = 0; j < roverPath.size(); j++)
    {
        pathFile << roverPath[j].position[0] << " " << roverPath[j].position[1] << "\n";
    }

    pathFile.close();
    std::cout << "The resulting path has " << roverPath.size() << " Waypoints" << std::endl;
    std::cout << "The waypoint number to erase is " << numWaypoints << std::endl;
    double zRes = 0.1;
    dummyPlan.executeEndEffectorPlanning(&dummyMap, zRes);
    // Decide where to stop the rover to fetch optimally
/*    FetchingPoseEstimator_lib::FetchingPoseEstimator dummyFetchPosePlanner;
    int endWaypoint = dummyFetchPosePlanner.getFetchWaypointIndex(roverPath);
    roverPath->erase(roverPath->begin() + endWaypoint + 1, roverPath->end());

    std::vector<std::vector<double>> *DEM
        = new std::vector<std::vector<double>>(costMap.size(), std::vector<double>(costMap[0].size(), 1));
    double zResolution = 0.1;
    std::vector<std::vector<double>> *endEffectorPath = new std::vector<std::vector<double>>;
    std::vector<int> *pathsAssignment = new std::vector<int>;

    ArmPlanner_lib::ArmPlanner dummyArmPlanner;
    dummyArmPlanner.planEndEffectorPath(
        roverPath, DEM, dummyDem.nodeSize_m, zResolution, samplePos, endEffectorPath, pathsAssignment);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning 3D: " << elapsed_secs << std::endl;
    */
}

