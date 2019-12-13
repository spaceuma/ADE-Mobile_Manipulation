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
    double res = 0.1; // meters
    double n_row, n_col;
    std::vector<std::vector<double>> vvd_costMap;
    std::vector<std::vector<double>> vector_elevationData;
    readMatrixFile("test/unit/data/input/ColmenarRocks_smaller_10cmDEM.csv", vector_elevationData); 
    readMatrixFile("test/unit/data/results/costMap.txt", vvd_costMap);
    MobileManipMap dummyMap;
    dummyMap.setElevationMap(vector_elevationData,res);
    dummyMap.setCostMap(vvd_costMap);

    MotionPlan dummyPlan;

    clock_t begin = clock();

    base::Waypoint roverPos, samplePos;
    roverPos.position[0] = 6.5;
    roverPos.position[1] = 6.5;
    roverPos.heading = 0;

    samplePos.position[0] = 4.0;
    samplePos.position[1] = 2.0;
    samplePos.heading = 0;

    std::cout << "MOTIONPLANTEST: initiating path planning" << std::endl;
    clock_t ini2D = clock();
    dummyPlan.executeRoverBasePathPlanning(&dummyMap, roverPos, samplePos);
    int numWaypoints = dummyPlan.shortenPathForFetching();
    double zRes = 0.08;
    dummyPlan.executeEndEffectorPlanning(&dummyMap, zRes);
    std::vector<base::Waypoint>* roverPath = dummyPlan.getPath();
    clock_t end2D = clock();
    double t = double(end2D - ini2D) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning 2D: " << t << std::endl;

    std::ofstream pathFile;

    pathFile.open("test/unit/data/results/path.txt");

    for (int j = 0; j < roverPath->size(); j++)
    {
        pathFile << roverPath->at(j).position[0] << " " << roverPath->at(j).position[1] << " " << roverPath->at(j).heading << "\n";
    }

    pathFile.close();
    std::cout << "The resulting path has " << roverPath->size() << " Waypoints" << std::endl;
    std::cout << "The waypoint number to erase is " << numWaypoints << std::endl;
    
    std::vector<std::vector<double>>* pvvd_arm_motion_profile = dummyPlan.getArmMotionProfile();
    std::ofstream f_arm_motion;

    f_arm_motion.open("test/unit/data/results/armMotionProfile.txt");

    for (int j = 0; j < pvvd_arm_motion_profile->size(); j++)
    {
        for (int i = 0; i < (* pvvd_arm_motion_profile)[0].size(); i++)
        {
            f_arm_motion << (* pvvd_arm_motion_profile)[j][i] << " ";
        }
        f_arm_motion << "\n";
    }

    f_arm_motion.close();

    std::vector<int> assignmentVector = dummyPlan.getAssignmentVector();
    std::ofstream assignmentFile;

    assignmentFile.open("test/unit/data/results/assignment.txt");

    /*for (int j = 0; j < assignmentVector.size(); j++)
    {
        assignmentFile << assignmentVector[j] << "\n";
    }*/

    pathFile.close();

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

