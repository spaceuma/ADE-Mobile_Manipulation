#include "FastMarching.h"
#include "MobileManipMap.h"
#include "MotionPlan.h"
#include "readMatrixFile.h"
#include <ctime>
#include <fstream>
#include <gtest/gtest.h>
// This comes next
#include "ArmPlanner.h"
#include "FetchingPoseEstimator.h"

using namespace FastMarching_lib;

TEST(MMMotionPlanTest, nominal_working_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_costMap;
    std::vector<std::vector<double>> vvd_elevationMap;
    readMatrixFile("test/unit/data/input/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevationMap);
    readMatrixFile("test/unit/data/input/costMap.txt", vvd_costMap);
    double res = 0.1; // meters
    MobileManipMap dummyMap(vvd_elevationMap, vvd_costMap, res);

    // Creating the Motion Plan
    MotionPlan mplan_dummy;

    base::Waypoint roverPos, samplePos;
    roverPos.position[0] = 3.0;
    roverPos.position[1] = 2.0;
    roverPos.heading = 0;

    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    samplePos.heading = 0;

    // 2d Rover Base Path Planning
    clock_t ini2D = clock();
    ASSERT_NO_THROW(mplan_dummy.executeRoverBasePathPlanning(
        &dummyMap, roverPos, samplePos));
    std::vector<base::Waypoint> *roverPath = mplan_dummy.getPath();
    ASSERT_EQ(roverPath->size(), 110);
    int numWaypoints = mplan_dummy.shortenPathForFetching();
    ASSERT_EQ(roverPath->size(), 90);
    double t = double(clock() - ini2D) / CLOCKS_PER_SEC;
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: " << t << " s\033[0m" << std::endl;

    // Exporting the Path into a txt file
    std::ofstream pathFile;
    pathFile.open("test/unit/data/results/path.txt");
    for (int j = 0; j < roverPath->size(); j++)
    {
        pathFile << roverPath->at(j).position[0] << " "
                 << roverPath->at(j).position[1] << " "
                 << roverPath->at(j).heading << "\n";
    }
    pathFile.close();

    // 3d End Effector Motion Planning
    double zRes = 0.08;
    mplan_dummy.executeEndEffectorPlanning(&dummyMap, zRes);

    // Exporting the Arm Motion Profile into a txt file
    std::vector<std::vector<double>> *pvvd_arm_motion_profile
        = mplan_dummy.getArmMotionProfile();
    std::ofstream f_arm_motion;
    f_arm_motion.open("test/unit/data/results/armMotionProfile.txt");
    for (int j = 0; j < pvvd_arm_motion_profile->size(); j++)
    {
        for (int i = 0; i < (*pvvd_arm_motion_profile)[0].size(); i++)
        {
            f_arm_motion << (*pvvd_arm_motion_profile)[j][i] << " ";
        }
        f_arm_motion << "\n";
    }
    f_arm_motion.close();
}

TEST(MMMotionPlanTest, rover_in_obstacle_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_costMap;
    std::vector<std::vector<double>> vvd_elevationMap;
    readMatrixFile("test/unit/data/input/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevationMap);
    readMatrixFile("test/unit/data/input/costMap.txt", vvd_costMap);
    double res = 0.1; // meters
    MobileManipMap dummyMap(vvd_elevationMap, vvd_costMap, res);

    // Creating the Motion Plan
    MotionPlan mplan_dummy;

    base::Waypoint roverPos, samplePos;
    roverPos.position[0] = 6.0;
    roverPos.position[1] = 2.0;
    roverPos.heading = 0;

    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    samplePos.heading = 0;

    unsigned int i_error_code = 0;

    i_error_code = mplan_dummy.executeRoverBasePathPlanning(
        &dummyMap, roverPos, samplePos);

    ASSERT_EQ(i_error_code,1);
}
