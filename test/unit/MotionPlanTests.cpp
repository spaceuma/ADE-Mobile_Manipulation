#include "FastMarching.h"
#include "MobileManipMap.h"
#include "MotionPlan.h"
#include "mmFileManager.h"
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
    std::vector<std::vector<double>> vvd_cost_map_shadowing, vvd_cost_map_no_shadowing, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_smaller_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_noShadowing.txt",
                                   vvd_cost_map_no_shadowing));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_Shadowing.txt",
                                   vvd_cost_map_shadowing));
    double res = 0.1; // meters
    double zRes = 0.08;
    MobileManipMap mmmap_no_shadowing(vvd_elevation_map, vvd_cost_map_no_shadowing, res), mmmap_shadowing(vvd_elevation_map, vvd_cost_map_shadowing, res);

    // Creating the Motion Plan
    MotionPlan mplan_no_shadowing(&mmmap_no_shadowing,zRes), mplan_shadowing(&mmmap_shadowing, zRes);

    base::Waypoint w_rover_pos_01, w_rover_pos_02, samplePos;
    ASSERT_NO_THROW(w_rover_pos_01 = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_01.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(w_rover_pos_02 = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_02.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos.txt")) << "Input Sample Waypoint file is missing";


    // 1st Case: Without Shadowing
    clock_t ini2D = clock();
    ASSERT_NO_THROW(mplan_no_shadowing.executeRoverBasePathPlanning(
        w_rover_pos_01, samplePos));
    mplan_no_shadowing.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    mplan_no_shadowing.executeEndEffectorPlanning();
    saveProfile(mplan_no_shadowing.getArmMotionProfile(), "test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_profile_01.txt");
    savePath(mplan_no_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_path_01.txt");
    saveProfile(mplan_no_shadowing.getEndEffectorPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_eepath_01.txt");
    saveVolume(mplan_no_shadowing.get3DCostMap(), "test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_3dmap_01.txt");

    // 1st Case with Shadowing 
    ini2D = clock();
    ASSERT_NO_THROW(mplan_shadowing.executeRoverBasePathPlanning(
        w_rover_pos_01, samplePos));
    mplan_shadowing.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    mplan_shadowing.executeEndEffectorPlanning();
    saveProfile(mplan_shadowing.getArmMotionProfile(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_profile_01.txt");
    savePath(mplan_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_01.txt");
    saveProfile(mplan_shadowing.getEndEffectorPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_eepath_01.txt");

    // 2nd Case without Shadowing
    ini2D = clock();
    ASSERT_NO_THROW(mplan_no_shadowing.executeRoverBasePathPlanning(
        w_rover_pos_02, samplePos));
    mplan_no_shadowing.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    mplan_no_shadowing.executeEndEffectorPlanning();
    saveProfile(mplan_no_shadowing.getArmMotionProfile(), "test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_profile_02.txt");
    savePath(mplan_no_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_no_shadowing_path_02.txt");

    // 2nd Case with Shadowing
    ini2D = clock();
    ASSERT_NO_THROW(mplan_shadowing.executeRoverBasePathPlanning(
        w_rover_pos_02, samplePos));
    mplan_shadowing.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    mplan_shadowing.executeEndEffectorPlanning();
    saveProfile(mplan_shadowing.getArmMotionProfile(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_profile_02.txt");
    savePath(mplan_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_02.txt");
}

TEST(MMMotionPlanTest, rover_or_sample_poses_nonvalid_test)
{
    // Reading the elevation and cost maps
    std::vector<std::vector<double>> vvd_costMap;
    std::vector<std::vector<double>> vvd_elevationMap;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_smaller_10cmDEM.csv",
                       vvd_elevationMap))
        << "\033[31m[----------]\033[0m Input CSV elevation map is missing";
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_Shadowing.txt",
                                   vvd_costMap))
        << "\033[31m[----------]\033[0m "
           "test/unit/data/input/costMap_shadowing.txt is missing";
    double res = 0.1; // meters
    MobileManipMap dummyMap(vvd_elevationMap, vvd_costMap, res);

    // Creating the Motion Plan
    double zRes = 0.08;
    MotionPlan mplan_dummy(&dummyMap, zRes);

    base::Waypoint roverPos, samplePos;
    unsigned int i_error_code = 0;

    // Rover out of the map
    roverPos.position[0] = -6.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    i_error_code = mplan_dummy.executeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 1)
        << "\033[31m[----------]\033[0m Expected Error Code 1";

    // Rover within obstacle area
    roverPos.position[0] = 6.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    i_error_code = mplan_dummy.executeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 2)
        << "\033[31m[----------]\033[0m Expected Error Code 2";

    // Sample out of the map
    roverPos.position[0] = 3.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = -5.3;
    samplePos.position[1] = 5.6;
    i_error_code = mplan_dummy.executeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 3)
        << "\033[31m[----------]\033[0m Expected Error Code 3";

    // Sample within obstacle area
    roverPos.position[0] = 3.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = 6.0;
    samplePos.position[1] = 2.0;
    i_error_code = mplan_dummy.executeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 4)
        << "\033[31m[----------]\033[0m Expected Error Code 4";
}

TEST(MMMotionPlanTest, non_reachable_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_splitted_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_splittedMap.txt",
                                   vvd_cost_map));
    double res = 0.1; // meters
    double zRes = 0.08;
    MobileManipMap mmmap(vvd_elevation_map, vvd_cost_map, res);

    // Creating the Motion Plan
    MotionPlan mplan(&mmmap, zRes);

    base::Waypoint w_rover_pos, samplePos;
    ASSERT_NO_THROW(w_rover_pos = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_03.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos_02.txt")) << "Input Sample Waypoint file is missing";


    // 1st Case: Without Shadowing
    clock_t ini2D = clock();
    unsigned int ui_error_code;
    ASSERT_NO_THROW(ui_error_code = mplan.executeRoverBasePathPlanning(
        w_rover_pos, samplePos));
    ASSERT_EQ(ui_error_code, 5)
        << "\033[31m[----------]\033[0m Expected Error Code 5";
}
