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
    std::vector<std::vector<double>> vvd_cost_map_shadowing, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/RH1_Zone1_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/RH1_Zone1_costMap.txt",
                                   vvd_cost_map_shadowing));
    base::Waypoint w_rover_pos_01, samplePos;
    ASSERT_NO_THROW(w_rover_pos_01 = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_01.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos.txt")) << "Input Sample Waypoint file is missing";
    samplePos.position[0] = 9.8; 
    samplePos.position[1] = 5.1; 
    w_rover_pos_01.position[0] = 7.0;
    w_rover_pos_01.position[1] = 1.5;

    double res = 0.1; // meters
    double zRes = 0.08;
    unsigned int ui_error_code = 0;

    MobileManipMap mmmap_shadowing(vvd_elevation_map, vvd_cost_map_shadowing, res, samplePos, 1.0, 0.94);

    // Creating the Motion Plan
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    }
    MotionPlan mplan_shadowing(&mmmap_shadowing, s_urdf_path);


    // 1st Case with Shadowing 
    clock_t ini2D = clock();
    ASSERT_NO_THROW(ui_error_code = mplan_shadowing.computeRoverBasePathPlanning(
        w_rover_pos_01));
    ASSERT_EQ(ui_error_code, 0);
    mplan_shadowing.shortenPathForFetching();
    savePath(mplan_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_01.txt");
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    ui_error_code = mplan_shadowing.computeArmProfilePlanning();
    EXPECT_EQ(ui_error_code, 0);
    savePath(mplan_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_01.txt");
    saveProfile(mplan_shadowing.getArmMotionProfile(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_profile_01.txt");
    saveProfile(mplan_shadowing.getWristPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_eepath_01.txt");

}

TEST(MMMotionPlanTest, rover_closeto_sample_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map_shadowing, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_Nominal_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap.txt",
                                   vvd_cost_map_shadowing));
    base::Waypoint w_rover_pos_02, samplePos;
    ASSERT_NO_THROW(w_rover_pos_02 = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_02.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos.txt")) << "Input Sample Waypoint file is missing";
    double res = 0.1; // meters
    double zRes = 0.08;
    unsigned int ui_error_code = 0;

    MobileManipMap mmmap_shadowing(vvd_elevation_map, vvd_cost_map_shadowing, res, samplePos, 1.0, 0.94);
    // Creating the Motion Plan
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    }
    MotionPlan mplan_shadowing(&mmmap_shadowing, s_urdf_path);
    // 2nd Case with Shadowing
    clock_t ini2D = clock();
    ASSERT_NO_THROW(ui_error_code = mplan_shadowing.computeRoverBasePathPlanning(
        w_rover_pos_02));
    ASSERT_EQ(ui_error_code, 0);
    mplan_shadowing.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    ui_error_code = mplan_shadowing.computeArmProfilePlanning();
    EXPECT_EQ(ui_error_code, 0);
    saveProfile(mplan_shadowing.getArmMotionProfile(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_profile_02.txt");
    savePath(mplan_shadowing.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_02.txt");
    saveProfile(mplan_shadowing.getWristPath(), "test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_eepath_02.txt");
    saveVolume(mplan_shadowing.get3DCostMap(), "test/unit/data/results/MMMotionPlanTest/close_to_goal_3dmap.txt");
}

// DEPRECATED
/*
TEST(MMMotionPlanTest, colliding_profile_test)
{
 
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map_shadowing, vvd_cost_map_no_shadowing, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_smaller_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_noShadowing.txt",
                                   vvd_cost_map_no_shadowing));
    double res = 0.1; // meters
    double zRes = 0.08;
    unsigned int ui_error_code = 0;

    base::Waypoint w_rover_pos_01, samplePos;
    ASSERT_NO_THROW(w_rover_pos_01 = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_04.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos_03.txt")) << "Input Sample Waypoint file is missing";

    MobileManipMap mmmap_no_shadowing(vvd_elevation_map, vvd_cost_map_no_shadowing, res, samplePos, 1.0, 0.94);
    // Creating the Motion Plan
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    }
    MotionPlan mplan_no_shadowing(&mmmap_no_shadowing,zRes, s_urdf_path);
    clock_t ini2D = clock();
    ASSERT_NO_THROW(ui_error_code = mplan_no_shadowing.computeRoverBasePathPlanning(
        w_rover_pos_01));
    ASSERT_EQ(ui_error_code, 0);
    mplan_no_shadowing.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    ui_error_code = mplan_no_shadowing.computeArmProfilePlanning();
    ASSERT_EQ(ui_error_code, 1);

}*/

// TODO - Fix this part with new constructors
/*TEST(MMMotionPlanTest, rover_or_sample_poses_nonvalid_test)
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
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    }
    MotionPlan mplan_dummy(&dummyMap, zRes, s_urdf_path);

    base::Waypoint roverPos, samplePos;
    unsigned int i_error_code = 0;

    // Rover out of the map
    roverPos.position[0] = -6.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    i_error_code = mplan_dummy.computeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 1)
        << "\033[31m[----------]\033[0m Expected Error Code 1";

    // Rover within obstacle area
    roverPos.position[0] = 6.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    i_error_code = mplan_dummy.computeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 2)
        << "\033[31m[----------]\033[0m Expected Error Code 2";

    // Sample out of the map
    roverPos.position[0] = 3.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = -5.3;
    samplePos.position[1] = 5.6;
    i_error_code = mplan_dummy.computeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 3)
        << "\033[31m[----------]\033[0m Expected Error Code 3";

    // Sample within obstacle area
    roverPos.position[0] = 3.0;
    roverPos.position[1] = 2.0;
    samplePos.position[0] = 6.0;
    samplePos.position[1] = 2.0;
    i_error_code = mplan_dummy.computeRoverBasePathPlanning(
        roverPos, samplePos);
    ASSERT_EQ(i_error_code, 4)
        << "\033[31m[----------]\033[0m Expected Error Code 4";
}*/

TEST(MMMotionPlanTest, non_reachable_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_splitted_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_splittedMap.txt",
                                   vvd_cost_map));
    base::Waypoint w_rover_pos, samplePos;
    ASSERT_NO_THROW(w_rover_pos = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_03.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos_02.txt")) << "Input Sample Waypoint file is missing";

    double res = 0.1; // meters
    double zRes = 0.08;
    MobileManipMap mmmap(vvd_elevation_map, vvd_cost_map, res, samplePos, 1.0, 0.94);

    // Creating the Motion Plan
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    }
    MotionPlan mplan(&mmmap, s_urdf_path);


    // 1st Case: Without Shadowing
    clock_t ini2D = clock();
    unsigned int ui_error_code;
    ASSERT_NO_THROW(ui_error_code = mplan.computeRoverBasePathPlanning(
        w_rover_pos));
    ASSERT_EQ(ui_error_code, 4)
        << "\033[31m[----------]\033[0m Expected Error Code 5";
}

TEST(MMMotionPlanTest, nonsmooth_path_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_splitted_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_discontinuous.txt",
                                   vvd_cost_map));
    base::Waypoint w_rover_pos, samplePos;
    ASSERT_NO_THROW(w_rover_pos = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_05.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos_04.txt")) << "Input Sample Waypoint file is missing";

    double res = 0.1; // meters
    double zRes = 0.08;
    MobileManipMap mmmap(vvd_elevation_map, vvd_cost_map, res, samplePos, 1.0, 0.94);

    // Creating the Motion Plan
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    } 
    MotionPlan mplan(&mmmap, s_urdf_path);


    // 1st Case: Without Shadowing
    clock_t ini2D = clock();
    unsigned int ui_error_code;
    ASSERT_NO_THROW(ui_error_code = mplan.computeRoverBasePathPlanning(
        w_rover_pos));
    ASSERT_EQ(ui_error_code, 5)
        << "\033[31m[----------]\033[0m Expected Error Code 6";
}

TEST(MMMotionPlanTest, sample_farfromtunnel_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map, vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/ColmenarRocks_smaller_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/costMap_noShadowing.txt",
                                   vvd_cost_map));
    base::Waypoint w_rover_pos, samplePos;
    ASSERT_NO_THROW(w_rover_pos = getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_04.txt")) << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos_03.txt")) << "Input Sample Waypoint file is missing";

    double res = 0.1; // meters
    double zRes = 0.08;
    unsigned int ui_error_code = 0;

    MobileManipMap mmmap(vvd_elevation_map, vvd_cost_map, res, samplePos, 1.0, 0.94);

    // Creating the Motion Plan
    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    } 
    MotionPlan mplan(&mmmap, s_urdf_path);


    // 1st Case: Without Shadowing
    clock_t ini2D = clock();
    ASSERT_NO_THROW(ui_error_code = mplan.computeRoverBasePathPlanning(
        w_rover_pos));
    ASSERT_EQ(ui_error_code, 0);
    mplan.shortenPathForFetching();
    std::cout << "\033[32m[----------]\033[0m 2D path planning execution time: "
              << double(clock() - ini2D) / CLOCKS_PER_SEC << " s\033[0m" << std::endl;
    savePath(mplan.getRoverPath(), "test/unit/data/results/MMMotionPlanTest/sample_outoftunnel_path_01.txt");
    ASSERT_NO_THROW(ui_error_code = mplan.computeArmProfilePlanning());
    ASSERT_EQ(ui_error_code, 2);
}


