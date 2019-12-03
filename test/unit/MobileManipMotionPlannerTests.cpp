#include "MM_status.h"
#include "MobileManipMotionPlanner.h"
#include <gtest/gtest.h>
#include "readMatrixFile.h"

TEST(MMMPTest, NominalStateFlow)
{
    // Reading the DEM 
    MotionPlan dummyPlan;
    double res = 0.04; // meters
    double n_row, n_col;
    std::vector<double> vector_elevationData;
    readMatrixFile("test/unit/data/ColmenarRocks_smaller_4cmDEM.csv", res, n_row, n_col, vector_elevationData);
    std::cout << "MMMotionPlanTest: The size is " << vector_elevationData.size() << std::endl;
   
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

    // Introducing RG-DEM into planner
    MobileManipMotionPlanner dummyPlanner(dummyDem);

    base::Waypoint roverPos, samplePos;
    std::vector<base::Waypoint> *roverPath = new std::vector<base::Waypoint>;

    BiFastMarching dummyFM;
    clock_t begin = clock();
    base::Waypoint roverPos, samplePos;

    roverPos.position[0] = 1.0;
    roverPos.position[1] = 1.2;
    roverPos.heading = 0;

    samplePos.position[0] = 1.0;
    samplePos.position[1] = 6.4;
    samplePos.position[2] = 1.1;
    samplePos.heading = 0;

    EXPECT_EQ(IDLE, dummyPlanner.getStatus());
    Joints arm_joints;
    /*dummyPlanner.generateMotionPlan(roverPos, samplePos, arm_joints);
    EXPECT_EQ(READY_TO_MOVE, dummyPlanner.getStatus());
    dummyPlanner.start();
    EXPECT_EQ(EXECUTING_MOTION_PLAN, dummyPlanner.getStatus());
    Joints arm_command;
    MotionCommand rover_command;
    dummyPlanner.updateRoverArmPos(arm_command, rover_command, roverPos, arm_joints);
    EXPECT_EQ(EXECUTING_ARM_OPERATION, dummyPlanner.getStatus());
    dummyPlanner.updateRoverArmPos(arm_command, rover_command, roverPos, arm_joints);
    EXPECT_EQ(RETRIEVING_ARM, dummyPlanner.getStatus());
    dummyPlanner.updateRoverArmPos(arm_command, rover_command, roverPos, arm_joints);
    EXPECT_EQ(FINISHED, dummyPlanner.getStatus());
    dummyPlanner.ack();
    EXPECT_EQ(IDLE, dummyPlanner.getStatus());*/
}

TEST(MMMPTest, PauseAction)
{
    /*RoverGuidance_Dem navCamDEM;
    base::Waypoint roverPos, samplePos;
    MobileManipMotionPlanner dummyPlanner(navCamDEM);
    EXPECT_EQ(IDLE, dummyPlanner.getStatus());
    roverPos.position[0] = 1.0;
    roverPos.position[1] = 1.2;
    roverPos.heading = 0;

    samplePos.position[0] = 1.0;
    samplePos.position[1] = 6.4;
    samplePos.position[2] = 1.1;
    samplePos.heading = 0;

    Joints arm_joints;
    dummyPlanner.generateMotionPlan(roverPos, sample, arm_joints);
    EXPECT_EQ(READY_TO_MOVE, dummyPlanner.getStatus());
    dummyPlanner.start();
    EXPECT_EQ(EXECUTING_MOTION_PLAN, dummyPlanner.getStatus());
    Joints arm_command;
    MotionCommand rover_command;
    dummyPlanner.pause(arm_command, rover_command);
    EXPECT_EQ(PAUSE, dummyPlanner.getStatus());
    dummyPlanner.resumeOperation();
    EXPECT_EQ(EXECUTING_MOTION_PLAN, dummyPlanner.getStatus());*/
}
