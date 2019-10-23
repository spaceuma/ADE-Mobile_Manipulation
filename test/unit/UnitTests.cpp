#include "MobileManipMotionPlanner.h"
#include "MM_status.h"
#include <gtest/gtest.h>

TEST(MMMPTest, StateChanges){
  RoverGuidance_Dem navCamDEM;
  MobileManipMotionPlanner dummyPlanner(navCamDEM);
  EXPECT_EQ(IDLE,dummyPlanner.getStatus());
  Pose rover_position;
  Pose sample;
  Joints arm_joints;
  dummyPlanner.generateMotionPlan(rover_position, sample, arm_joints);
  EXPECT_EQ(READY_TO_MOVE,dummyPlanner.getStatus());
  dummyPlanner.start();
  EXPECT_EQ(EXECUTING_MOTION_PLAN,dummyPlanner.getStatus());
  Joints arm_command;
  MotionCommand rover_command;
  dummyPlanner.updateRoverArmPos(arm_command,rover_command,rover_position, arm_joints);
  EXPECT_EQ(EXECUTING_ARM_OPERATION,dummyPlanner.getStatus());
  dummyPlanner.updateRoverArmPos(arm_command,rover_command,rover_position, arm_joints);
  EXPECT_EQ(RETRIEVING_ARM,dummyPlanner.getStatus());
  dummyPlanner.updateRoverArmPos(arm_command,rover_command,rover_position, arm_joints);
  EXPECT_EQ(FINISHED,dummyPlanner.getStatus());
  dummyPlanner.ack();
  EXPECT_EQ(IDLE,dummyPlanner.getStatus());
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
