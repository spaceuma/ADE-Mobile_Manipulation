#include "MotionPlan.h"
#include "MotionCommand.h"
#include "WaypointNavigation.hpp"

class MobileManipExecutor {

private:
  MotionPlan* currentMotionPlan;
  double corridorWidth;

public:
  MobileManipExecutor();

  void updateMotionPlan(MotionPlan* newMotionPlan);

  bool isRoverWithinCorridor(Pose rover_pose);

  bool isArmColliding();

  MotionCommand getRoverCommand(Pose rover_pose);

  Joints getArmCommand(Joints arm_joints);
};
