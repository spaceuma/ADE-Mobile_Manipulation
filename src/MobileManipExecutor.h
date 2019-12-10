#include "MotionPlan.h"
#include "MotionCommand.h"
#include "WaypointNavigation.hpp"

using namespace waypoint_navigation_lib;

class MobileManipExecutor {

private:
  MotionPlan* currentMotionPlan;
  double corridorWidth;
  WaypointNavigation pathTracker;
  std::vector<base::Waypoint*> pointerPath;

public:
  MobileManipExecutor();
  MobileManipExecutor(MotionPlan &currentMotionPlan);

  void updateMotionPlan(MotionPlan &newMotionPlan);

  bool isRoverWithinCorridor(Pose rover_pose);

  bool isArmColliding();

  bool isFinished();

  MotionCommand getRoverCommand(Pose rover_pose);

  Joints getArmCommand(Joints arm_joints);
};
