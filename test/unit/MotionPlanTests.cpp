#include <gtest/gtest.h>
#include "MotionPlan.h"
#include "Waypoint.hpp"
#include "WaypointNavigation.hpp"

using namespace waypoint_navigation_lib;

TEST(MotionPlanTests, trajectorycontrol){
  MotionPlan dummyPlan;
  WaypointNavigation pathTracker;

  Waypoint initWaypoint;
  initWaypoint.position[0] = 0.0;
  initWaypoint.position[0] = 0.0;

  Waypoint endWaypoint;
  endWaypoint.position[0] = 0.0;
  endWaypoint.position[0] = 0.0;

  std::vector<Waypoint> dummyPath;

  dummyPath.push_back(initWaypoint);
  dummyPath.push_back(endWaypoint);


  Pose robotPose;
  pathTracker.setPose(robotPose);
  

}
