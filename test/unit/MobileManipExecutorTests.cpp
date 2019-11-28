#include <gtest/gtest.h>
#include "Waypoint.hpp"
#include "WaypointNavigation.hpp"
#include "MobileManipExecutor.h"

using namespace waypoint_navigation_lib;

TEST(MMExecutorTest, trajectorycontrol){
  MobileManipExecutor dummyExecutor;
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
