#ifndef __MOBILE_MANIP_MOTION_PLAN__
#define __MOBILE_MANIP_MOTION_PLAN__

#include "Joints.h"
#include "Waypoint.hpp"
#include "ArmOperation.h"
using namespace proxy_library;
using namespace base;

class MotionPlan {

private:
	/**
	 * sampleTime to be considered during the execution of the planner.
	 */
	float sampleTime;
	/**
	 * The path that must be followed by the rover, made up by 2d waypoints
	 */
	std::vector<Waypoint> roverPath;
	/**
	 * Profile of joint positions to be followed by the arm.
	 */
	std::vector<Joints> jointsProfile;


public:
  MotionPlan();

  void updateMotionPlan(std::vector<Waypoint> newRoverPath, std::vector<Joints> newJointsProfile);

};

#endif
