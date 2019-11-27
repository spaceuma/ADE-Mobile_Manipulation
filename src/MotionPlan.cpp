#include "MotionPlan.h"

MotionPlan::MotionPlan()
{
    // TODO - implement MotionPlan::MotionPlan
}

void MotionPlan::updateMotionPlan(std::vector<Waypoint> newRoverPath, std::vector<Joints> newJointsProfile)
{
    // TODO - implement MotionPlan::updateMotionPlan
    this->roverPath = newRoverPath;
    this->jointsProfile = newJointsProfile;
}
