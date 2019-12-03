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

void MotionPlan::executeRoverBasePathPlanning(MobileManipMap* inputMap, base::Waypoint rover_position, base::Waypoint sample){
	std::vector<std::vector<double>> costMap;
	inputMap->getCostMap(costMap);
	this->fmPlanner.planPath(&costMap, inputMap->getResolution(), rover_position, sample, &(this->roverPath));

}

std::vector<base::Waypoint> MotionPlan::getPath(){
	return this->roverPath;
}
