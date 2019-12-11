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
        this->samplePos = sample;
}

int MotionPlan::shortenPathForFetching(){
        //FetchingPoseEstimator_lib::FetchingPoseEstimator dummyFetchPosePlanner;
        int endWaypoint = this->fetchPosePlanner.getFetchWaypointIndex(&(this->roverPath));
        std::cout << "MotionPlan: the end waypoint is " << endWaypoint << std::endl;
        this->roverPath.erase(this->roverPath.begin() + endWaypoint + 1, this->roverPath.end());
        return endWaypoint;
}

void MotionPlan::executeEndEffectorPlanning(MobileManipMap* inputMap, double zResolution){
    std::vector<std::vector<double>> *endEffectorPath = new std::vector<std::vector<double>>;
    std::vector<std::vector<double>> elevationMap;
    inputMap->getElevationMap(elevationMap);

    /*for (uint i = 0; i < this->roverPath.size(); i++)
    {
	std::cout << "Waypoint " << i << " is ( " << this->roverPath[i].position[0] << ", " << this->roverPath[i].position[1] << " )" << std::endl;
    }*/
   std::cout << "The sample position is " << this->samplePos.position[0] << "," << this->samplePos.position[1] << std::endl;
   this->armPlanner.planEndEffectorPath(
        &(this->roverPath), &elevationMap, inputMap->getResolution(), zResolution, this->samplePos, endEffectorPath, &pathsAssignment);


}

std::vector<base::Waypoint>* MotionPlan::getPath(){
	return &(this->roverPath);
}

std::vector<int> MotionPlan::getAssignment(){
	return pathsAssignment;	
}
