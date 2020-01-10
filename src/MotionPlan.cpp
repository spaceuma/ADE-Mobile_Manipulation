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
	//inputMap->getSamplingCostMap( costMap, sample );
	inputMap->getCostMap( costMap );
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
    this->vvd_arm_motion_profile.clear();
    std::vector<std::vector<double>> elevationMap;
    inputMap->getElevationMap(elevationMap);
    /*for ( uint j = 0; j < elevationMap.size(); j++ ){
            for ( uint i = 0; i < elevationMap[0].size(); i++){
	        std::cout << " Cost map at " << i << ", " << j << " is " << elevationMap[j][i] << std::endl;
	    }
    }*/
    /*for (uint i = 0; i < this->roverPath.size(); i++)
    {
	std::cout << "Waypoint " << i << " is ( " << this->roverPath[i].position[0] << ", " << this->roverPath[i].position[1] << " )" << std::endl;
    }*/
   std::cout << " The resolution is " << inputMap->getResolution() << std::endl;
   std::cout << "The sample position is " << this->samplePos.position[0] << "," << this->samplePos.position[1] << std::endl;
   this->armPlanner.planArmMotion(
        &(this->roverPath), &elevationMap, inputMap->getResolution(), zResolution, this->samplePos, &(this->vvd_arm_motion_profile));


}

std::vector<base::Waypoint>* MotionPlan::getPath(){
	return &(this->roverPath);
}

std::vector<std::vector<double>>* MotionPlan::getArmMotionProfile(){
	return &(this->vvd_arm_motion_profile);
}

void MotionPlan::setArmMotionProfile(std::vector<std::vector<double>> &m_arm_motion_profile){
	std::vector<double> row;
	std::cout << "Setting Arm Motion Profile of " << m_arm_motion_profile[0].size() << " joints and " << m_arm_motion_profile.size() << " samples" << std::endl;
	this->vvd_arm_motion_profile.clear();
	for ( uint j = 0; j < m_arm_motion_profile.size(); j++ ){
		for ( uint i = 0; i < m_arm_motion_profile[0].size(); i++){
			row.push_back(m_arm_motion_profile[j][i]);
		}
		this->vvd_arm_motion_profile.push_back(row);
		row.clear();
	}
}

void MotionPlan::setPath(std::vector<Waypoint> &vw_path){
	this->roverPath.clear();
	std::cout << "Setting Path of " << vw_path.size() << " waypoints" << std::endl;
	for (uint i = 0; i < vw_path.size(); i++)
	{
		this->roverPath.push_back(vw_path[i]);
	}
        std::cout << "Done setting path" << std::endl;	
}

