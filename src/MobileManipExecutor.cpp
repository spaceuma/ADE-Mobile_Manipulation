#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"

MobileManipExecutor::MobileManipExecutor() {
}

MobileManipExecutor::MobileManipExecutor(MotionPlan &currentMotionPlan) {
  
    std::vector<base::Waypoint>* roverPath = currentMotionPlan.getPath();
    pointerPath.clear();

    for (size_t i = 0; i < roverPath->size(); i++)
    {
        roverPath->at(i).tol_position = 0.1;
        pointerPath.push_back(&roverPath->at(i));
    }
    pathTracker.setTrajectory(pointerPath);
    pathTracker.setNavigationState(DRIVING);
}

void MobileManipExecutor::updateMotionPlan(MotionPlan &newMotionPlan)
{
  this->currentMotionPlan = &newMotionPlan;
}

bool MobileManipExecutor::isRoverWithinCorridor(Pose rover_pose) {
	// TODO - implement MobileManipExecutor::isRoverWithinCorridor
	throw "Not yet implemented";
}

bool MobileManipExecutor::isArmColliding() {
	// TODO - implement MobileManipExecutor::isArmColliding
	throw "Not yet implemented";
}

bool MobileManipExecutor::isFinished(){
    return pathTracker.getNavigationState() == TARGET_REACHED;
}

MotionCommand MobileManipExecutor::getRoverCommand(Pose rover_pose) {
	// TODO - implement MobileManipExecutor::getRoverCommand
        MotionCommand mc;
        pathTracker.setPose(rover_pose);
        pathTracker.update(mc);
	return mc;
}

Joints MobileManipExecutor::getArmCommand(Joints arm_joints) {
	// TODO - implement MobileManipExecutor::getArmCommand
	throw "Not yet implemented";
}
