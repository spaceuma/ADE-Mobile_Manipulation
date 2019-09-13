#include "MobileManipMotionPlanner.h"

MobileManipMotionPlanner::MobileManipMotionPlanner(/* Provided DEM using the same data struct as Airbus */RoverGuidance_Dem dem) {
	// TODO - implement MobileManipMotionPlanner::MobileManipMotionPlanner
	throw "Not yet implemented";
}

int MobileManipMotionPlanner::ExecuteMotion(/* Coupled rover-manipulator motion plan to be followed. */MotionPlan readyMotionPlan) {
	// TODO - implement MobileManipMotionPlanner::ExecuteMotion
	throw "Not yet implemented";
}

MotionPlan MobileManipMotionPlanner::generateMotionPlan(Rover roverPose, Sample samplePos) {
	// TODO - implement MobileManipMotionPlanner::generateMotionPlan
	throw "Not yet implemented";
}

int MobileManipMotionPlanner::updateMap(/* DEM using the Airbus data struct */RoverGuidance_Dem currentDEM) {
	// TODO - implement MobileManipMotionPlanner::updateMap
	throw "Not yet implemented";
}

int MobileManipMotionPlanner::getStatus() {
	return this->status;
}

void MobileManipMotionPlanner::setStatus(int newStatus) {
	this->status = newStatus;
}
