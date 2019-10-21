#include "MobileManipMotionPlanner.h"

MobileManipMotionPlanner::MobileManipMotionPlanner(/* Provided DEM using the same data struct as Airbus */RoverGuidance_Dem navCamDEM/* Provided DEM using the same data struct as Airbus */, RoverGuidance_Dem dem) {
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

MM_status MobileManipMotionPlanner::getStatus() {
	return this->status;
}

void MobileManipMotionPlanner::setStatus(int newStatus) {
	this->status = newStatus;
}

void MobileManipMotionPlanner::executeMotion(/* Coupled rover-manipulator motion plan to be followed. */MotionPlan readyMotionPlan) {
	// TODO - implement MobileManipMotionPlanner::executeMotion
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::updateNavCamDEM(/* DEM using the Airbus data struct */RoverGuidance_Dem navCamDEM) {
	// TODO - implement MobileManipMotionPlanner::updateNavCamDEM
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::generateMotionPlan(/* It should include the estimation error. */Pose rover_position, /* It should include the estimation error. */SamplePose sample, Joints arm_joints) {
	// TODO - implement MobileManipMotionPlanner::generateMotionPlan
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::executeAtomicOperation(ArmOperation arm_operation) {
	// TODO - implement MobileManipMotionPlanner::executeAtomicOperation
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::abort() {
	// TODO - implement MobileManipMotionPlanner::abort
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::updateRoverArmPos(/**
	 * Command to compute for the arm.
	 */
	Joints& arm_command, /**
	 * Command to compute for the rover base.
	 */
	MotionCommand& rover_command, /**
	 * Current pose of the rover base.
	 */
	Pose rover_position, /**
	 * Current position of the joints.
	 */
	Joints arm_joints) {
	// TODO - implement MobileManipMotionPlanner::updateRoverArmPos
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::updateLocCamDEM(/**
	 * DEM using Airbus data struct
	 */
	RoverGuidance_DEM locCamDEM, /**
	 * Current position of the base
	 */
	Pose rover_position, /**
	 * Current position of the joints
	 */
	Joints arm_joints) {
	// TODO - implement MobileManipMotionPlanner::updateLocCamDEM
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::updateSamplePos(/**
	 * Pose of the sample including error.
	 */
	SamplePose sample) {
	// TODO - implement MobileManipMotionPlanner::updateSamplePos
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::pause(/**
	 * By reference parameter to get commands to stop the joints.
	 */
	Joints& arm_command, /**
	 * By reference parameter to get commands to stop the rover base.
	 */
	MotionCommand& rover_command) {
	// TODO - implement MobileManipMotionPlanner::pause
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::resumeOperation() {
	// TODO - implement MobileManipMotionPlanner::resumeOperation
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::ack() {
	// TODO - implement MobileManipMotionPlanner::ack
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::resumeError() {
	// TODO - implement MobileManipMotionPlanner::resumeError
	throw "Not yet implemented";
}

MM_error MobileManipMotionPlanner::getErrorCode() {
	// TODO - implement MobileManipMotionPlanner::getErrorCode
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::start() {
	// TODO - implement MobileManipMotionPlanner::start
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::stopMotion(/**
	 * By reference parameter to get commands to stop the joints.
	 */
	Joints& arm_command, /**
	 * By reference parameter to get commands to stop the rover base.
	 */
	MotionCommand& rover_command) {
	// TODO - implement MobileManipMotionPlanner::stopMotion
	throw "Not yet implemented";
}
