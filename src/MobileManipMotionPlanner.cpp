#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;

MobileManipMotionPlanner::MobileManipMotionPlanner(/* Provided DEM using the same data struct as Airbus */RoverGuidance_Dem navCamDEM) {
	// TODO - implement MobileManipMotionPlanner::MobileManipMotionPlanner
	cout << "MMPLANNER: Creating MMMP" << endl;
	this->status = IDLE;
	this->error = NO_ERROR;
	this->currentMap.setRGDem(navCamDEM);	
}

void executeMotion(/* Coupled rover-manipulator motion plan to be followed. */MotionPlan readyMotionPlan) {
	// TODO - implement MobileManipMotionPlanner::executeMotion
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::generateMotionPlan(base::Waypoint rover_position, base::Waypoint sample_position, Joints arm_joints) {
	// TODO - implement MobileManipMotionPlanner::generateMotionPlan
        cout << "MMPLANNER: Generating Motion Plan" << endl;
	if (this->status == IDLE)
	{
	  // TODO - Since for now there is no computation, the state will go to READY_TO_MOVE
	  this->status = GENERATING_MOTION_PLAN;
	  this->currentMotionPlan.executeRoverBasePathPlanning(&(this->currentMap), rover_position, sample_position);
	  this->currentMotionPlan.shortenPathForFetching();
	  this->status = READY_TO_MOVE;
	  std::vector<base::Waypoint>* roverPath = this->currentMotionPlan.getPath();
	  std::cout << "MMPLANNER: The resulting path has " << roverPath->size() << " Waypoints" << std::endl;
          cout << "MMPLANNER: Ready to move" << endl;
	}
	else
	{
		cout << "MMPLANNER: generateMotionPlan() can only be called in IDLE state " << endl;
	}
}

MM_status MobileManipMotionPlanner::getStatus() {
	return this->status;
}

void MobileManipMotionPlanner::executeMotion(/* Coupled rover-manipulator motion plan to be followed. */MotionPlan readyMotionPlan) {
	// TODO - implement MobileManipMotionPlanner::executeMotion
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::updateNavCamDEM(/* DEM using the Airbus data struct */RoverGuidance_Dem navCamDEM) {
	// TODO - implement MobileManipMotionPlanner::updateNavCamDEM
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

void MobileManipMotionPlanner::updateRoverArmPos(/* Command to compute for the arm.*/
	Joints& arm_command, /* Command to compute for the rover base.*/
	MotionCommand& rover_command, /* Current pose of the rover base.*/
	Pose rover_position, /* Current position of the joints.*/
	Joints arm_joints)
{
	// TODO - implement MobileManipMotionPlanner::updateRoverArmPos
  if (this->status == RETRIEVING_ARM)
  {
    cout << "MMPLANNER: arm is retrieved" << endl;
    this->status = FINISHED;
    cout << "MMPLANNER: the operation is finished" << endl;
  }
  else
  {
    
  }
  if (this->status == EXECUTING_ARM_OPERATION)
  {
    cout << "MMPLANNER: finished arm operation" << endl;
    this->status = RETRIEVING_ARM;
    cout << "MMPLANNER: proceeding to retrieve the arm" << endl;
  }
  else
  {
    
  }
  if (this->status == EXECUTING_MOTION_PLAN)
  {
    cout << "MMPLANNER: reached goal state" << endl;
    this->status = EXECUTING_ARM_OPERATION;
    cout << "MMPLANNER: proceeding to execute arm operation" << endl;
  }
  else
  {
    
  }
}

void MobileManipMotionPlanner::updateLocCamDEM(/**
	 * DEM using Airbus data struct
	 */
	RoverGuidance_Dem locCamDEM, /**
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
	Pose sample) {
	// TODO - implement MobileManipMotionPlanner::updateSamplePos
	throw "Not yet implemented";
}

void MobileManipMotionPlanner::pause(/* By reference parameter to get commands to stop the joints.*/
  Joints& arm_command, 
  /* By reference parameter to get commands to stop the rover base.*/
  MotionCommand& rover_command)
{
  // TODO - implement MobileManipMotionPlanner::pause
  this->priorStatus = this->status;
  this->status = PAUSE;
  cout << "MMPLANNER: execution in pause" << endl;
}

void MobileManipMotionPlanner::resumeOperation() {
  // TODO - implement MobileManipMotionPlanner::pause
  this->status = this->priorStatus;
  this->priorStatus = PAUSE;
  cout << "MMPLANNER: resuming operation" << endl;
}

void MobileManipMotionPlanner::ack() {
	// TODO - implement MobileManipMotionPlanner::ack
  if (this->status == FINISHED)
  {
    this->status = IDLE;
    cout << "MMPLANNER: returned to IDLE state" << endl;
  }
  else
  {
    cout << "MMPLANNER: ack() can only be called in FINISHED state " << endl;
  }
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
	cout << "MMPLANNER: starting the execution of the motion plan" << endl;
	if (this->status == READY_TO_MOVE)
	{
		this->status = EXECUTING_MOTION_PLAN;
	}
	else
	{
		cout << "MMPLANNER: start() can only be called in READY_TO_MOVE state " << endl;
	}
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
