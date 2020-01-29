#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;

MobileManipMotionPlanner::MobileManipMotionPlanner(/* Provided DEM using the same data struct as Airbus */const RoverGuidance_Dem &navCamDEM) {
	cout << "MMPLANNER: Creating MMMP" << endl;
	this->status = IDLE;
	this->error = NO_ERROR;
	// DEM is introduced into the map class
	this->p_mmmap = new MobileManipMap(navCamDEM);
	// Each class contains a pointer to the previous one
	this->p_motionplan = new MotionPlan(this->p_mmmap,0.08);// Maybe enter here Z resolution??
	this->p_mmexecutor = new MobileManipExecutor(this->p_motionplan);
}

bool MobileManipMotionPlanner::generateMotionPlan(const base::Waypoint &rover_position, const base::Waypoint &sample_position) {
	if (getStatus() == IDLE)
	{
            unsigned int ui_code = 0;
	  // TODO - Since for now there is no computation, the state will go to READY_TO_MOVE
	  setStatus(GENERATING_MOTION_PLAN);
	  this->p_mmmap->computeFACE(sample_position);
	  ui_code = this->p_motionplan->executeRoverBasePathPlanning(rover_position, sample_position);
          switch(ui_code)
	  {
              case 0:
                  break;
	      case 1:
		  setError(OOB_ROVER_POS);
		  return false;
	      case 2:
		  setError(OBS_ROVER_POS);
		  return false;
	      case 3:
		  setError(OOB_GOAL_POS);
		  return false;
	      case 4:
		  setError(OBS_GOAL_POS);
		  return false;
	  }
	  if(!(this->p_motionplan->shortenPathForFetching()))
	  {
              setError(GOAL_TOO_CLOSE);
	      return false;
	  }
	  printRoverPathInfo();
	  // TODO - Deal with EndEffectorPlanning errors
          this->p_motionplan->executeEndEffectorPlanning();
	  setStatus(READY_TO_MOVE);
	  return true;
	}
	else
	{
		setError(IMPROPER_CALL);
		return false;
	}
}

void MobileManipMotionPlanner::setError(MMError error_m) {
    if(error_m != NO_ERROR)
    {
        setStatus(ERROR);
    }
    this->error = error_m;
}

void MobileManipMotionPlanner::setStatus(MMStatus status_m) {
	this->priorStatus = getStatus();
	this->status = status_m;
}

MMStatus MobileManipMotionPlanner::getStatus() {
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
	switch(getStatus())
	{
            case IDLE:
		    break;
            case GENERATING_MOTION_PLAN:
		    break;

	}
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
    switch(this->error)
    {
	    case NO_ERROR:
		    break;
	    case POOR_DEM:
		    break;
	    case POOR_CONFIG:
		    break;
	    case OOB_ROVER_POS:
		    break;
	    case OOB_GOAL_POS:
		    break;
	    case OBS_ROVER_POS:
		    break;
	    case OBS_GOAL_POS:
		    break;
	    case UNREACH_GOAL:
		    break;
	    case UNCERT_GOAL:
		    break;
	    case NON_RESP_ARM:
		    break;
	    case COLLIDING_ARM:
		    break;
	    case NON_RESP_ROVER:
		    break;
	    case EXCESSIVE_DRIFT:
		    break;
	    case UNCERT_HEADING:
		    break;
	    case GOAL_TOO_CLOSE:
		    break;
	    case IMPROPER_CALL:
                    setStatus(priorStatus);
		    setError(NO_ERROR);
		    break;
    }
}

void MobileManipMotionPlanner::printRoverPathInfo()
{
    std::cout << " The Rover Path has " << this->p_motionplan->getNumberWaypoints() << " waypoints" << std::endl; 
}

void MobileManipMotionPlanner::printStatus()
{
    std::cout << "Current Status: ";
    switch(this->status)
    {
	    case IDLE:
		    std::cout << "IDLE";
		    break;
	    case GENERATING_MOTION_PLAN:
		    std::cout << "GENERATING_MOTION_PLAN";
		    break;
	    case READY_TO_MOVE:
		    std::cout << "READY_TO_MOVE";
		    break;
	    case EXECUTING_MOTION_PLAN:
		    std::cout << "EXECUTING_MOTION_PLAN";
		    break;
	    case EXECUTING_ARM_OPERATION:
		    std::cout << "EXECUTING_ARM_OPERATION";
		    break;
	    case RETRIEVING_ARM:
		    std::cout << "RETRIEVING_ARM";
		    break;
	    case FINISHED:
		    std::cout << "FINISHED";
		    break;
	    case ERROR:
		    std::cout << "ERROR";
		    break;
	    case REPLANNING:
		    std::cout << "REPLANNING";
		    break;
	    case PAUSE:
		    std::cout << "PAUSE";
		    break;
    }
    std::cout << std::endl;
}

void MobileManipMotionPlanner::printErrorCode()
{
    std::cout << " Current Error Code: ";
    switch(this->error)
    {
	    case NO_ERROR:
		    std::cout << "NO_ERROR";
		    break;
	    case POOR_DEM:
		    std::cout << "POOR_DEM";
		    break;
	    case POOR_CONFIG:
		    std::cout << "POOR_CONFIG";
		    break;
	    case OOB_ROVER_POS:
		    std::cout << "OOB_ROVER_POS";
		    break;
	    case OOB_GOAL_POS:
		    std::cout << "OOB_GOAL_POS";
		    break;
	    case OBS_ROVER_POS:
		    std::cout << "OBS_ROVER_POS";
		    break;
	    case OBS_GOAL_POS:
		    std::cout << "OBS_GOAL_POS";
		    break;
	    case UNREACH_GOAL:
		    std::cout << "UNREACH_GOAL";
		    break;
	    case UNCERT_GOAL:
		    std::cout << "UNCERT_GOAL";
		    break;
	    case NON_RESP_ARM:
		    std::cout << "NON_RESP_ARM";
		    break;
	    case COLLIDING_ARM:
		    std::cout << "COLLIDING_ARM";
		    break;
	    case NON_RESP_ROVER:
		    std::cout << "NON_RESP_ROVER";
		    break;
	    case EXCESSIVE_DRIFT:
		    std::cout << "EXCESSIVE_DRIFT";
		    break;
	    case UNCERT_HEADING:
		    std::cout << "UNCERT_HEADING";
		    break;
	    case GOAL_TOO_CLOSE:
		    std::cout << "GOAL_TOO_CLOSE";
		    break;
	    case IMPROPER_CALL:
		    std::cout << "IMPROPER_CALL";
		    break;
    }
    std::cout << std::endl;
}

MMError MobileManipMotionPlanner::getErrorCode() {
	return this->error;
}

void MobileManipMotionPlanner::start() {
	if (getStatus() == READY_TO_MOVE)
	{
		setStatus(EXECUTING_MOTION_PLAN);
	}
	else
	{
		setError(IMPROPER_CALL);
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
