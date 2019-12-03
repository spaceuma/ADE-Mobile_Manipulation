#include "MM_status.h"
#include "MM_error.h"
#include "RoverGuidance_Dem.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "MobileManipMap.h"
#include "MobileManipExecutor.h"
#include "Waypoint.hpp"

/**
 * It is the main class that receives all the information from other components, start the execution of the coupled rover-manipulator motion, etc.
 */
class MobileManipMotionPlanner {

private:
	/**
	 * Rover surronding map that would be used to generate the rover-manipulator trajectories.
	 */
	MobileManipMap currentMap;
	/**
	 * The motion plan that is currently available.
	 */
	MotionPlan currentMotionPlan;
	/**
	 * Object that run the execution of the planned trajectory.
	 */
	MobileManipExecutor executor;
	/**
	 * Status of the motion planner:
	 * IDLE
	 * GENERATING_MOTION_PLAN
	 * READY_TO_MOVE
	 * EXECUTING_MOTION_PLAN
	 * EXECUTING_ARM_OPERATION
	 * RETRIEVING_ARM
	 * FINISHED
	 * ERROR
	 * REPLANNING
	 * PAUSE
	 */
	MM_status status;
	/**
	 * Attribute to indicate the error code
	 */
	MM_error error;
	/**
	 * The status prior to entering to the current one. This is useful for entering and exiting PAUSE status.
	 */
	MM_status priorStatus;
	/**
	 * A variable that indicates which kind of operation shall be performed with the arm (e.g. some atomic operation or sample handling)
	 */
	ArmOperation currentArmOperation;
	/**
	 * The pose of the sample. TODO: CHANGE ITS TYPE BY THE ONE USED BY MAGELLIUM!!
	 */
	Pose currentSamplePos;
	/**
	 * The current pose of the rover.
	 */
	Pose currentRoverPos;
	/**
	 * The current position of the arm joints.
	 */
	//Joints currentJointPositions;

public:
	/**
	 * Constructor, it receives a DEM and generates the Map object.
	 */
	MobileManipMotionPlanner(/* Provided DEM using the same data struct as Airbus */RoverGuidance_Dem navCamDEM);

	/**
	 * Run the execution of the motion
	 */
	void executeMotion(/* Coupled rover-manipulator motion plan to be followed. */MotionPlan readyMotionPlan);

	/**
	 * It updates the stored map
	 */
	void updateNavCamDEM(/* DEM using the Airbus data struct */RoverGuidance_Dem navCamDEM);

	/**
	 * It generates a motion plan based on the Map, the rover pose and the sample position.
	 */
	void generateMotionPlan(/* It should include the estimation error. */base::Waypoint rover_position, /* It should include the estimation error. */base::Waypoint sample, Joints arm_joints);

	/**
	 * It returns the status in which the software is.
	 */
	MM_status getStatus();

	/**
	 * It serves to perform an operation with only the arm.
	 */
	void executeAtomicOperation(ArmOperation arm_operation);

	/**
	 * It makes the software finish immediately.
	 */
	void abort();

	/**
	 * It provides commands depending on the current position of the rover and the arm joints
	 */
	void updateRoverArmPos(/**
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
	Joints arm_joints);

	/**
	 * It procceses the input LocCamDEM and triggers a replanning if necessary.
	 */
	void updateLocCamDEM(/**
	 * DEM using Airbus data struct
	 */
	RoverGuidance_Dem locCamDEM, /**
	 * Current position of the base
	 */
	Pose rover_position, /**
	 * Current position of the joints
	 */
	Joints arm_joints);

	/**
	 * Goal is updated during the execution of the motion plan. It requires to recalculate the motion plan taking into consideration the previously received DEM.
	 */
	void updateSamplePos(/**
	 * Pose of the sample including error.
	 */
	Pose sample);

	/**
	 * It makes the software enter into the PAUSE state, first creating commands to stop the rover base and arm.
	 */
	void pause(/**
	 * By reference parameter to get commands to stop the joints.
	 */
	Joints& arm_command, /**
	 * By reference parameter to get commands to stop the rover base.
	 */
	MotionCommand& rover_command);

	/**
	 * It returns to the state indicated by priorStatus. Useful to exit the PAUSE state.
	 */
	void resumeOperation();

	/**
	 * It serves to acknowledge by the user that the operation is completely finished.
	 */
	void ack();

	/**
	 * It handles the error and behaves according to its type.
	 */
	void resumeError();

	/**
	 * Returns the indication of which error affects the software.
	 */
	MM_error getErrorCode();

	/**
	 * Serves to actively start moving the rover and arm once a motion plan is available.
	 */
	void start();

	/**
	 * Provides commands to completely stop the rover.
	 */
	void stopMotion(/**
	 * By reference parameter to get commands to stop the joints.
	 */
	Joints& arm_command, /**
	 * By reference parameter to get commands to stop the rover base.
	 */
	MotionCommand& rover_command);
};
