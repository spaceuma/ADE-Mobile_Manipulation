/**
 * It is the main class that receives all the information from other components, start the execution of the coupled rover-manipulator motion, etc.
 */
class MobileManipMotionPlanner {

private:
	/**
	 * Rover surronding map that would be used to generate the rover-manipulator trajectories.
	 */
	MobileManipMap currentMap;
	MotionPlan currentMotionPlan;
	/**
	 * Object that run the execution of the planned trajectory.
	 */
	MobileManipExecutor executor;
	/**
	 * Status of the motion planner:
	 * Idle
	 * Processing
	 * Running
	 * TBD
	 */
	MM_status status;
	/**
	 * Attribute to indicate the error code
	 */
	MM_error error;
	MM_status priorStatus;
	ArmOperation currentArmOperation;
	samplePose currentSamplePos;
	Pose currentRoverPos;
	Joints currentJointPositions;

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
	void generateMotionPlan(/* It should include the estimation error. */Pose rover_position, /* It should include the estimation error. */SamplePose sample, Joints arm_joints);

	int getStatus();

	void executeAtomicOperation(ArmOperation arm_operation);

	void abort();

	void updateRoverArmPos(Joints& arm_command, MotionCommand& rover_command, Pose rover_position, Joints arm_joints);

	void updateLocCamDEM(RoverGuidance_DEM locCamDEM, Pose rover_position, Joints arm_joints);

	/**
	 * Goal is updated during the execution of the motion plan. It requires to recalculate the motion plan taking into consideration the previously received DEM.
	 */
	void updateSamplePos(SamplePose sample);

	void pause(Joints& arm_command, MotionCommand& rover_command);

	void resumeOperation();

	void ack();

	void resumeError();

	void getErrorCode();

	void start();

	void stopMotion(Joints& arm_command, MotionCommand& rover_command);
};
