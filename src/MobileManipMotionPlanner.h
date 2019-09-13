/**
 * It is the main class that receives all the information from other components, start the execution of the coupled rover-manipulator motion, etc.
 */
class MobileManipMotionPlanner {

private:
	/**
	 * Rover surronding map that would be used to generate the rover-manipulator trajectories.
	 */
	int currentMap;
	/**
	 * Object that run the execution of the planned trajectory.
	 */
	int MMExecution;
	/**
	 * Status of the motion planner:
	 * Idle
	 * Processing
	 * Running
	 * TBD
	 */
	int status;
	/**
	 * Attribute to indicate the error code
	 */
	int error;

public:
	/**
	 * Constructor, it receives a DEM and generates the Map object.
	 */
	MobileManipMotionPlanner(/* Provided DEM using the same data struct as Airbus */RoverGuidance_Dem dem);

	/**
	 * Run the execution of the motion
	 */
	int ExecuteMotion(/* Coupled rover-manipulator motion plan to be followed. */MotionPlan readyMotionPlan);

	/**
	 * It generates a motion plan based on the Map, the rover pose and the sample position.
	 */
	MotionPlan generateMotionPlan(Rover roverPose, Sample samplePos);

	/**
	 * It updates the stored map
	 */
	int updateMap(/* DEM using the Airbus data struct */RoverGuidance_Dem currentDEM);

	int getStatus();

	void setStatus(int newStatus);
};
