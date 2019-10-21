class MotionPlan {

private:
	/**
	 * sampleTime to be considered during the execution of the planner.
	 */
	float sampleTime;
	/**
	 * The path that must be followed by the rover, made up by 2d waypoints
	 */
	Waypoint roverPath[];
	/**
	 * Profile of joint positions to be followed by the arm.
	 */
	Joints jointsProfile[];

public:
	MotionPlan(ArmOperation arm_operation);

	void updateMotionPlan();
};
