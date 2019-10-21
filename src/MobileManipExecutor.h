class MobileManipExecutor {

private:
	MotionPlan currentMotionPlan;

public:
	MobileManipExecutor(MotionPlan motion_plan);

	boolean isRoverWithinCorridor(Pose rover_pose);

	boolean isArmColliding();

	MotionCommand getRoverCommand(Pose rover_pose);

	Joints getArmCommand(Joints arm_joints);
};
