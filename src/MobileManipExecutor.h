#include "MotionPlan.h"
#include "MotionCommand.h"

class MobileManipExecutor {

private:
	MotionPlan currentMotionPlan;

public:
	MobileManipExecutor();

	bool isRoverWithinCorridor(Pose rover_pose);

	bool isArmColliding();

	MotionCommand getRoverCommand(Pose rover_pose);

	Joints getArmCommand(Joints arm_joints);
};
