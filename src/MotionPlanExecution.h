/**
 * This object will launch a Thread to execute the motion plan.
 */
class MotionPlanExecution : MobileManipulatorExecution {

public:
	/**
	 * Defined motion plan to be followed by the execution.
	 */
	MotionPlan currentMotionPlan;

	/**
	 * Constructor
	 */
	MotionPlanExecution(MotionPlan currentMotionPlan, MobileManipMotionPlanner currentMMMP);
};
