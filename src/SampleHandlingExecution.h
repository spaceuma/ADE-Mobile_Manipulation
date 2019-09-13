/**
 * This classs would perform the last motion to do a task with the sample, e.g.:
 * To place the end effector in touch with the sample.
 * To pick up samples of soil, etc.
 */
class SampleHandlingExecution : MobileManipulatorExecution {

public:
	/**
	 * Sample position
	 */
	Sample SamplePos;

	SampleHandlingExecution(MobileManipMotionPlanner currentMMMP, Sample samplePos);
};
