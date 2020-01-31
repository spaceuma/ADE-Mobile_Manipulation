#include "MMError.h"
#include "MMStatus.h"
#include "MobileManipExecutor.h"
#include "MobileManipMap.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "RoverGuidance_Dem.h"
#include "Waypoint.hpp"

/**
 * It is the main class that receives all the information from other components,
 * start the execution of the coupled rover-manipulator motion, etc.
 */
class MobileManipMotionPlanner
{

private:
    /**
     * Rover surronding map that would be used to generate the rover-manipulator
     * trajectories.
     */
    MobileManipMap *p_mmmap;
    /**
     * The motion plan that is currently available.
     */
    MotionPlan *p_motionplan;
    /**
     * Object that run the execution of the planned trajectory.
     */
    MobileManipExecutor *p_mmexecutor;
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
    MMStatus status;
    /**
     * Attribute to indicate the error code
     */
    MMError error;
    /**
     * The status prior to entering to the current one. This is useful for
     * entering and exiting PAUSE status.
     */
    MMStatus priorStatus;
    /**
     * A variable that indicates which kind of operation shall be performed with
     * the arm (e.g. some atomic operation or sample handling)
     */
    ArmOperation currentArmOperation;
    /**
     * The pose of the sample. TODO: CHANGE ITS TYPE BY THE ONE USED BY
     * MAGELLIUM!!
     */
    Pose currentSamplePos;
    /**
     * The current pose of the rover.
     */
    Pose currentRoverPos;
    /**
     * The current position of the arm joints.
     */
    // Joints currentJointPositions;
    /**
 * Set the current status
 */
    void setStatus(MMStatus status_m);
    /**
 * Set an error code
 */
    void setError(MMError error_m);

public:
    /**
     * Constructor, it receives a DEM and generates the Map object.
     */
    MobileManipMotionPlanner(const RoverGuidance_Dem &navCamDEM,
                             double d_zres_m);

    /**
     * It generates a motion plan based on the Map, the rover pose and the
     * sample position.
     */
    bool generateMotionPlan(const base::Waypoint &rover_position,
                            const base::Waypoint &sample_position);

    /**
     * It serves to perform an operation with only the arm.
     */
    void executeAtomicOperation(ArmOperation arm_operation);

    /**
     * Serves to actively start moving the rover and arm once a motion plan is
     * available.
     */
    void start();

    /**
     * It makes the software finish immediately.
     */
    void abort();

    /**
     * It makes the software enter into the PAUSE state, first creating commands
     * to stop the rover base and arm.
     */
    void pause(Joints &arm_command, MotionCommand &rover_command);

    /**
     * It returns to the state indicated by priorStatus. Useful to exit the
     * PAUSE state.
     */
    void resumeOperation();

    /**
     * It procceses the input LocCamDEM and triggers a replanning if necessary.
     */
    void updateLocCamDEM(RoverGuidance_Dem locCamDEM,
                         Pose rover_position,
                         Joints arm_joints);

    /**
     * It provides commands depending on the current position of the rover and
     * the arm joints
     */
    bool updateRoverArmPos(Joints &arm_command,
                           MotionCommand &rover_command,
                           Pose rover_position,
                           Joints arm_joints);

    /**
     * Goal is updated during the execution of the motion plan. It requires to
     * recalculate the motion plan taking into consideration the previously
     * received DEM.
     */
    void updateSamplePos(Pose sample);

    /**
     * It serves to acknowledge by the user that the operation is completely
     * finished.
     */
    bool ack();

    /**
     * It handles the error and behaves according to its type.
     */
    void resumeError();

    /**
     * Returns the indication of which error affects the software.
     */
    MMError getErrorCode();

    /**
     * It returns the status in which the software is.
     */
    MMStatus getStatus();

    /**
 * Prints information regarding the resulting path.
 */
    void printRoverPathInfo();

    /**
     * Indicates the current status
     */
    void printStatus();

    /**
     * Returns the indication of which error affects the software.
     */
    void printErrorCode();
};
