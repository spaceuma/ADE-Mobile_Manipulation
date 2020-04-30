#include "MMError.h"
#include "MMStatus.h"
#include "MobileManipExecutor.h"
#include "MobileManipMap.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "Pose.h"
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
     * The current position of the arm joints.
     */
    /**
     * Set the current status
     */
    void setStatus(MMStatus status_m);
    /**
    * Set an error code
    */
    void setError(MMError error_m);

    base::Waypoint w_current_rover_position;
    /**
     * Configurable parameter: Z resolution (in meters)
     */
    double d_zres = 0.08;
    /**
     * Configurable parameter: Avoidance Distance (in meters)
     */
    double d_avoid_dist = 1.0;
    /**
     * Configurable parameter: rover kinematic configuration (in meters)
     * (d0, a1, a2, c2, a3, d4, d6) TODO - Maybe these values could be taken from urdf model?
     */
    std::vector<double> vd_kin_conf = {0.5,0.225,0.735,0.03,0.03,0.695,0.3};

    /**
     * Z-distance from base reference frame to ground
     */
    double d_base_height = 0.645;

    /**
     * Z-distance margin from final end effector position and ground
     */
    double d_finalEE_height = 0.4;

    /**
     * Configurable parameter: Max arm fetching distance (in meters)
     */
    double d_maxfetching_dist = vd_kin_conf[1] 
          + sqrt(pow(vd_kin_conf[2] + vd_kin_conf[5], 2)
                 - pow(d_base_height + vd_kin_conf[0] - vd_kin_conf[6] - d_finalEE_height, 2));
    double d_minfetching_dist = (d_maxfetching_dist<0.24)?0.0:(d_maxfetching_dist-0.24);


public:
    /**
     * Constructor, it receives a DEM and generates the Map object.
     */
    MobileManipMotionPlanner(const RoverGuidance_Dem &navCamDEM,
                             const Joints &j_present_readings,
			     std::string s_urdf_path_m);

    /**
     * It generates a motion plan based on the Map, the rover pose and the
     * sample position.
     */
    bool generateMotionPlan(proxy_library::Pose plpose_m,
                            double d_sample_pos_x,
                            double d_sample_pos_y);

    /**
     * Serves to actively start moving the rover and arm once a motion plan is
     * available.
     */
    bool start();

    /**
     * It makes the software finish immediately.
     */
    bool abort();

    /**
     * It makes the software enter into the PAUSE state, first creating commands
     * to stop the rover base.
     */
    bool pause(MotionCommand &rover_command);

    /**
     * It returns to the state indicated by priorStatus. Useful to exit the
     * PAUSE state.
     */
    bool resumeOperation();

    /**
     * It provides commands depending on the current position of the rover and
     * the arm joints
     */
    bool updateRoverArmPos(Joints &arm_command,
                           MotionCommand &rover_command,
                           proxy_library::Pose rover_position,
                           Joints arm_joints);

    /**
     * Goal is updated during the execution of the motion plan. It requires to
     * recalculate the motion plan taking into consideration the previously
     * received DEM.
     */
    void updateSamplePos(proxy_library::Pose sample);

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
     * Sets the value of Z-resolution in meters.
     */
    bool setZres(double d_zres_m);

    /**
     * Sets the value of avoidance distance in meters.
     */
    bool setAvoidanceDistance(double d_avoid_dist_m);
    
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

    /**
     * Shows current values of different config variables
     */
    void printConfig();

    /**
     * Returns if the class is in status ERROR.
     */
    bool isStatusError();
    /**
     * Returns the current value of the rover yaw.
     */
    double getCurrentRoverYaw();
    /**
     * A pointer to the current end effector path is returned
     */
    std::vector<std::vector<double>> *getWristPath();
    /**
     * A pointer to the current rover path is returned
     */
    std::vector<base::Waypoint> *getRoverPath();
    std::vector<std::vector<double>> *getCostMap();
    /**
     * A pointer to the arm motion profile is returned
     */
    std::vector<std::vector<double>> *getArmMotionProfile();
    /**
     * A pointer to the 3d cost map is returned
     */
    std::vector<std::vector<std::vector<double>>> *get3DCostMap();
    
    bool updateNavCamDEM(const RoverGuidance_Dem &navCamDEM);
    
    /**
     * It procceses the input LocCamDEM and triggers a replanning if necessary.
     */
    void updateLocCamDEM(RoverGuidance_Dem locCamDEM,
                         proxy_library::Pose rover_position,
                         Joints arm_joints);

    /**
     * It serves to perform an operation with only the arm.
     */
    void executeAtomicOperation();

};
