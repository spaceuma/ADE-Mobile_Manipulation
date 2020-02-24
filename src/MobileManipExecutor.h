#include "MotionCommand.h"
#include "MotionPlan.h"
#include "WaypointNavigation.hpp"
#include "coupledControl.hpp"

using namespace waypoint_navigation_lib;
using namespace coupled_control;

enum ArmExecutionState
{
    INITIALIZING = 0,        // 0
    READY,           // 1
    COUPLED_MOVING,     // 2
    SAMPLING_POS,  // 3
    RETRIEVING      // 4
};


class MobileManipExecutor
{

private:
    /**
     * Pointer to the present Motion Plan
     */
    MotionPlan *p_motion_plan;
    /**
     * Width of the path corridor 
     */
    double d_corridor_width;
    /**
     * Indicates if the arm is initialized or not 
     */
    bool b_ArmReady;
    /**
     * Waypoint Navigation class 
     */
    WaypointNavigation waypoint_navigation;
    /**
     * Vector of pointers to each waypoint of the present path 
     */
    std::vector<base::Waypoint *> vpw_path;
    /**
     * Motion Command class 
     */
    MotionCommand motion_command;
    /**
     * Coupled Control class 
     */
    coupledControl coupled_control;
    /**
     * The arm motion profile
     */
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    /**
     * The next configuration to be reached by the arm 
     */
    std::vector<double> vd_arm_present_readings;
    std::vector<double> vd_arm_present_command;
    /**
     * The previous arm state 
     */
    std::vector<double> vd_arm_previous_readings;
    std::vector<double> vd_arm_previous_command;
    /**
     * The present state of the Waypoint Navigation 
     */
    ArmExecutionState armstate;
    /**
     * The present state of the Waypoint Navigation 
     */
    NavigationState navstate;
    /**
     * Arm Variables Initialization 
     */
    void initializeArmVariables(const Joints &j_present_readings);
    /**
     * Returns a (0,0,0) rover command 
     */
    MotionCommand getZeroRoverCommand();
    /**
     * Provides the next arm command according to the present situation 
     */
    bool getArmCommand(Joints &j_next_arm_command);
    // Temporal fix for motion command bug
    void fixMotionCommand(MotionCommand &mc_m);
public:
    /**
     * Class Constructor using the present motion plan 
     */
    MobileManipExecutor(MotionPlan* presentMotionPlan, const Joints &j_present_readings);
    /**
     * Update the data extracted from the motion plan 
     */
    void updateMotionPlan();

    /**
     * Indicates whether the rover is within the corridor or not 
     */
    bool isRoverWithinCorridor(Pose pose_rover);

    /**
     * Indicates whether the arm is colliding with something or not 
     */
    bool isArmColliding();

    /**
     * Indicates whether the execution of the present operation is completed
     * or not
     */
    bool isRoverFinished();
    /**
     * Provides the next arm command according to the present situation 
     */
    unsigned int getCoupledCommand(Pose &rover_pose, const Joints &j_arm_present_readings_m, MotionCommand &mc_m, Joints &j_next_arm_command_m);
    /**
     * Checks if the arm is at Ready position 
     */
    bool isArmReady(const Joints &j_next_command, const Joints &j_present_joints);
    /**
     * Checks if the arm is still following the arm commands 
     */
    bool isArmWorking(const Joints &j_next_command, const Joints &j_present_joints);
    void getSamplingCommand(const Joints &j_arm_present_readings_m, Joints &j_next_arm_command_m);
    void getAtomicCommand();
};
