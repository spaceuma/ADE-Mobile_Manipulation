#include "MotionCommand.h"
#include "MotionPlan.h"
#include "WaypointNavigation.hpp"
#include "coupledControl.hpp"

using namespace waypoint_navigation_lib;
using namespace coupled_control;

class MobileManipExecutor
{

private:
    /**
     * Pointer to the current Motion Plan
     */
    MotionPlan *p_motion_plan;
    /**
     * Width of the path corridor 
     */
    double d_corridor_width;
    /**
     * Waypoint Navigation class 
     */
    WaypointNavigation waypoint_navigation;
    /**
     * Vector of pointers to each waypoint of the current path 
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
     * The segment currently followed by the rover 
     */
    int i_current_segment;
    /**
     * The arm motion profile
     */
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    /**
     * The next configuration to be reached by the arm 
     */
    std::vector<double> vd_next_arm_config;

public:
    /**
     * Class Constructor 
     */
    MobileManipExecutor();
    /**
     * Class Constructor using the current motion plan 
     */
    MobileManipExecutor(MotionPlan &motion_plan_m);

    /**
     * Updates the current motion plan with a new one 
     */
    void updateMotionPlan(MotionPlan &motion_plan_m);

    /**
     * Indicates whether the rover is within the corridor or not 
     */
    bool isRoverWithinCorridor(Pose pose_rover);

    /**
     * Indicates whether the arm is colliding with something or not 
     */
    bool isArmColliding();

    /**
     * Indicates whether the execution of the current operation is completed
     * or not
     */
    bool isFinished();

    /**
     * Returns the motion command corresponding to the current situation 
     */
    unsigned int getRoverCommand(Pose pose_rover, MotionCommand &mc_m);

    /**
     * Provides the next arm command according to the current situation 
     */
    void getArmCommand(Joints &j_next_arm_command);
};
