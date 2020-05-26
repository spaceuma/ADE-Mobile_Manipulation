#include  "Joints.h"

#include "MotionCommand.h"
#include "MotionPlan.h"
#include "WaypointNavigation.hpp"
#include "coupledControl.hpp"
#include "CollisionDetector.h"

//using namespace proxy_library;
using namespace waypoint_navigation_lib;
using namespace coupled_control;

enum ArmExecutionState
{
    INITIALIZING = 0,        // 0
    FORBIDDEN_POS, //1
    READY,           // 2 
    COUPLED_MOVING,     // 3
    SAMPLING_POS,  // 4
    RETRIEVING      // 5
};


class MobileManipExecutor
{

private:
    /**
     * Adhoc variables to make arm retrieval
     */
    proxy_library::Joints j_first_retrieval_position;
    proxy_library::Joints j_second_retrieval_position;
    bool b_first_retrieval_point_reached;
    bool b_second_retrieval_point_reached;
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
    proxy_library::MotionCommand motion_command;
    /**
     * Coupled Control class 
     */
    coupledControl coupled_control;
    /**
     * Collision Detector class 
     */
    CollisionDetector* p_collision_detector;
    /**
     * The arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_arm_motion_profile;
    /**
     * The next configuration to be reached by the arm 
     */
    std::vector<double> vd_arm_present_readings;
    std::vector<double> vd_arm_present_command;
    std::vector<double> vd_arm_abs_speed;
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
    void initializeArmVariables(const proxy_library::Joints &j_present_readings);
    /**
     * Returns a (0,0,0) rover command 
     */
    proxy_library::MotionCommand getZeroRoverCommand();
    /**
     * Provides the next arm command according to the present situation 
     */
    bool getArmCommand(proxy_library::Joints &j_next_arm_command);
    // Temporal fix for motion command bug
    void fixMotionCommand(proxy_library::MotionCommand &mc_m);
public:
    /**
     * Class Constructor using the present motion plan 
     */
    MobileManipExecutor(MotionPlan* presentMotionPlan, const proxy_library::Joints &j_present_readings, std::string s_urdf_path_m);
    /**
     * Update the data extracted from the motion plan 
     */
    void updateMotionPlan();

    /**
     * Indicates whether the rover is within the corridor or not 
     */
    bool isRoverWithinCorridor(base::Pose pose_rover);

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
    unsigned int getCoupledCommand(base::Pose &rover_pose, const proxy_library::Joints &j_arm_present_readings_m, proxy_library::MotionCommand &mc_m, proxy_library::Joints &j_next_arm_command_m);
    /**
     * Checks if the arm is at Ready position 
     */
    bool isArmReady(const proxy_library::Joints &j_next_command, const proxy_library::Joints &j_present_joints);
    /**
     * Checks if the arm is currently colliding 
     */
    bool isArmColliding(const proxy_library::Joints &j_present_joints_m);
    /**
     * Checks if the arm is not in forbidden workspace 
     */
    bool isArmSafe(const proxy_library::Joints &j_present_joints_m);
    /**
     * Checks if the arm is still following the arm commands 
     */
    bool isArmWorking(const proxy_library::Joints &j_next_command, const proxy_library::Joints &j_present_joints);
    void getSamplingCommand(const proxy_library::Joints &j_arm_present_readings_m, proxy_library::Joints &j_next_arm_command_m);
    void getAtomicCommand();
    unsigned int getRetrievalCommand(const proxy_library::Joints &j_arm_present_readings_m, proxy_library::Joints &j_next_arm_command_m);
};
