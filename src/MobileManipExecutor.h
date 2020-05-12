#include "Joints.h"

#include "MotionCommand.h"
#include "MotionPlan.h"
#include "WaypointNavigation.hpp"
#include "coupledControl.hpp"
#include "CollisionDetector.h"

using namespace proxy_library;
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
    Joints j_first_retrieval_position;
    Joints j_second_retrieval_position;
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
     * Index to current segment 
     */
    int i_current_segment;
    /**
     * Index to initial segment 
     */
    int i_initial_segment;

    bool b_is_last_segment;
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
     * Collision Detector class 
     */
    CollisionDetector* p_collision_detector;
    /**
     * The arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_arm_motion_profile;
    /**
     * The sweeping arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_arm_sweeping_profile;
    /**
     * The initial arm motion times
     */
    std::vector<double> *pvd_arm_sweeping_times;
    /**
     * The initial arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_init_arm_profile;
    /**
     * The initial arm motion times
     */
    std::vector<double> *pvd_init_time_profile;
    /**
     * The next configuration to be reached by the arm 
     */
    std::vector<double> vd_arm_present_readings;
    std::vector<double> vd_arm_present_command;
    std::vector<double> vd_arm_abs_speed;
    /**
     * The previous arm state 
     */
    std::vector<double> vd_arm_previous_command;
    /**
     * The present state of the Executor Class 
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
     * Provides the next arm command according to the present situation 
     */
    bool updateArmCommandAndPose(Joints &j_next_arm_command, const Joints &j_present_joints_m);
    // Temporal fix for motion command bug
    void fixMotionCommand(MotionCommand &mc_m);

    std::vector<double> vd_arm_posmargin = {0.2,0.2,0.2,0.2,0.2,0.2};// TODO - Adhoc margin for arm positions
    int i_iteration_counter;
    int i_current_coverage_index; 
    double d_call_period;
public:
    /**
     * Class Constructor using the present motion plan 
     */
    MobileManipExecutor(MotionPlan* presentMotionPlan, const Joints &j_present_readings, std::string s_urdf_path_m);
    /**
     * Update the data extracted from the motion plan 
     */
    void updateMotionPlan();

    /**
     * Indicates whether the rover is within the corridor or not 
     */
    bool isRoverWithinCorridor(Pose pose_rover);

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
     * Checks if the arm is currently colliding 
     */
    bool isArmColliding();
    /**
     * Checks if the arm is not in forbidden workspace 
     */
    bool isArmSafe(const Joints &j_present_joints_m);
    /**
     * Checks if the arm is still following the arm commands 
     */
    bool isArmFollowing(const Joints &j_next_command, const Joints &j_present_joints);
    /**
     * Returns a (0,0,0) rover command 
     */
    MotionCommand getZeroRoverCommand();
    void getSamplingCommand(const Joints &j_arm_present_readings_m, Joints &j_next_arm_command_m);
    void getAtomicCommand();
    unsigned int getRetrievalCommand(const Joints &j_arm_present_readings_m, Joints &j_next_arm_command_m);
    unsigned int getCoverageCommand(Joints &j_next_arm_command, const Joints &j_present_joints_m);
    std::vector<double>* getArmCurrentReadings();
};
