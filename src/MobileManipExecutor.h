#include "Joints.h"

#include "CollisionDetector.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "WaypointNavigation.hpp"

// using namespace proxy_library;
using namespace waypoint_navigation_lib;

enum ArmExecutionState
{
    INITIALIZING = 0, // 0
    READY,            // 2
    COUPLED_MOVING,   // 3
    SAMPLING_POS,     // 4
    RETRIEVING        // 5
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
     * Index to current segment
     */
    int i_current_segment;
    /**
     * Limit Index to trigger replanning
     */
    int i_replan_segment = 0;
    /**
     * Distance from rover to sample
     */
    double d_dist_to_sample;
    /**
     * Index to initial segment
     */
    int i_initial_segment;
    unsigned int ui_num_joints = 6;
    bool b_is_last_segment;
    /**
     * Vector of pointers to each waypoint of the present path
     */
    std::vector<base::Waypoint *> vpw_path;
    /**
     * Motion Command class
     */
    proxy_library::MotionCommand motion_command;
    /**
     * Collision Detector class
     */
    CollisionDetector *p_collision_detector;
    /**
     * The arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_arm_motion_profile;
    /**
     * The sweeping arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_arm_sweeping_profile;
    /**
     * LSC: Turning Radius Matrix
     */
    std::vector<std::vector<double>> *pvvd_turning_curvature_matrix;
    /**
     * LSC: Turning Angle Matrix
     */
    std::vector<std::vector<double>> *pvvd_turning_angle_matrix;
    /**
     * LSC: X0 
     */
    std::vector<double> *pvd_lsc_x0;
    /**
     * LSC: Y0
     */
    std::vector<double> *pvd_lsc_y0;
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
     * The retrieval arm motion profile
     */
    std::vector<std::vector<double>> *pvvd_retrieval_arm_profile;
    /**
     * The retrieval arm motion times
     */
    std::vector<double> *pvd_retrieval_time_profile;
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
    // Temporal fix for motion command bug
    void fixMotionCommand(proxy_library::MotionCommand &mc_m);

    bool updateArmPresentReadings(const proxy_library::Joints &j_present_joints_m);
    bool updateArmCommandVectors();
    bool updateArmCommandVectors(
        const std::vector<double> &vd_present_command_m);
    double computeBilinearInterpolation(double x, double y, 
		double x1, double x2, double y1, double y2, double Q11, 
		double Q12, double Q21, double Q22);

    bool assignPresentCommand(proxy_library::Joints &j_command);
    std::vector<double> vd_arm_posmargin
        = {0.2,
           0.2,
           0.2,
           0.2,
           0.2,
           0.2}; // TODO - Adhoc margin for arm positions
    int i_current_coverage_index;
    int i_current_init_index;
    int i_current_retrieval_index;
    double d_operational_time;
    unsigned int ui_current_timestamp;
    unsigned int ui_past_timestamp;
public:
    /**
     * Class Constructor using the present motion plan
     */
    MobileManipExecutor(MotionPlan *presentMotionPlan,
                        std::string s_urdf_path_m,
			unsigned int ui_operation_mode = 3);
 
    /**
     * Sets the operation mode meant to be executed with the arm
     */
    void setOperationMode(unsigned int ui_operation_mode, std::string s_urdf_path_m);
    
    /**
     * Update the data extracted from the motion plan
     */
    void updateMotionPlan();
    void updateRetrieval();
    void updateDeployment();
    /**
     * Indicates whether the rover is within the corridor or not
     */
    bool isRoverWithinCorridor(base::Pose pose_rover);

    /**
     * Provides the next arm command according to the present situation
     */
    unsigned int getCoupledCommand(
        base::Pose &rover_pose,
        const proxy_library::Joints &j_arm_present_readings_m,
        proxy_library::MotionCommand &mc_m,
        proxy_library::Joints &j_next_arm_command_m);
    unsigned int getAtomicCommand(const proxy_library::Joints &j_arm_present_readings_m,
                                      proxy_library::Joints &j_next_arm_command_m,
				      unsigned int ui_mode);
    /**
     * Checks if the arm is at Ready position
     */
    bool isArmReady(proxy_library::Joints &j_next_command,
                    const proxy_library::Joints &j_present_joints);
    /**
     * Checks if the arm is currently colliding
     */
    bool isArmColliding();
    /**
     * Checks if the arm is still following the arm commands
     */
    bool isArmFollowing(const proxy_library::Joints &j_next_command,
                        const proxy_library::Joints &j_present_joints);
    bool isArmMoving(const proxy_library::Joints &j_present_joints);
    /**
     * Returns a (0,0,0) rover command
     */
    proxy_library::MotionCommand getZeroRoverCommand();
    bool getLastSectionCommand(base::Pose &rover_pose, proxy_library::MotionCommand &mc);
    /**
     * Returns a Point Turn rover command
     */
    proxy_library::MotionCommand getPointTurnRoverCommand(double d_turnSpeed_rads);

    /**
     * Arm Variables Initialization
     */
    void initializeArmVariables(const proxy_library::Joints &j_present_readings);
    void resetOperationTime();
    std::vector<double> *getArmCurrentReadings();
    std::vector<double> *getFirstCoverageProfile();
    std::vector<double> *getLastCoverageProfile();
    bool isAligned(base::Pose &rover_pose);
    void printExecutionStatus();
};
