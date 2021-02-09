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
    COUPLED_MOVING,   // 3
    SAMPLING_POS      // 4
};

class MobileManipExecutor
{

private:

    /**
     * Component execution status
     */
    ArmExecutionState armstate; // Executor
    NavigationState navstate; // Waypoint Navigation

    /**
     * Dependency classes
     */
    MotionPlan *p_motion_plan;
    WaypointNavigation waypoint_navigation;
    CollisionDetector *p_collision_detector;

    /**
     * Configurable parameters
     */
    double d_corridor_width;
    double d_dist_to_sample; //Rover-sample distance
    unsigned int ui_num_joints = 6;
    std::vector<double> vd_arm_posmargin = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

    /**
     * Flags and indexes
     */
    bool b_ArmReady; //Arm is initialized or not
    bool b_is_last_segment;
    int i_current_segment;
    int i_initial_segment;
    int i_current_coverage_index;
    int i_current_init_index;
    int i_current_retrieval_index;

    /**
     * Motion profiles and path/time references
     */
    std::vector<std::vector<double>> *pvvd_arm_motion_profile;
    std::vector<std::vector<double>> *pvvd_arm_sweeping_profile;
    std::vector<std::vector<double>> *pvvd_init_arm_profile;
    std::vector<std::vector<double>> *pvvd_retrieval_arm_profile;
    std::vector<base::Waypoint *> vpw_path; //Pointers to rover path
    std::vector<double> *pvd_arm_sweeping_times;
    std::vector<double> *pvd_init_time_profile;
    std::vector<double> *pvd_retrieval_time_profile;

    /**
     * Last Section Control parameters
     */
    std::vector<std::vector<double>> *pvvd_turning_curvature_matrix;
    std::vector<std::vector<double>> *pvvd_turning_angle_matrix;
    std::vector<double> *pvd_lsc_x0;
    std::vector<double> *pvd_lsc_y0;

    /**
     * Control Variables
     */
    std::vector<double> vd_arm_previous_command;
    std::vector<double> vd_arm_present_command;
    std::vector<double> vd_arm_present_readings;
    double d_operational_time;
    unsigned int ui_current_timestamp;
    unsigned int ui_past_timestamp;

    /*
     * Update Functions
     */
    bool updateArmPresentReadings(
		             const proxy_library::Joints &j_present_joints_m);
    bool updateArmCommandVectors();
    bool updateArmCommandVectors(
                             const std::vector<double> &vd_present_command_m);
    bool assignPresentCommand(proxy_library::Joints &j_command);

    /*
     * Supporting Functions
     */
    void fixMotionCommand(proxy_library::MotionCommand &mc_m);
    double computeBilinearInterpolation(double x, double y, 
		double x1, double x2, double y1, double y2, double Q11, 
		double Q12, double Q21, double Q22);

public:

    /**
     * Class Constructor using the present motion plan
     */
    MobileManipExecutor(MotionPlan *presentMotionPlan,
                        std::string s_urdf_path_m,
			unsigned int ui_operation_mode = 3);
 
    /**
     * Initialization 
     */
    void setOperationMode(unsigned int ui_operation_mode, std::string s_urdf_path_m);
    void initializeArmVariables(const proxy_library::Joints &j_present_readings);
    void resetOperationTime();

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
    bool isArmReady(proxy_library::Joints &j_next_command,
                    const proxy_library::Joints &j_present_joints);
    bool isArmColliding();
    bool isArmFollowing(const proxy_library::Joints &j_next_command,
                        const proxy_library::Joints &j_present_joints);
    bool isArmMoving(const proxy_library::Joints &j_present_joints);
    bool isCoupledMoving();
    bool isAligned(base::Pose &rover_pose);

    /**
     * Provides the next commands according to the present situation
     */
    unsigned int getCoupledCommand(
                    base::Pose &rover_pose,
                    const proxy_library::Joints &j_arm_present_readings_m,
                    proxy_library::MotionCommand &mc_m,
                    proxy_library::Joints &j_next_arm_command_m);
    unsigned int getAtomicCommand(
		    const proxy_library::Joints &j_arm_present_readings_m,
                    proxy_library::Joints &j_next_arm_command_m,
                    unsigned int ui_mode);
    proxy_library::MotionCommand getZeroRoverCommand();
    bool getLastSectionCommand(base::Pose &rover_pose, proxy_library::MotionCommand &mc);

    /*
     * Get Data
     */
    std::vector<double> *getArmCurrentReadings();
    std::vector<double> *getFirstCoverageProfile();
    std::vector<double> *getLastCoverageProfile();
    void printExecutionStatus();

};
