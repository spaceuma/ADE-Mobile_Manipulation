#ifndef __MOBILE_MANIP_MOTION_PLAN__
#define __MOBILE_MANIP_MOTION_PLAN__

// Respect the order of headers
#include "Waypoint.hpp"
// Must come after
#include "CollisionDetector.h"
#include "FastMarching.h"
#include "MobileManipMap.h"
#include "RoverGuidance_InputDataStruct.h"
// Must come after
#include "ArmPlanner.h"
#include "FetchingPoseEstimator.h"

using namespace base;
using namespace FastMarching_lib;
using namespace FetchingPoseEstimator_lib;
using namespace ArmPlanner_lib;

/**
 * This class includes methods to produce the manipulator-rover paths
 */
class MotionPlan
{

private:

    /**
    * Dependency classes
    */
    MobileManipMap *pmm_map;
    BiFastMarching bifm_planner;
    FastMarching fm_planner;
    FetchingPoseEstimator fetching_pose_estimator;
    ArmPlanner *p_arm_planner;
    CollisionDetector *p_collision_detector;
   
    /**
     * Configurable Parameters
     */
    std::string s_urdf_path;
    double d_zres = 0.08;
    double d_gauss_sigma = 5.0;
    int i_gauss_numsamples = 9;
    std::vector<double> vd_retrieval_position 
                              = {2.705, -1.571, 2.443, 0.0, -1.134, 2.354};

    /**
     * Flags
     */
    bool b_is_retrieval_computed = false;
    bool b_is_initialization_computed = false;

    /**
     * Motion Plan results
     */
    std::vector<Waypoint> vw_rover_path;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    std::vector<std::vector<double>> vvd_smoothed_arm_motion_profile;
    std::vector<std::vector<double>> vvd_init_arm_profile;
    std::vector<std::vector<double>> vvd_retrieval_arm_profile;
    std::vector<double> vd_init_time_profile;
    std::vector<double> vd_retrieval_time_profile;
    std::vector<double> vd_atomic_time_profile;

    /**
     * Supporting Functions
     */
    bool isSmoothPath();
    bool isArmProfileSafe(
        const std::vector<std::vector<double>> &vvd_profile_m);

public:

    /**
     * Class Constructor.
     */
    MotionPlan(MobileManipMap *pmmmap_m,
               std::string s_urdf_path_m,
	       unsigned int ui_deployment = 2); //BEGINNING deployment type by default 
    MotionPlan(MobileManipMap *pmmmap_m,
               std::string s_urdf_path_m,
               std::vector<Waypoint> &vw_rover_path_m,
               std::vector<std::vector<double>> &vj_joints_profile_m);

    /**
     * Planning Functions
     */
    void setDeployment(unsigned int ui_deployment);
    unsigned int computeRoverBasePathPlanning(base::Waypoint w_rover_pos_m);
    bool shortenPathForFetching();
    void addSampleWaypointToPath();
    unsigned int computeArmProfilePlanning();
    unsigned int computeArmDeployment(
                                 int i_segment_m,
                                 const std::vector<double> &vd_arm_readings);
    unsigned int computeArmDeployment(
                                 const base::Waypoint &w_goal,
                                 const std::vector<double> &vd_orientation_goal,
                                 const std::vector<double> &vd_arm_readings);
    unsigned int computeArmDeployment(
                                 const std::vector<double> &vd_arm_goal,
                                 const std::vector<double> &vd_arm_readings);
    unsigned int computeArmRetrieval(const std::vector<double> &vd_init, 
		                 int mode = 0);
    unsigned int computeAtomicOperation();
       
    /**
     * Get Data
     */
    std::vector<base::Waypoint> *getRoverPath();
    std::vector<std::vector<double>> *getWristPath();
    std::vector<std::vector<double>> *getCoupledArmMotionProfile();
    std::vector<std::vector<double>> *getInitArmMotionProfile();
    std::vector<std::vector<double>> *getRetrievalArmMotionProfile();
    std::vector<double> *getBackInitArmMotionProfile();
    std::vector<double> *getBackArmMotionProfile();
    std::vector<double> *getAtomicTimeProfile();
    std::vector<double> *getInitArmTimeProfile();
    std::vector<double> *getRetrievalArmTimeProfile();
    unsigned int getNumberWaypoints();
    unsigned int getNumberDeploymentSamples();
    unsigned int getNumberRetrievalSamples();
    std::vector<std::vector<std::vector<double>>> *get3DCostMap();

    /*
     * Checking Functions
     */
    bool isInitArmMotionProfileEmpty();
    bool isPathColliding(); // Used in replanning 
    
    std::vector<std::vector<std::vector<double>>> vvvd_3d_costmap; //TODO: make alternative solution
    std::vector<std::vector<double>> vvd_wristpath;
};

#endif
