#ifndef __MOBILE_MANIP_MOTION_PLAN__
#define __MOBILE_MANIP_MOTION_PLAN__

// Respect the order of headers
#include "Waypoint.hpp"
// Must come after
#include "FastMarching.h"
#include "CollisionDetector.h"
#include "MobileManipMap.h"
#include  "RoverGuidance_InputDataStruct.h"
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
     * The path that must be followed by the rover, made up by 2d waypoints
     */
    std::vector<Waypoint> vw_rover_path;
    /**
     * Path Planning class that uses bi-Fast Marching
     */
    BiFastMarching bi_fast_marching;
    /**
     * Class to estimate the best waypoint to fetch in an existing path
     */
    FetchingPoseEstimator fetching_pose_estimator;
    /**
     * Motion Planner for the arm
     */
    ArmPlanner* p_arm_planner;
    CollisionDetector* p_collision_detector;
    std::string s_urdf_path;
    std::vector<double> vd_retrieval_position;
    /**
     * Profile of position values per joint and sample
     */
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    /**
     * Profile of position values per joint and sample
     */
    std::vector<std::vector<double>> vvd_smoothed_arm_motion_profile;
    /**
     * Profile of times related with the initialization operation
     */
    std::vector<double> vd_init_time_profile;
    /**
     * Profile of times related with the retrieval operation
     */
    std::vector<double> vd_retrieval_time_profile;
    /**
     * Profile of times related with an atomic operation
     */
    std::vector<double> vd_time_profile;
    /**
     * Profile of position values per joint and sample
     */
    std::vector<std::vector<double>> vvd_init_arm_profile;
    /**
     * Profile of position values per joint and sample
     */
    std::vector<std::vector<double>> vvd_retrieval_arm_profile;
/**
     * Pointer to the map class
     */
    MobileManipMap * pmm_map;
    /**
     * Value of Z resolution
     */
    double d_zres;
    /**
     * Checks if the current path is smooth
     */
    bool isSmoothPath();
    bool isArmProfileSafe(const std::vector<std::vector<double>> &vvd_profile_m);
    base::Waypoint w_rover_pos;
    double d_gauss_sigma = 5.0;
    int i_gauss_numsamples = 5;

    bool b_is_retrieval_computed;
    bool b_is_initialization_computed;

public:
    /**
     * Class Constructor.
     */
    MotionPlan(MobileManipMap * pmmmap_m, double d_zres_m, std::string s_urdf_path_m);
    /**
     * An existing path and profile are introduced into the motion plan 
     */
    MotionPlan(MobileManipMap * pmmmap_m,
               double d_zres_m, std::string s_urdf_path_m,
	       std::vector<Waypoint> &vw_rover_path_m,
               std::vector<std::vector<double>> &vj_joints_profile_m);    
    /**
     * A pointer to the current end effector path is returned
     */
    std::vector<std::vector<double>> * getWristPath();
    /**
     * A pointer to the current rover path is returned
     */
    std::vector<base::Waypoint> * getRoverPath();
    /**
     * A pointer to the current rover path is returned
     */
    unsigned int getNumberWaypoints();
    /**
     * A pointer to the arm motion profile is returned
     */
    std::vector<std::vector<double>> *getArmMotionProfile();
    /**
     * A pointer to the initial arm motion profile is returned
     */
    std::vector<std::vector<double>> *getInitArmMotionProfile();
    std::vector<double> *getBackArmMotionProfile();
    std::vector<double> *getBackInitArmMotionProfile();
    /**
     * A pointer to the initial arm motion profile is returned
     */
    std::vector<double> *getInitArmTimeProfile();
    /**
     * A pointer to the retrieval arm motion profile is returned
     */
    std::vector<std::vector<double>> *getRetrievalArmMotionProfile();
    /**
     * A pointer to the retrieval arm motion profile is returned
     */
    std::vector<double> *getRetrievalArmTimeProfile();
    /**
     * A pointer to the time profile is returned
     */
    std::vector<double> *getTimeProfile();
    /**
     * A pointer to the 3d cost map is returned
     */
    std::vector<std::vector<std::vector<double>>> * get3DCostMap();
    /**
     * The path for the rover base, vw_rover_path, is calculated
     */
    unsigned int computeRoverBasePathPlanning(base::Waypoint w_rover_pos_m);
    /**
     * The path is shortened, ending in the best waypoint to fetch the sample
     */
    bool shortenPathForFetching();

    bool isRetrievalComputed();
    bool isInitializationComputed();
    /**
     * Calculates the profile of positions for the arm
     */
    unsigned int computeArmProfilePlanning();

    /**
     * Calculates the profile of positions for the arm
     */
    unsigned int computeArmDeployment(int i_segment_m, const std::vector<double> &vd_arm_readings);
    unsigned int computeArmDeployment(const base::Waypoint &w_goal, const std::vector<double> &vd_orientation_goal, const std::vector<double> &vd_arm_readings);
    unsigned int computeArmDeployment(const std::vector<double> &vd_arm_goal, const std::vector<double> &vd_arm_readings);

    unsigned int computeArmRetrieval(const std::vector<double> &vd_init);
    unsigned int computeAtomicOperation();
    void setArmGaussFilter(double sigma, int numsamples);
};

#endif
