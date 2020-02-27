#ifndef __MOBILE_MANIP_MOTION_PLAN__
#define __MOBILE_MANIP_MOTION_PLAN__

// Respect the order of headers
#include "Waypoint.hpp"
// Must come after
#include "FastMarching.h"
#include "CollisionDetector.h"
#include "MobileManipMap.h"
#include <types/RoverGuidance_Dem.h>
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
    ArmPlanner arm_planner;
    CollisionDetector collision_detector;
    std::string s_urdf_path;
    /**
     * Position of the sample in waypoint format
     */
    Waypoint w_sample_pos;
    /**
     * Profile of position values per joint and sample
     */
    std::vector<std::vector<double>> vvd_arm_motion_profile;
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
    bool isArmProfileSafe();
public:
    /**
     * Class Constructor.
     */
    MotionPlan(MobileManipMap * pmmmap_m, double d_zres_m, std::string s_urdf_path_m);
    /**
     * An existing path and profile are introduced into the motion plan 
     */
    MotionPlan(std::vector<Waypoint> &vw_rover_path_m,
               std::vector<std::vector<double>> &vj_joints_profile_m);    
    /**
     * A pointer to the current end effector path is returned
     */
    std::vector<std::vector<double>> * getEndEffectorPath();
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
     * A pointer to the 3d cost map is returned
     */
    std::vector<std::vector<std::vector<double>>> * get3DCostMap();
    /**
     * The path for the rover base, vw_rover_path, is calculated
     */
    unsigned int executeRoverBasePathPlanning(base::Waypoint w_rover_pos_m,
                                      base::Waypoint w_sample_pos_m);
    /**
     * The path is shortened, ending in the best waypoint to fetch the sample
     */
    bool shortenPathForFetching();
    /**
     * Calculates the path of the end effector
     */
    bool executeEndEffectorPlanning();
};

#endif
