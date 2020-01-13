#ifndef __MOBILE_MANIP_MOTION_PLAN__
#define __MOBILE_MANIP_MOTION_PLAN__

// Respect the order of headers
#include "Joints.h"
#include "Waypoint.hpp"
// Must come after
#include "ArmOperation.h"
#include "FastMarching.h"
#include "MobileManipMap.h"
#include <types/RoverGuidance_Dem.h>
// Must come after
#include "ArmPlanner.h"
#include "FetchingPoseEstimator.h"

using namespace proxy_library;
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
     * Profile of joint positions to be followed by the arm.
     */
    std::vector<Joints> vj_joints_profile;
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
    /**
     * Position of the sample in waypoint format
     */
    Waypoint w_sample_pos;
    /**
     * Profile of position values per joint and sample
     */
    std::vector<std::vector<double>> vvd_arm_motion_profile;

public:
    /**
     * Class Constructor.
     */
    MotionPlan();
    /**
     * An existing path and profile are introduced into the motion plan 
     */
    void updateMotionPlan(std::vector<Waypoint> vw_rover_path_m,
                          std::vector<Joints> vj_joints_profile_m);
    /**
     * Updates the arm motion profile
     */
    void setArmMotionProfile(
        std::vector<std::vector<double>> &vj_joints_profile_m);
    /**
     * Updates the path
     */
    void setPath(std::vector<Waypoint> &vw_path_m);
    /**
     * A pointer to the current path is returned
     */
    std::vector<base::Waypoint> *getPath();
    /**
     * A pointer to the arm motion profile is returned
     */
    std::vector<std::vector<double>> *getArmMotionProfile();
    /**
     * The path for the rover base, vw_rover_path, is calculated
     */
    void executeRoverBasePathPlanning(MobileManipMap *p_mobile_manip_map_m,
                                      base::Waypoint w_rover_pos_m,
                                      base::Waypoint w_sample_pos_m);
    /**
     * The path is shortened, ending in the best waypoint to fetch the sample
     */
    int shortenPathForFetching();
    /**
     * Calculates the path of the end effector
     */
    void executeEndEffectorPlanning(MobileManipMap *p_mobile_manip_map_m,
                                    double d_z_res_m);
};

#endif
