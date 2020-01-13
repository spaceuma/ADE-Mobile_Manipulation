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

class MotionPlan
{

private:
    /**
     * sampleTime to be considered during the execution of the planner.
     */
    float sampleTime;
    /**
     * The path that must be followed by the rover, made up by 2d waypoints
     */
    std::vector<Waypoint> roverPath;
    /**
     * Profile of joint positions to be followed by the arm.
     */
    std::vector<Joints> jointsProfile;

    BiFastMarching fmPlanner;
    FetchingPoseEstimator fetchPosePlanner;
    ArmPlanner armPlanner;
    Waypoint samplePos;
    std::vector<std::vector<double>> vvd_arm_motion_profile;

public:
    MotionPlan();

    void updateMotionPlan(std::vector<Waypoint> newRoverPath,
                          std::vector<Joints> newJointsProfile);
    void executeRoverBasePathPlanning(MobileManipMap *inputMap,
                                      base::Waypoint rover_position,
                                      base::Waypoint sample);
    std::vector<base::Waypoint> *getPath();
    std::vector<std::vector<double>> *getArmMotionProfile();
    int shortenPathForFetching();
    void executeEndEffectorPlanning(MobileManipMap *inputMap,
                                    double zResolution);
    void setArmMotionProfile(
        std::vector<std::vector<double>> &m_arm_motion_profile);
    void setPath(std::vector<Waypoint> &vw_path);
};

#endif
