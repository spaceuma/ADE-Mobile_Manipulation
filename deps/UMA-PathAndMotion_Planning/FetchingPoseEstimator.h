#ifndef __FETCHING_POSE_ESTIMATOR__
#define __FETCHING_POSE_ESTIMATOR__

#include "ArmPlanner.h"
#include "Waypoint.hpp"
#include <vector>

#define pi 3.14159265359

namespace FetchingPoseEstimator_lib
{
class FetchingPoseEstimator
{
private:
public:
    // -- PARAMETERS --
    double d0 = 0.500;
    double a1 = 0.225;
    double a2 = 0.735;
    double c2 = 0.030;
    double a3 = 0.030;
    double d4 = 0.695;
    double d6 = 0.300;

    double heightGround2BCS = 0.645;
    double fetchingZDistance = 0.4;

    double maxFetchingDistance
        = a1
          + sqrt(pow(a2 + d4, 2)
                 - pow(heightGround2BCS + d0 - d6 - fetchingZDistance, 2));
    double minFetchingDistance = 0.7;

    // -- FUNCTIONS --
    int getFetchWaypointIndex(const std::vector<base::Waypoint> *roverPath);

    double getReachabilityCost(double distance);

    double getHeadingCost(base::Waypoint pathPose, base::Waypoint samplePose);
};
} // namespace FetchingPoseEstimator_lib

#endif
