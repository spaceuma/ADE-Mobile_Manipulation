#ifndef __FETCHING_POSE_ESTIMATOR__ 
#define __FETCHING_POSE_ESTIMATOR__

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
    double maxFetchingDistance = 1.3895;
    double minFetchingDistance = 0.0; // TODO define parameters properly

    // -- FUNCTIONS --
    int getFetchWaypointIndex(const std::vector<base::Waypoint> *roverPath);

    double getReachabilityCost(double distance);

    double getHeadingCost(base::Waypoint pathPose, base::Waypoint samplePose);
};
} // namespace FetchingPoseEstimator_lib

#endif
