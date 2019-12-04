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
    double maxFetchingDistance = 1.5; // TODO define parameters properly
    double minFetchingDistance = 0.5; // TODO define parameters properly

    // -- FUNCTIONS --
    int getFetchWaypointIndex(const std::vector<base::Waypoint> *roverPath);

    double getReachabilityCost(double distance);

    double getHeadingCost(base::Waypoint pathPose, base::Waypoint samplePose);
};
} // namespace FetchingPoseEstimator_lib
