#include "FetchingPoseEstimator.h"
#include <fstream>
#include <iostream>
#include <math.h>

#define pi 3.14159265359

using namespace FetchingPoseEstimator_lib;

int FetchingPoseEstimator::getFetchWaypointIndex(
    const std::vector<base::Waypoint> *roverPath)
{
    int fetchWaypointIndex = -1;
    double cost, minCost = 10;

    base::Waypoint samplePos = (*roverPath)[roverPath->size() - 1];
    for (int i = 0; i < roverPath->size(); i++)
    {
        double distance = sqrt(
            pow((*roverPath)[i].position[0] - samplePos.position[0], 2)
            + pow((*roverPath)[i].position[1] - samplePos.position[1], 2));
        if (distance < maxFetchingDistance && distance > minFetchingDistance)
        {
            cost = 0;
            cost += getReachabilityCost(distance);
            cost += getHeadingCost((*roverPath)[i],
                                   (*roverPath)[roverPath->size() - 1]);
            if (cost < minCost)
            {
                minCost = cost;
                fetchWaypointIndex = i;
            }
        }
    }
    if (fetchWaypointIndex > 0)
        return fetchWaypointIndex;
    else
    {
        std::cout
            << "\033[1;31mERROR "
               "[FetchingPoseEstimator_lib::getFetchWaypointIndex]: No valid "
               "position for "
               "fetching\033[0m\n";
        return 0;
    }
}

double FetchingPoseEstimator::getReachabilityCost(double distance)
{
    double cost
        = 2 * abs(distance - (maxFetchingDistance + minFetchingDistance) / 2)
          / (maxFetchingDistance - minFetchingDistance);
    return cost;
}

double FetchingPoseEstimator::getHeadingCost(base::Waypoint pathPose,
                                             base::Waypoint samplePose)
{
    double bestHeading = atan2(samplePose.position[1] - pathPose.position[1],
                               samplePose.position[0] - pathPose.position[0]);
    double devAngle = pathPose.heading - bestHeading;

    if (devAngle > pi) devAngle -= 2 * pi;
    if (devAngle < -pi) devAngle += 2 * pi;
    return abs(devAngle) / pi;
}
