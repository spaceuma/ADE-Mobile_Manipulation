// MIT License
// -----------
// 
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// 
// Authors: J. Ricardo Sánchez Ibáñez, Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)


#include "FetchingPoseEstimator.h"
#include <fstream>
#include <iostream>
#include <math.h>

#define pi 3.14159265359

using namespace FetchingPoseEstimator_lib;

int FetchingPoseEstimator::getFetchWaypointIndex(
    const std::vector<base::Waypoint> *roverPath,
    double d_minFetchingDistance,
    double d_maxFetchingDistance)
{
    int fetchWaypointIndex = -1;
    double cost, minCost = 10;

    if (d_minFetchingDistance >= d_maxFetchingDistance)
    {
        //TODO: Print message
        d_minFetchingDistance = minFetchingDistance;
	d_maxFetchingDistance = maxFetchingDistance;
    }

    base::Waypoint samplePos = (*roverPath)[roverPath->size() - 1];
    for (int i = 0; i < roverPath->size(); i++)
    {
        double distance = sqrt(
            pow((*roverPath)[i].position[0] - samplePos.position[0], 2)
            + pow((*roverPath)[i].position[1] - samplePos.position[1], 2));
        if (distance < d_maxFetchingDistance && distance > d_minFetchingDistance)
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
        /*std::cout
            << "\033[1;31mERROR "
               "[FetchingPoseEstimator_lib::getFetchWaypointIndex]: No valid "
               "position for "
               "fetching\033[0m\n";*/
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
