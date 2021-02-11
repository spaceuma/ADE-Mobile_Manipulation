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
    int getFetchWaypointIndex(const std::vector<base::Waypoint> *roverPath,
    double d_minFetchingDistance,
    double d_maxFetchingDistance);

    double getReachabilityCost(double distance);

    double getHeadingCost(base::Waypoint pathPose, base::Waypoint samplePose);
};
} // namespace FetchingPoseEstimator_lib

#endif
