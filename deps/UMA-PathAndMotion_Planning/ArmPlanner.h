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

#ifndef __ARM_PLANNER__
#define __ARM_PLANNER__

#include "FastMarching.h"
#include "KinematicModel.h"
#include "Waypoint.hpp"
#include <vector>

#define pi 3.14159265359

#define CONSERVATIVE 1
#define PERMISSIVE 0

#define END 1
#define TRAJECTORY 0
#define BEGINNING -1

#define MAX_HORIZON 1.3
#define MIN_HORIZON 0.3

namespace ArmPlanner_lib
{
class ArmPlanner
{
private:
    std::vector<base::Waypoint> * interpolatedRoverPath;
    std::vector<std::vector<std::vector<double>>> * volume_cost_map;

    std::vector<double> * timeProfile;

public:
    KinematicModel_lib::Manipulator * sherpa_tt_arm;

    // -- PARAMETERS --
    // Geometric parameters (BCS = Body Coordinate System, EE = end effector)
    double heightGround2BCS = 0.645;
    double optimalLeftDeviation = 0.4;
    double fetchingZDistance = 0.6;
    std::vector<double> finalEEorientation = {pi, 0, 0};

    // -- VARIABLES --
    std::vector<std::vector<double>> * roverPath6;
    std::vector<std::vector<double>> * wristPath6;

    double mapResolution;
    double zResolution;
    const std::vector<std::vector<double>> * DEM;

    bool approach, varyingHorizon = false;
    int deployment;
    double horizonDistance;

    // -- FUNCTIONS --
    ArmPlanner(std::string s_data_path_m,
               bool _approach = CONSERVATIVE,
               int _deployment = TRAJECTORY);

    ~ArmPlanner();

    void setApproach(bool _approach);

    bool setDeployment(unsigned int _deployment);

    std::vector<base::Waypoint> * getInterpolatedRoverPath();

    std::vector<std::vector<double>> * getWristPath();

    std::vector<std::vector<std::vector<double>>> * getVolumeCostMap();

    bool planArmMotion(std::vector<base::Waypoint> * roverPath,
                       const std::vector<std::vector<double>> * _DEM,
                       double _mapResolution,
                       double _zResolution,
                       base::Waypoint samplePos,
                       std::vector<std::vector<double>> * armJoints,
                       CollisionDetector * p_collision_detector);

    bool planAtomicOperation(double _mapResolution,
                             double _zResolution,
                             std::vector<double> initialArmConfiguration,
                             std::vector<double> goalArmConfiguration,
                             std::vector<std::vector<double>> * armJoints,
                             std::vector<double> * timeProfile,
                             int mode = 0);

    bool planAtomicOperation(double _mapResolution,
                             double _zResolution,
                             std::vector<double> initialArmConfiguration,
                             base::Waypoint goalEEPosition,
                             std::vector<double> goalEEOrientation,
                             std::vector<std::vector<double>> * armJoints,
                             std::vector<double> * timeProfile);

    // Function deprecated
    bool planAtomicOperation(const std::vector<std::vector<double>> * _DEM,
                             double _mapResolution,
                             double _zResolution,
                             base::Waypoint roverWaypoint,
                             base::Waypoint initialEEPosition,
                             base::Waypoint goalEEPosition,
                             std::vector<std::vector<double>> * armJoints,
                             std::vector<double> * timeProfile);

    unsigned int generateTunnel(base::Waypoint iniPos,
                                base::Waypoint samplePos,
                                std::vector<std::vector<std::vector<double>>> * costMap3D);

    void generateReachabilityTunnel(base::Waypoint iniPos,
                                    base::Waypoint goalPos,
                                    std::vector<double> roverPose6,
                                    bool isTunnelPermisive,
                                    std::vector<std::vector<std::vector<double>>> * costMap3D,
                                    int mode = 0);

    bool computeWaypointAssignment(std::vector<int> * pathsAssignment,
                                   CollisionDetector * p_collision_detector);

    void computeWaypointInterpolation(const std::vector<int> * pathsAssignment,
                                      std::vector<base::Waypoint> * newRoverPath,
                                      std::vector<int> * newAssignment);

    void smoothRoverPathHeading(double headingThreshold);

    std::vector<base::Waypoint> getLinearInterpolation(base::Waypoint waypoint0,
                                                       base::Waypoint waypoint1,
                                                       int numberIntWaypoints);

    std::vector<std::vector<double>> getLinearInterpolation(std::vector<double> waypoint0,
                                                            std::vector<double> waypoint1,
                                                            int numberIntWaypoints);

    std::vector<base::Waypoint> getCubicInterpolation(base::Waypoint waypoint0,
                                                      base::Waypoint waypoint1,
                                                      int numberIntWaypoints);

    double getDist3(std::vector<double> a, std::vector<double> b);

    double getGaussValue(double sigma, double x);

    std::vector<double> getGaussKernel(int samples, double sigma);

    std::vector<double> getGaussSmoothen(std::vector<double> values, double sigma, int samples);

    void checkIntersections(std::vector<std::vector<std::vector<int>>> * tunnelLabel,
                            std::vector<std::vector<std::vector<double>>> * tunnelCost,
                            int ix,
                            int iy,
                            int iz,
                            int threshold);

    double getTimeArmJointMovement(double initialPosition, double goalPosition, int armJointNumber);

    double getMaxTimeArmMovement(std::vector<double> initialConfiguration,
                                 std::vector<double> goalConfiguration);

    std::vector<double> getTimeProfile(std::vector<std::vector<double>> * armProfile);

    void computeArmProfileGaussSmoothening(const std::vector<std::vector<double>> * armProfile,
                                           std::vector<std::vector<double>> * smoothedArmProfile,
                                           double sigma = 5,
                                           int samples = 5);
};
}    // namespace ArmPlanner_lib
#endif
