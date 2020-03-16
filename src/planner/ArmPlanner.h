#ifndef __ARM_PLANNER__
#define __ARM_PLANNER__

#include "FastMarching.h"
#include "KinematicModel.h"
#include "Waypoint.hpp"
#include <vector>

#define pi 3.14159265359

namespace ArmPlanner_lib
{
class ArmPlanner
{
private:
    std::vector<base::Waypoint> *interpolatedRoverPath;
    std::vector<std::vector<std::vector<double>>> *volume_cost_map;

public:
    KinematicModel_lib::Manipulator sherpa_tt_arm;

    // -- PARAMETERS --
    // Geometric parameters (BCS = Body Coordinate System, EE = end effector)
    double heightGround2BCS = 0.645;
    double optimalLeftDeviation = sherpa_tt_arm.maxArmDistance / 3;
    double fetchingZDistance = 0.4;
    std::vector<double> finalEEorientation = {-pi, 0, -pi};

    std::vector<std::vector<double>> *roverPath6;
    std::vector<std::vector<double>> *endEffectorPath6;

    // -- FUNCTIONS --
    std::vector<base::Waypoint> *getInterpolatedRoverPath();

    std::vector<std::vector<double>> *getEEPath();

    std::vector<std::vector<std::vector<double>>> *getVolumeCostMap();

    bool planArmMotion(std::vector<base::Waypoint> *roverPath,
                       const std::vector<std::vector<double>> *DEM,
                       double mapResolution,
                       double zResolution,
                       base::Waypoint samplePos,
                       std::vector<std::vector<double>> *armJoints);

    void generateTunnel(
        const std::vector<std::vector<double>> *roverPath6,
        const std::vector<std::vector<double>> *DEM,
        double mapResolution,
        double zResolution,
        base::Waypoint iniPos,
        base::Waypoint samplePos,
        std::vector<std::vector<std::vector<double>>> *costMap3D);

    void computeWaypointAssignment(
        const std::vector<std::vector<double>> *roverPath6,
        const std::vector<std::vector<double>> *endEffectorPath6,
        std::vector<int> *pathsAssignment);

    void computeWaypointInterpolation(
        const std::vector<std::vector<double>> *roverPath6,
        const std::vector<int> *pathsAssignment,
        std::vector<base::Waypoint> *newRoverPath,
        std::vector<int> *newAssignment);

    std::vector<base::Waypoint> getCubicInterpolation(base::Waypoint waypoint0,
                                                      base::Waypoint waypoint1,
                                                      int numberIntWaypoints);

    double getDist3(std::vector<double> a, std::vector<double> b);

    double getGaussValue(double sigma, double x);

    std::vector<double> getGaussKernel(int samples, double sigma);

    std::vector<double> getGaussSmoothen(std::vector<double> values,
                                         double sigma,
                                         int samples);
};
} // namespace ArmPlanner_lib
#endif
