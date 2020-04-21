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
    std::vector<base::Waypoint> *interpolatedRoverPath;
    std::vector<std::vector<std::vector<double>>> *volume_cost_map;

public:
    KinematicModel_lib::Manipulator *sherpa_tt_arm;

    // -- PARAMETERS --
    // Geometric parameters (BCS = Body Coordinate System, EE = end effector)
    double heightGround2BCS = 0.645;
    double optimalLeftDeviation = 0.4;
    double fetchingZDistance = 0.4;
    std::vector<double> finalEEorientation = {pi, 0, 0};

    // -- VARIABLES --
    std::vector<std::vector<double>> *roverPath6;
    std::vector<std::vector<double>> *wristPath6;

    double mapResolution;
    double zResolution;
    const std::vector<std::vector<double>> *DEM;

    bool approach, varyingHorizon = false;
    int deployment;
    double horizonDistance;

    // -- FUNCTIONS --
    ArmPlanner(std::string s_data_path_m,
               bool _approach = CONSERVATIVE,
               int _deployment = TRAJECTORY);
    ~ArmPlanner();

    std::vector<base::Waypoint> *getInterpolatedRoverPath();

    std::vector<std::vector<double>> *getWristPath();

    std::vector<std::vector<std::vector<double>>> *getVolumeCostMap();

    bool planArmMotion(std::vector<base::Waypoint> *roverPath,
                       const std::vector<std::vector<double>> *DEM,
                       double mapResolution,
                       double zResolution,
                       base::Waypoint samplePos,
                       std::vector<std::vector<double>> *armJoints);

    void generateTunnel(
        base::Waypoint iniPos,
        base::Waypoint samplePos,
        std::vector<std::vector<std::vector<double>>> *costMap3D);

    void computeWaypointAssignment(std::vector<int> *pathsAssignment);

    void computeWaypointInterpolation(const std::vector<int> *pathsAssignment,
                                      std::vector<base::Waypoint> *newRoverPath,
                                      std::vector<int> *newAssignment);

    std::vector<base::Waypoint> getLinearInterpolation(base::Waypoint waypoint0,
                                                       base::Waypoint waypoint1,
                                                       int numberIntWaypoints);

    std::vector<base::Waypoint> getCubicInterpolation(base::Waypoint waypoint0,
                                                      base::Waypoint waypoint1,
                                                      int numberIntWaypoints);

    double getDist3(std::vector<double> a, std::vector<double> b);

    double getGaussValue(double sigma, double x);

    std::vector<double> getGaussKernel(int samples, double sigma);

    std::vector<double> getGaussSmoothen(std::vector<double> values,
                                         double sigma,
                                         int samples);

    void checkIntersections(std::vector<std::vector<std::vector<int>>> *tunnelLabel,
                            std::vector<std::vector<std::vector<double>>> *tunnelCost,
                            int ix, int iy, int iz, int threshold);
};
} // namespace ArmPlanner_lib
#endif
