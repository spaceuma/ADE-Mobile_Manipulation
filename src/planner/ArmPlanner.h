#include "FastMarching.h"
#include "KinematicModel.h"
#include "Waypoint.hpp"
#include <vector>

namespace ArmPlanner_lib
{
class ArmPlanner
{
private:
public:
    // -- PARAMETERS --

    // SHERPA_TT
    double d0 = 0.500;
    double a1 = 0.225;
    double a2 = 0.735;
    double c2 = 0.030;
    double a3 = 0.030;
    double d4 = 0.695;
    double d6 = 0.300;

    // Manipulator Workspace
    double maxArmDistance = a2 + d4 + d6;
    double maxZArm = maxArmDistance + d0;
    double maxXYArm = maxArmDistance + a1;
    double maxArmOptimalDistance = maxArmDistance - d6;
    double minArmOptimalDistance = d6;
    double minArmDistance = 0.0;
    double optimalArmRadius = (maxArmOptimalDistance + minArmOptimalDistance) / 2;

    // Geometric parameters (BCS = Body Coordinate System, EE = end effector)
    double heightGround2BCS = 0.645;
    double optimalLeftDeviation = 0.6;                              // TODO set parameter properly
    std::vector<double> BCS2iniEEpos = {0.738, 0, 0.550};           // TODO set parameter properly
    std::vector<double> iniEEorientation = {-1.933, 0.751, -1.822}; // TODO set parameter properly

    // -- FUNCTIONS --
    void planEndEffectorPath(const std::vector<base::Waypoint> *roverPath,
                             const std::vector<std::vector<double>> *DEM,
                             double mapResolution,
                             double zResolution,
                             base::Waypoint samplePos,
                             std::vector<std::vector<double>> *endEffectorPath6,
                             std::vector<int> *pathsAssignment);

    void generateTunnel(const std::vector<std::vector<double>> *roverPath6,
                        const std::vector<std::vector<double>> *DEM,
                        double mapResolution,
                        double zResolution,
                        base::Waypoint iniPos,
                        base::Waypoint samplePos,
                        std::vector<std::vector<std::vector<double>>> *costMap3D);

    void computeWaypointAssignment(const std::vector<std::vector<double>> *roverPath6,
                                   const std::vector<std::vector<double>> *endEffectorPath6,
                                   double assignmentDistance,
                                   std::vector<int> *pathsAssignment);

    double getDist3(std::vector<double> a, std::vector<double> b);
};
