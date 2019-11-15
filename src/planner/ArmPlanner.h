#include "FastMarching.h"
#include "Waypoint.hpp"
#include <vector>

namespace ArmPlanner_lib
{
class ArmPlanner
{
private:
public:
    // -- PARAMETERS --
    double maxArmDistance = 1; // TODO configure parameters correctly

    // -- FUNCTIONS --
    void planEndEffectorPath(const std::vector<base::Waypoint> *roverPath,
                             const std::vector<std::vector<double>> *DEM,
                             double mapResolution,
                             double zResolution,
                             base::Waypoint samplePos,
                             std::vector<base::Waypoint> *endEffectorPath,
                             std::vector<int> *pathsAssignment);

    void generateTunnel(const std::vector<base::Waypoint> *roverPath,
                        const std::vector<double> *roverHeading,
                        const std::vector<std::vector<double>> *DEM,
                        double mapResolution,
                        double zResolution,
                        base::Waypoint samplePos,
                        std::vector<std::vector<std::vector<double>>> *costMap3D);
};
