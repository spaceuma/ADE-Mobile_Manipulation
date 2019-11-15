#include "ArmPlanner.h"
#include <iostream>
#include <math.h>

using namespace ArmPlanner_lib;

void ArmPlanner::planEndEffectorPath(const std::vector<base::Waypoint> *roverPath,
                                     const std::vector<std::vector<double>> *DEM,
                                     double mapResolution,
                                     double zResolution,
                                     base::Waypoint samplePos,
                                     std::vector<base::Waypoint> *endEffectorPath,
                                     std::vector<int> *pathsAssignment)
{
    // Initial arm pos computation
    base::Waypoint iniPos;

    // Rover heading computation
    std::vector<double> *roverHeading;
    roverHeading->resize(roverPath->size());

    // Cost map 3D computation
    int n = DEM->size();
    int m = (*DEM)[0].size();
    int l = 100; // TODO define num nodes z in function of resolution, maxz, minz...

    std::vector<std::vector<std::vector<double>>> *costMap3D;
    costMap3D->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < l; k++)
                (*costMap3D)[i][j][k] = INFINITY;

    generateTunnel(roverPath, roverHeading, DEM, mapResolution, zResolution, samplePos, costMap3D);

    // End effector path planning
    FastMarching_lib::BiFastMarching3D pathPlanner3D;

    pathPlanner3D.planPath(costMap3D, mapResolution, zResolution, iniPos, samplePos, endEffectorPath);

    // Paths inbetween assignment

    //TODO what about orientation of the end effector?
}
void ArmPlanner::generateTunnel(const std::vector<base::Waypoint> *roverPath,
                    const std::vector<double> *roverHeading,
                    const std::vector<std::vector<double>> *DEM,
                    double mapResolution,
                    double zResolution,
                    base::Waypoint samplePos,
                    std::vector<std::vector<std::vector<double>>> *costMap3D)
{
}
}
