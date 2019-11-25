#include <iostream>
#include <fstream>
#include <math.h>
#include "ArmPlanner.h"

#define pi 3.14159265359

using namespace ArmPlanner_lib;
using namespace KinematicModel_lib;

void ArmPlanner::planEndEffectorPath(const std::vector<base::Waypoint> *roverPath,
                                     const std::vector<std::vector<double>> *DEM,
                                     double mapResolution,
                                     double zResolution,
                                     base::Waypoint samplePos,
                                     std::vector<std::vector<double>> *endEffectorPath6,
                                     std::vector<int> *pathsAssignment)
{
    // Rover z coordinate and heading computation
    std::vector<std::vector<double>> *roverPath6 = new std::vector<std::vector<double>>;
    roverPath6->resize(roverPath->size(), std::vector<double>(6));
    for (int i = 0; i < roverPath->size(); i++)
    {
        (*roverPath6)[i][0] = (*roverPath)[i].position[0];
        (*roverPath6)[i][1] = (*roverPath)[i].position[1];
        (*roverPath6)[i][2] = (*DEM)[(int)((*roverPath)[i].position(1) + 0.5)][(int)((*roverPath)[i].position(0) + 0.5)]
                              + heightGround2BCS;
        // TODO compute properly rover heading
        (*roverPath6)[i][3] = 0;
        (*roverPath6)[i][4] = 0;
        (*roverPath6)[i][5] = (*roverPath)[i].heading;
    }

    // Initial arm pos computation
    base::Waypoint iniPos;
    iniPos.position[0] = (*roverPath6)[0][0] + BCS2iniEEpos[0] * cos((*roverPath6)[0][5] + pi/4);
    iniPos.position[1] = (*roverPath6)[0][1] + BCS2iniEEpos[0] * sin((*roverPath6)[0][5] + pi/4);
    iniPos.position[2] = (*roverPath6)[0][2] + BCS2iniEEpos[2];

    // The sample position is slightly changed to avoid possible collisions with the mast
    samplePos.position[0] += optimalLeftDeviation * cos((*roverPath6)[roverPath6->size() - 1][5] - pi / 2);
    samplePos.position[1] += optimalLeftDeviation * sin((*roverPath6)[roverPath6->size() - 1][5] - pi / 2);

    std::cout<<"New sample position: ["<<samplePos.position[0]<<" "<<samplePos.position[1] <<"]\n";
    // Cost map 3D computation
    int n = DEM->size();
    int m = (*DEM)[0].size();

    double minz = INFINITY, maxz = 0;
    for (int i = 0; i < DEM->size(); i++)
        for (int j = 0; j < (*DEM)[0].size(); j++)
        {
            if ((*DEM)[i][j] < minz) minz = (*DEM)[i][j];
            if ((*DEM)[i][j] > maxz) maxz = (*DEM)[i][j];
        }

    int l = (int)((maxz + heightGround2BCS + maxZArm) / zResolution + 0.5);
    std::cout<<"Z size: "<<l<<"]\n";

    std::vector<std::vector<std::vector<double>>> *costMap3D = new std::vector<std::vector<std::vector<double>>>;
    costMap3D->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < l; k++)
                (*costMap3D)[i][j][k] = INFINITY;

    generateTunnel(roverPath6, DEM, mapResolution, zResolution, iniPos, samplePos, costMap3D);



    ///////////////////////////////////////////////////////////
    std::ofstream cMap3DFile;

    cMap3DFile.open("test/unit/data/results/cMap3D.txt");

    for (int j = 0; j < costMap3D->size(); j++)
    {
        for (int i = 0; i < (*costMap3D)[0].size(); i++)
        {
            for (int k = 0; k < (*costMap3D)[0][0].size(); k++)
            {
                cMap3DFile << (*costMap3D)[j][i][k] << " ";
            }
        }
        cMap3DFile << "\n";
    }

    cMap3DFile.close();
    ///////////////////////////////////////////////////////////




    // End effector path planning
    FastMarching_lib::BiFastMarching3D pathPlanner3D;
    std::vector<base::Waypoint> *endEffectorPath = new std::vector<base::Waypoint>;

    pathPlanner3D.planPath(costMap3D, mapResolution, zResolution, iniPos, samplePos, endEffectorPath);
    std::cout<<"20\n";

    // Orientation (roll, pitch, yaw) of the end effector at each waypoint
    endEffectorPath6->resize(endEffectorPath->size(), std::vector<double>(6));
    std::cout<<"30\n";

    std::vector<double> finalEEorientation = {-pi, 0, -pi};
    for (int i = 0; i < endEffectorPath->size(); i++)
    {
        (*endEffectorPath6)[i][0] = (*endEffectorPath)[i].position[0];
        (*endEffectorPath6)[i][1] = (*endEffectorPath)[i].position[1];
        (*endEffectorPath6)[i][2] = (*endEffectorPath)[i].position[2];

        (*endEffectorPath6)[i][3]
            = iniEEorientation[0] + i * (finalEEorientation[0] - iniEEorientation[0]) / endEffectorPath->size();
        (*endEffectorPath6)[i][4]
            = iniEEorientation[1] + i * (finalEEorientation[1] - iniEEorientation[1]) / endEffectorPath->size();
        (*endEffectorPath6)[i][5]
            = iniEEorientation[2] + i * (finalEEorientation[2] - iniEEorientation[2]) / endEffectorPath->size();
    }
    std::cout<<"40\n";

    // Paths inbetween assignment
    computeWaypointAssignment(roverPath6, endEffectorPath6, optimalArmRadius, pathsAssignment);
    std::cout<<"50\n";
}
void ArmPlanner::generateTunnel(const std::vector<std::vector<double>> *roverPath6,
                                const std::vector<std::vector<double>> *DEM,
                                double mapResolution,
                                double zResolution,
                                base::Waypoint iniPos,
                                base::Waypoint samplePos,
                                std::vector<std::vector<std::vector<double>>> *costMap3D)
{
    int n = roverPath6->size();
    int sx = (*costMap3D).size();
    int sy = (*costMap3D)[0].size();
    int sz = (*costMap3D)[0][0].size();

    std::vector<int> goal(3, 0);
    std::vector<int> start(3, 0);

    start[0] = (int)(iniPos.position[0] / mapResolution + 0.5);
    start[1] = (int)(iniPos.position[1] / mapResolution + 0.5);
    start[2] = (int)(iniPos.position[2] / zResolution + 0.5);
    (*costMap3D)[start[1]][start[0]][start[2]] = 1;

    goal[0] = (int)(samplePos.position[0] / mapResolution + 0.5);
    goal[1] = (int)(samplePos.position[1] / mapResolution + 0.5);
    goal[2] = (int)(samplePos.position[2] / zResolution + 0.5);
    (*costMap3D)[goal[1]][goal[0]][goal[2]] = 1;

    int tunnelSizeY = (int)(maxXYArm / mapResolution + 0.5);
    int tunnelSizeZ = (int)((maxZArm - d0) / zResolution + 0.5);

    for (int i = 0; i < n; i++)
    {
        std::vector<double> pos{(*roverPath6)[i][0], (*roverPath6)[i][1], (*roverPath6)[i][2]};
        double roll = (*roverPath6)[i][3];
        double pitch = (*roverPath6)[i][4];
        double yaw = (*roverPath6)[i][5];
        std::vector<std::vector<double>> TW2BCS
            = dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        for (int j = 0; j < tunnelSizeY; j++)
            for (int k = 0; k < tunnelSizeZ; k++)
            {
                double dist = sqrt(pow(mapResolution * j, 2) + pow(zResolution * k, 2));
                if (dist < maxArmDistance)
                {
                    std::vector<std::vector<double>> TBCS2Node
                        = {{1, 0, 0, 0}, {0, 1, 0, -mapResolution * j}, {0, 0, 1, d0 + zResolution * k}, {0, 0, 0, 1}};

                    std::vector<std::vector<double>> TW2Node = dot(TW2BCS, TBCS2Node);

                    int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                    int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                    int iz = (int)(TW2Node[2][3] / zResolution + 0.5);

                    if (ix > 0 && iy > 0 && iz > 0 && ix < sx && iy < sy && iz < sz)
                    {
                        if (isinf((*costMap3D)[iy][ix][iz]))
                            (*costMap3D)[iy][ix][iz] = 1 + abs(sqrt(pow(mapResolution * j - maxArmDistance / 2, 2)
                                                                    + pow(zResolution * k - maxArmDistance / 2, 2))
                                                               / (maxArmDistance / 2));
                    }
                }
            }
    }

    // Last stretch of the tunnel
    int numExtraWayp = 500;
    for (int i = 0; i < numExtraWayp; i++)
    {
        std::vector<double> pos{(*roverPath6)[n-1][0], (*roverPath6)[n-1][1], (*roverPath6)[n-1][2]};
        double roll = (*roverPath6)[n-1][3];
        double pitch = (*roverPath6)[n-1][4];
        double yaw = (*roverPath6)[n-1][5];
        std::vector<std::vector<double>> TW2BCS
            = dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        std::vector<double> desp1{0, 0, -heightGround2BCS};
        double pitchEnd = i * pi / 2 / numExtraWayp - (*roverPath6)[i][4];
        std::vector<double> desp2{0, 0, heightGround2BCS};
        std::vector<std::vector<double>> TBCS2NewWayp
            = dot(getTraslation(desp1), dot(getYrot(pitchEnd), getTraslation(desp2)));

        std::vector<std::vector<double>> TW2NewWayp = dot(TW2BCS, TBCS2NewWayp);

        for (int j = 0; j < tunnelSizeY; j++)
            for (int k = 0; k < tunnelSizeZ; k++)
            {
                double dist = sqrt(pow(mapResolution * j, 2) + pow(zResolution * k, 2));
                if (dist < maxArmDistance)
                {
                    std::vector<std::vector<double>> TNewWayp2Node
                        = {{1, 0, 0, 0}, {0, 1, 0, -mapResolution * j}, {0, 0, 1, d0 + zResolution * k}, {0, 0, 0, 1}};

                    std::vector<std::vector<double>> TW2Node = dot(TW2NewWayp, TNewWayp2Node);

                    int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                    int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                    int iz = (int)(TW2Node[2][3] / zResolution + 0.5);

                    if (ix > 0 && iy > 0 && iz > 0 && ix < sx && iy < sy && iz < sz)
                        if (TW2Node[2][3] > (*DEM)[iy][ix])
                            if (isinf((*costMap3D)[iy][ix][iz]))
                            {
                                (*costMap3D)[iy][ix][iz] = 1 + abs(sqrt(pow(mapResolution * j - maxArmDistance / 2, 2)
                                                                        + pow(zResolution * k - maxArmDistance / 2, 2))
                                                                   / (maxArmDistance / 2));
                                //std::cout<<"Node ["<<ix<<", "<<iy<<", "<<iz<<"], pos: ["<<TW2Node[0][3]<<", "<<TW2Node[1][3]<<", "<<TW2Node[2][3]<<"]"<<"], new wayp: ["<<TW2NewWayp[0][3]<<", "<<TW2NewWayp[1][3]<<", "<<TW2NewWayp[2][3]<<"]"<<std::endl;
                            }
                }
            }
    }
}

void ArmPlanner::computeWaypointAssignment(const std::vector<std::vector<double>> *roverPath6,
                                           const std::vector<std::vector<double>> *endEffectorPath6,
                                           double assignmentDistance,
                                           std::vector<int> *pathsAssignment)
{
    pathsAssignment->resize(endEffectorPath6->size());
    for (int i = roverPath6->size() - 1; i > -1; i--)
        for (int j = 0; j < endEffectorPath6->size(); j++)
            if (getDist3((*roverPath6)[i], (*endEffectorPath6)[j]) < assignmentDistance) (*pathsAssignment)[j] = i;

    (*pathsAssignment)[0] = 0;
    (*pathsAssignment)[endEffectorPath6->size()] = roverPath6->size() - 1;

    for (int i = pathsAssignment->size() - 1; i > 0; i--)
        if ((*pathsAssignment)[i] < (*pathsAssignment)[i - 1]) (*pathsAssignment)[i - 1] = (*pathsAssignment)[i];
}

double ArmPlanner::getDist3(std::vector<double> a, std::vector<double> b)
{
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
}
}
