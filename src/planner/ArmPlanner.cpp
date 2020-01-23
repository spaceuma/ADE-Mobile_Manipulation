#include "ArmPlanner.h"
#include <fstream>
#include <iostream>
#include <math.h>

#define pi 3.14159265359

using namespace KinematicModel_lib;
using namespace ArmPlanner_lib;

void ArmPlanner::planArmMotion(std::vector<base::Waypoint> *roverPath,
                               const std::vector<std::vector<double>> *DEM,
                               double mapResolution,
                               double zResolution,
                               base::Waypoint samplePos,
                               std::vector<std::vector<double>> *armJoints)
{
    // Rover z coordinate and heading computation
    std::vector<std::vector<double>> *roverPath6 = new std::vector<std::vector<double>>;
    endEffectorPath6 = new std::vector<std::vector<double>>;
    roverPath6->resize(roverPath->size(), std::vector<double>(6));

    std::vector<double> heading, smoothedHeading;

    (*roverPath6)[0][0] = (*roverPath)[0].position[0];
    (*roverPath6)[0][1] = (*roverPath)[0].position[1];
    (*roverPath6)[0][2] = (*DEM)[(int)((*roverPath)[0].position[1] / mapResolution + 0.5)]
                                [(int)((*roverPath)[0].position[0] / mapResolution + 0.5)]
                          + heightGround2BCS;
    (*roverPath6)[0][3] = 0;
    (*roverPath6)[0][4] = 0;
    (*roverPath6)[0][5] = (*roverPath)[0].heading;

    heading.push_back((*roverPath)[0].heading);

    double offset = 0;
    for (int i = 1; i < roverPath->size(); i++)
    {
        (*roverPath6)[i][0] = (*roverPath)[i].position[0];
        (*roverPath6)[i][1] = (*roverPath)[i].position[1];
        (*roverPath6)[i][2] = (*DEM)[(int)((*roverPath)[i].position[1] / mapResolution + 0.5)]
                                    [(int)((*roverPath)[i].position[0] / mapResolution + 0.5)]
                              + heightGround2BCS;
        (*roverPath6)[i][3] = 0;
        (*roverPath6)[i][4] = 0;
        (*roverPath6)[i][5] = (*roverPath)[i].heading;
        if ((*roverPath)[i].heading - (*roverPath)[i - 1].heading > pi)
            offset -= 2 * pi;
        else if ((*roverPath)[i].heading - (*roverPath)[i - 1].heading < -pi)
            offset += 2 * pi;

        heading.push_back((*roverPath)[i].heading + offset);
    }

    double sigma = 2;
    int samples = 5;
    smoothedHeading = getGaussSmoothen(heading, sigma, samples);

    for (int i = samples / 2 + 1; i < roverPath->size() - samples / 2 - 1; i++)
    {
        (*roverPath6)[i][5] = smoothedHeading[i];
        while ((*roverPath6)[i][5] > pi)
            (*roverPath6)[i][5] -= 2 * pi;
        while ((*roverPath6)[i][5] < -pi)
            (*roverPath6)[i][5] += 2 * pi;
    }

    // Initial arm pos computation
    base::Waypoint iniPos;
    iniPos.position[0] = (*roverPath6)[0][0] + BCS2iniEEpos[0] * cos((*roverPath6)[0][5] + pi / 3);
    iniPos.position[1] = (*roverPath6)[0][1] + BCS2iniEEpos[0] * sin((*roverPath6)[0][5] + pi / 3);
    iniPos.position[2] = (*roverPath6)[0][2] + BCS2iniEEpos[2];

    // The sample position is slightly changed to avoid possible collisions with
    // the mast
    samplePos.position[0] += optimalLeftDeviation * cos((*roverPath6)[roverPath6->size() - 1][5] + pi / 2);
    samplePos.position[1] += optimalLeftDeviation * sin((*roverPath6)[roverPath6->size() - 1][5] + pi / 2);
    samplePos.position[2]
        = (*DEM)[(int)(samplePos.position[1] / mapResolution + 0.5)][(int)(samplePos.position[0] / mapResolution + 0.5)]
          + fetchingZDistance;

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

    volume_cost_map = new std::vector<std::vector<std::vector<double>>>;
    volume_cost_map->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < l; k++)
                (*volume_cost_map)[i][j][k] = INFINITY;

    clock_t init = clock();
    generateTunnel(roverPath6, DEM, mapResolution, zResolution, iniPos, samplePos, volume_cost_map);
    clock_t endt = clock();
    double t = double(endt - init) / CLOCKS_PER_SEC;

    // End effector path planning
    FastMarching_lib::FastMarching3D pathPlanner3D;
    std::vector<base::Waypoint> *endEffectorPath = new std::vector<base::Waypoint>;

    clock_t inip = clock();
    pathPlanner3D.planPath(volume_cost_map, mapResolution, zResolution, iniPos, samplePos, endEffectorPath);

    // Orientation (roll, pitch, yaw) of the end effector at each waypoint
    endEffectorPath6->resize(endEffectorPath->size(), std::vector<double>(6));

    std::vector<double> finalEEorientation = {-pi, 0, -pi};
    for (int i = 0; i < endEffectorPath->size(); i++)
    {
        (*endEffectorPath6)[i][0] = (*endEffectorPath)[i].position[0];
        (*endEffectorPath6)[i][1] = (*endEffectorPath)[i].position[1];
        (*endEffectorPath6)[i][2] = (*endEffectorPath)[i].position[2];

        (*endEffectorPath6)[i][3]
            = iniEEorientation[0] + i * (finalEEorientation[0] - iniEEorientation[0]) / (endEffectorPath->size() - 1);
        (*endEffectorPath6)[i][4]
            = iniEEorientation[1] + i * (finalEEorientation[1] - iniEEorientation[1]) / (endEffectorPath->size() - 1);
        (*endEffectorPath6)[i][5]
            = iniEEorientation[2] + i * (finalEEorientation[2] - iniEEorientation[2]) / (endEffectorPath->size() - 1);
    }

    // Paths inbetween assignment
    std::vector<int> *pathsAssignment = new std::vector<int>;
    computeWaypointAssignment(roverPath6, endEffectorPath6, pathsAssignment);

    // Waypoint interpolation to smooth the movements of the arm joints
    interpolatedRoverPath = new std::vector<base::Waypoint>(roverPath6->size());
    std::vector<int> *interpolatedAssignment = new std::vector<int>(roverPath6->size());
    computeWaypointInterpolation(roverPath6, pathsAssignment, interpolatedRoverPath, interpolatedAssignment);

    // Computing inverse kinematics
    Manipulator sherpa_tt_arm;

    for (int i = 0; i < interpolatedRoverPath->size(); i++)
    {
        int eeInd = (*interpolatedAssignment)[i];
        std::vector<std::vector<double>> TW2BCS(4, std::vector<double>(4));
        std::vector<std::vector<double>> TW2EE(4, std::vector<double>(4));
        std::vector<std::vector<double>> TBCS2EE(4, std::vector<double>(4));

        double x = (*interpolatedRoverPath)[i].position[0];
        double y = (*interpolatedRoverPath)[i].position[1];
        double z = (*interpolatedRoverPath)[i].position[2];
        std::vector<double> position{x, y, z};
        double roll = 0;
        double pitch = 0;
        double yaw = (*interpolatedRoverPath)[i].heading;

        TW2BCS = dot(getTraslation(position), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        position[0] = (*endEffectorPath6)[eeInd][0];
        position[1] = (*endEffectorPath6)[eeInd][1];
        position[2] = (*endEffectorPath6)[eeInd][2];
        roll = (*endEffectorPath6)[eeInd][3];
        pitch = (*endEffectorPath6)[eeInd][4];
        yaw = (*endEffectorPath6)[eeInd][5];

        TW2EE = dot(getTraslation(position), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        TBCS2EE = dot(getInverse(&TW2BCS), TW2EE);

        position[0] = TBCS2EE[0][3];
        position[1] = TBCS2EE[1][3];
        position[2] = TBCS2EE[2][3];
        // TODO change the orientation from absolute to relative
        roll = (*endEffectorPath6)[eeInd][3];
        pitch = (*endEffectorPath6)[eeInd][4];
        yaw = (*endEffectorPath6)[eeInd][5];
        std::vector<double> orientation{roll, pitch, yaw};

        std::vector<double> config = sherpa_tt_arm.getManipJoints(position, orientation, 1, 1);
        // if(config == std::vector<double>(1,0))
        // armJoints->push_back((*armJoints)[armJoints->size()-1]);
        // else
        armJoints->push_back(config);
    }

    (*roverPath) = (*interpolatedRoverPath);

    clock_t endp = clock();
    double tp = double(endp - inip) / CLOCKS_PER_SEC;
}

std::vector<std::vector<double>> * ArmPlanner::getEEPath()
{
    return endEffectorPath6;
}

std::vector<base::Waypoint> * ArmPlanner::getInterpolatedRoverPath()
{
    return interpolatedRoverPath;
}	

std::vector<std::vector<std::vector<double>>> * ArmPlanner::getVolumeCostMap()
{
    return this->volume_cost_map;
}
    ///////////////////////////////////////////////////////////
    // Printing results into .txt files

    
/*    std::ofstream inipathFile;
    inipathFile.open("test/unit/data/results/iniroverPath.txt");

    for (int j = 0; j < roverPath->size(); j++)
    {
        inipathFile << (*roverPath6)[j][0] << " " << (*roverPath6)[j][1] << " " << (*roverPath6)[j][2] << " "
                    << (*roverPath6)[j][5] << "\n";
    }

    inipathFile.close();

    std::ofstream pathFile;
    pathFile.open("test/unit/data/results/roverPath.txt");

    for (int j = 0; j < interpolatedRoverPath->size(); j++)
    {
        pathFile << (*interpolatedRoverPath)[j].position[0] << " " << (*interpolatedRoverPath)[j].position[1] << " "
                 << (*interpolatedRoverPath)[j].position[2] << " " << (*interpolatedRoverPath)[j].heading << "\n";
    }

    pathFile.close();

    std::ofstream cMap3DFile;
    cMap3DFile.open("test/unit/data/results/cMap3D.txt");

    cMap3DFile << (*costMap3D).size() << " " << (*costMap3D)[0].size() << " " << (*costMap3D)[0][0].size() << "\n";
    cMap3DFile << mapResolution << " " << zResolution << "\n";
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

    std::ofstream path3DFile;
    path3DFile.open("test/unit/data/results/EEPath.txt");

    for (int j = 0; j < endEffectorPath6->size(); j++)
    {
        path3DFile << (*endEffectorPath6)[j][0] << " " << (*endEffectorPath6)[j][1] << " " << (*endEffectorPath6)[j][2]
                   << " " << (*endEffectorPath6)[j][3] << " " << (*endEffectorPath6)[j][4] << " "
                   << (*endEffectorPath6)[j][5] << "\n";
    }

    path3DFile.close();


    std::ofstream armJointsFile;
    armJointsFile.open("test/unit/data/results/armJoints.txt");

    for (int i = 0; i < armJoints->size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            armJointsFile << (*armJoints)[i][j] << " ";
        }
        armJointsFile << "\n";
    }

    armJointsFile.close();*/
    ///////////////////////////////////////////////////////////

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
                        = {{1, 0, 0, 0}, {0, 1, 0, mapResolution * j}, {0, 0, 1, d0 + zResolution * k}, {0, 0, 0, 1}};

                    std::vector<std::vector<double>> TW2Node = dot(TW2BCS, TBCS2Node);

                    int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                    int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                    int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                    double cost = 1
                                  + abs(sqrt(pow(mapResolution * j - maxArmDistance / 2, 2)
                                             + pow(zResolution * k - maxArmDistance / 2, 2))
                                        / (maxArmDistance / 2));

                    if (ix > 0 && iy > 0 && iz > 0 && ix < sx - 1 && iy < sy - 1 && iz < sz - 1)
                        if (isinf((*costMap3D)[iy][ix][iz])) (*costMap3D)[iy][ix][iz] = cost;

                    if (ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1 && iz < sz - 1)
                        if (isinf((*costMap3D)[iy][ix + 1][iz])) (*costMap3D)[iy][ix + 1][iz] = cost;
                    if (ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1 && iz < sz - 1)
                        if (isinf((*costMap3D)[iy][ix - 1][iz])) (*costMap3D)[iy][ix - 1][iz] = cost;
                    if (ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1 && iz < sz - 1)
                        if (isinf((*costMap3D)[iy + 1][ix][iz])) (*costMap3D)[iy + 1][ix][iz] = cost;
                    if (ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1 && iz < sz - 1)
                        if (isinf((*costMap3D)[iy - 1][ix][iz])) (*costMap3D)[iy - 1][ix][iz] = cost;
                }
            }
    }

    // Last stretch of the tunnel
    double iRes, iDis;
    int numExtraWayp = (int)(30 * 0.1 / zResolution + 0.5);
    for (int i = 0; i < numExtraWayp; i++)
    {
        std::vector<double> pos{(*roverPath6)[n - 1][0], (*roverPath6)[n - 1][1], (*roverPath6)[n - 1][2]};
        double roll = (*roverPath6)[n - 1][3];
        double pitch = (*roverPath6)[n - 1][4];
        double yaw = (*roverPath6)[n - 1][5];
        std::vector<std::vector<double>> TW2BCS
            = dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        double pitchEnd = i * pi / 2 / numExtraWayp - (*roverPath6)[n - 1][4];
        std::vector<double> desp1{-sin(pitchEnd) * heightGround2BCS, 0, -heightGround2BCS};
        std::vector<double> desp2{0, 0, heightGround2BCS};
        std::vector<std::vector<double>> TBCS2NewWayp
            = dot(getTraslation(desp1), dot(getYrot(pitchEnd), getTraslation(desp2)));

        std::vector<std::vector<double>> TW2NewWayp = dot(TW2BCS, TBCS2NewWayp);

        iRes = zResolution + sin(pitchEnd) * (mapResolution - zResolution);
        iDis = d0 + sin(pitchEnd) * (a1 - d0);

        for (int j = 0; j < 2 * tunnelSizeY; j++)
            for (int k = 0; k < 2 * (int)(tunnelSizeZ * zResolution / iRes + 0.5); k++)
            {
                double dist = sqrt(pow(mapResolution * j / 2, 2) + pow(zResolution * k / 2, 2));
                if (dist < maxArmDistance)
                {
                    std::vector<std::vector<double>> TNewWayp2Node = {
                        {1, 0, 0, 0}, {0, 1, 0, mapResolution * j / 2}, {0, 0, 1, iDis + iRes * k / 2}, {0, 0, 0, 1}};

                    std::vector<std::vector<double>> TW2Node = dot(TW2NewWayp, TNewWayp2Node);

                    int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                    int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                    int iz = (int)(TW2Node[2][3] / zResolution + 0.5);

                    double cost = 1
                                  + abs(sqrt(pow(mapResolution * j / 2 - maxArmDistance / 2, 2)
                                             + pow(zResolution * k / 2 - maxArmDistance / 2, 2))
                                        / (maxArmDistance / 2));

                    if (ix > 0 && iy > 0 && iz > 0 && ix < sx - 1 && iy < sy - 1 && iz < sz - 1)
                        if (TW2Node[2][3] > (*DEM)[iy][ix])
                            if (isinf((*costMap3D)[iy][ix][iz])) (*costMap3D)[iy][ix][iz] = cost;

                    if (ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1 && iz < sz - 1)
                        if (TW2Node[2][3] > (*DEM)[iy][ix + 1])
                            if (isinf((*costMap3D)[iy][ix + 1][iz])) (*costMap3D)[iy][ix + 1][iz] = cost;
                    if (ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1 && iz < sz - 1)
                        if (TW2Node[2][3] > (*DEM)[iy][ix - 1])
                            if (isinf((*costMap3D)[iy][ix - 1][iz])) (*costMap3D)[iy][ix - 1][iz] = cost;
                    if (ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1 && iz < sz - 1)
                        if (TW2Node[2][3] > (*DEM)[iy + 1][ix])
                            if (isinf((*costMap3D)[iy + 1][ix][iz])) (*costMap3D)[iy + 1][ix][iz] = cost;
                    if (ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1 && iz < sz - 1)
                        if (TW2Node[2][3] > (*DEM)[iy - 1][ix])
                            if (isinf((*costMap3D)[iy - 1][ix][iz])) (*costMap3D)[iy - 1][ix][iz] = cost;
                }
            }
    }
}

void ArmPlanner::computeWaypointAssignment(const std::vector<std::vector<double>> *roverPath6,
                                           const std::vector<std::vector<double>> *endEffectorPath6,
                                           std::vector<int> *pathsAssignment)
{
    std::vector<double> armBasePos;
    (*pathsAssignment) = std::vector<int>(roverPath6->size(), 0);

    for (int i = 0; i < roverPath6->size(); i++)
    {
        armBasePos = (*roverPath6)[i];
        armBasePos[2] += d0;
        for (int j = endEffectorPath6->size() - 1; j > -1; j--)
        {
            if (getDist3(armBasePos, (*endEffectorPath6)[j]) < maxArmOptimalDistance)
            {
                (*pathsAssignment)[i] = j;
                break;
            }
        }
    }

    for (int i = pathsAssignment->size() - 1; i > 0; i--)
        if ((*pathsAssignment)[i] < (*pathsAssignment)[i - 1]) (*pathsAssignment)[i - 1] = (*pathsAssignment)[i];

    (*pathsAssignment)[0] = 0;
    (*pathsAssignment)[roverPath6->size() - 1] = endEffectorPath6->size() - 1;
}

void ArmPlanner::computeWaypointInterpolation(const std::vector<std::vector<double>> *roverPath6,
                                              const std::vector<int> *pathsAssignment,
                                              std::vector<base::Waypoint> *newRoverPath,
                                              std::vector<int> *newAssignment)
{
    for (int i = 0; i < pathsAssignment->size(); i++)
    {
        (*newRoverPath)[i].position[0] = (*roverPath6)[i][0];
        (*newRoverPath)[i].position[1] = (*roverPath6)[i][1];
        (*newRoverPath)[i].position[2] = (*roverPath6)[i][2];
        (*newRoverPath)[i].heading = (*roverPath6)[i][5];

        (*newAssignment)[i] = (*pathsAssignment)[i];
    }

    int i = 1;
    while (i < newRoverPath->size())
    {
        int diff = (*newAssignment)[i] - (*newAssignment)[i - 1];
        if (diff > 5)
        {
            std::vector<base::Waypoint> newWaypoints
                = getCubicInterpolation((*newRoverPath)[i - 1], (*newRoverPath)[i], diff - 1);
            newRoverPath->insert(newRoverPath->begin() + i, newWaypoints.begin(), newWaypoints.end());
            for (int j = 0; j < diff - 1; j++)
                newAssignment->insert(newAssignment->begin() + i + j, (*newAssignment)[i - 1] + j + 1);
            i += diff - 1;
        }
        i++;
    }
}

std::vector<base::Waypoint> ArmPlanner::getCubicInterpolation(base::Waypoint waypoint0,
                                                              base::Waypoint waypoint1,
                                                              int numberIntWaypoints)
{
    double x0 = waypoint0.position[0];
    double y0 = waypoint0.position[1];
    double yaw0 = waypoint0.heading;
    double dy0 = tan(yaw0);

    double x1 = waypoint1.position[0];
    double y1 = waypoint1.position[1];
    double yaw1 = waypoint1.heading;
    double dy1 = tan(yaw1);

    double A = x1 * x1 * x1 - 3 * x1 * x0 * x0 + 2 * x0 * x0 * x0;
    double B = x1 * x1 - 2 * x1 * x0 + x0 * x0;
    double C = x1 * dy0 + y0 - x0 * dy0;
    double D = (dy1 - dy0) / (3 * (x1 * x1 - x0 * x0));
    double E = 2 * (x1 - x0) / (3 * (x1 * x1 - x0 * x0));

    double b = (y1 - D * A - C) / (B - E * A);
    double a = D - E * b;
    double c = dy0 - 3 * a * x0 * x0 - 2 * b * x0;
    double d = y0 - a * x0 * x0 * x0 - b * x0 * x0 - c * x0;

    double l;
    std::vector<double> xi(10002, 0);
    std::vector<double> yi(10002, 0);
    std::vector<double> laccum(10002, 0);

    xi[0] = x0;
    yi[0] = y0;
    for (int i = 1; i < 10002; i++)
    {
        xi[i] = x0 + i * (x1 - x0) / 10001;
        yi[i] = a * xi[i] * xi[i] * xi[i] + b * xi[i] * xi[i] + c * xi[i] + d;
        l = sqrt((xi[i] - xi[i - 1]) * (xi[i] - xi[i - 1]) + (yi[i] - yi[i - 1]) * (yi[i] - yi[i - 1]));
        laccum[i] = laccum[i - 1] + l;
    }

    d = laccum[10001] / (numberIntWaypoints + 1);

    std::vector<base::Waypoint> newWaypoints(numberIntWaypoints);
    for (int i = 0; i < numberIntWaypoints; i++)
        for (int j = 0; j < 10002; j++)
            if (laccum[j] > (i + 1) * d)
            {
                newWaypoints[i].position[0] = xi[j];
                newWaypoints[i].position[1] = yi[j];
                newWaypoints[i].position[2] = (waypoint0.position[2] + waypoint1.position[2]) / 2;
                if (abs(waypoint0.heading - waypoint1.heading) > pi)
                    newWaypoints[i].heading = (waypoint0.heading + waypoint1.heading + 2 * pi) / 2;
                if (newWaypoints[i].heading > pi)
                    newWaypoints[i].heading -= 2 * pi;
                else
                    newWaypoints[i].heading = (waypoint0.heading + waypoint1.heading) / 2;
                break;
            }
    return newWaypoints;
}

double ArmPlanner::getDist3(std::vector<double> a, std::vector<double> b)
{
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
}

double ArmPlanner::getGaussValue(double sigma, double x)
{
    double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
    double divider = sqrt(2 * M_PI * pow(sigma, 2));
    return (1 / divider) * exp(expVal);
}

std::vector<double> ArmPlanner::getGaussKernel(int samples, double sigma)
{
    std::vector<double> v;

    bool doubleCenter = false;
    if (samples % 2 == 0)
    {
        doubleCenter = true;
        samples--;
    }
    int steps = (samples - 1) / 2;
    double stepSize = (3 * sigma) / steps;

    for (int i = steps; i >= 1; i--)
    {
        v.push_back(getGaussValue(sigma, i * stepSize * -1));
    }

    v.push_back(getGaussValue(sigma, 0));
    if (doubleCenter)
    {
        v.push_back(getGaussValue(sigma, 0));
    }

    for (int i = 1; i <= steps; i++)
    {
        v.push_back(getGaussValue(sigma, i * stepSize));
    }

    assert(v.size() == samples);

    return v;
}

std::vector<double> ArmPlanner::getGaussSmoothen(std::vector<double> values, double sigma, int samples)
{
    std::vector<double> out;
    auto kernel = getGaussKernel(samples, sigma);
    double kernelSum = 0;
    for (int i = 0; i < kernel.size(); i++)
        kernelSum += kernel[i];
    int sampleSide = samples / 2;
    int valueIdx = samples / 2 + 1;
    unsigned long ubound = values.size();
    for (unsigned long i = 0; i < ubound; i++)
    {
        double sample = 0;
        int sampleCtr = 0;
        for (long j = i - sampleSide; j <= i + sampleSide; j++)
        {
            if (j > 0 && j < ubound)
            {
                int sampleWeightIndex = sampleSide + (j - i);
                sample += kernel[sampleWeightIndex] * values[j];
                sampleCtr++;
            }
        }
        double smoothed = sample / ((double)sampleCtr * kernelSum);
        out.push_back(smoothed * samples);
    }
    return out;
}
