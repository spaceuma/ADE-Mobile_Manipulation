#include "ArmPlanner.h"
#include <fstream>
#include <iostream>
#include <math.h>

#define pi 3.14159265359

using namespace KinematicModel_lib;
using namespace ArmPlanner_lib;

ArmPlanner::ArmPlanner(std::string s_data_path_m)
{
   sherpa_tt_arm = new Manipulator(s_data_path_m); 
}
ArmPlanner::~ArmPlanner()
{
    ;
}

bool ArmPlanner::planArmMotion(std::vector<base::Waypoint> *roverPath,
                               const std::vector<std::vector<double>> *_DEM,
                               double _mapResolution,
                               double _zResolution,
                               base::Waypoint samplePos,
                               std::vector<std::vector<double>> *armJoints)
{
    this->mapResolution = _mapResolution;
    this->zResolution = _zResolution;
    this->DEM = _DEM;

    // Rover z coordinate and heading computation
    (*roverPath)[roverPath->size()-1].heading = atan2(samplePos.position[1]-(*roverPath)[roverPath->size()-1].position[1], samplePos.position[0]-(*roverPath)[roverPath->size()-1].position[0]);

    roverPath6 = new std::vector<std::vector<double>>;
    roverPath6->resize(roverPath->size(), std::vector<double>(6));

    wristPath6 = new std::vector<std::vector<double>>;

    std::vector<double> heading, smoothedHeading;

    (*roverPath6)[0][0] = (*roverPath)[0].position[0];
    (*roverPath6)[0][1] = (*roverPath)[0].position[1];
    (*roverPath6)[0][2]
        = (*DEM)[(int)((*roverPath)[0].position[1] / mapResolution + 0.5)]
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
        (*roverPath6)[i][2]
            = (*DEM)[(int)((*roverPath)[i].position[1] / mapResolution + 0.5)]
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
    std::vector<std::vector<double>> initialBCS2Wrist
        = sherpa_tt_arm->getWristTransform(sherpa_tt_arm->initialConfiguration);
    std::vector<double> roverIniPos{
        (*roverPath6)[0][0], (*roverPath6)[0][1], (*roverPath6)[0][2]};
    std::vector<std::vector<double>> world2Wrist
        = dot(dot(getTraslation(roverIniPos), getZrot((*roverPath6)[0][5])),
              initialBCS2Wrist);

    base::Waypoint iniPos;
    iniPos.position[0] = world2Wrist[0][3];
    iniPos.position[1] = world2Wrist[1][3];
    iniPos.position[2] = world2Wrist[2][3];

    // The sample position is slightly changed to fit in the reachability area
    // of the manipulator
    samplePos.position[2]
        = (*DEM)[(int)(samplePos.position[1] / mapResolution + 0.5)]
                [(int)(samplePos.position[0] / mapResolution + 0.5)]
          + fetchingZDistance + sherpa_tt_arm->d6; // Adding d6 to find wrist pos

    std::vector<double> pos{(*roverPath6)[roverPath6->size() - 1][0],
                            (*roverPath6)[roverPath6->size() - 1][1],
                            (*roverPath6)[roverPath6->size() - 1][2]};
    double roll = (*roverPath6)[roverPath6->size() - 1][3];
    double pitch = (*roverPath6)[roverPath6->size() - 1][4];
    double yaw = (*roverPath6)[roverPath6->size() - 1][5];

    std::vector<std::vector<double>> TW2BCS
        = dot(getTraslation(pos),
              dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

    pos = {samplePos.position[0], samplePos.position[1], samplePos.position[2]};

    std::vector<std::vector<double>> TW2Sample = getTraslation(pos);

    std::vector<std::vector<double>> TBCS2Sample
        = dot(getInverse(&TW2BCS), TW2Sample);
    TBCS2Sample[1][3] += optimalLeftDeviation;

    TW2Sample = dot(TW2BCS, TBCS2Sample);
    
    samplePos.position[0] = TW2Sample[0][3];
    samplePos.position[1] = TW2Sample[1][3];

    // Cost map 3D computation
    int n = DEM->size();
    int m = (*DEM)[0].size();

    double maxz = 0;
    for (int i = 0; i < DEM->size(); i++)
        for (int j = 0; j < (*DEM)[0].size(); j++)
            if ((*DEM)[i][j] > maxz) maxz = (*DEM)[i][j];

    int l
        = (int)((maxz + heightGround2BCS + sherpa_tt_arm->maxZArm) / zResolution
                + 0.5);

    volume_cost_map = new std::vector<std::vector<std::vector<double>>>;
    volume_cost_map->resize(
        n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < l; k++)
                (*volume_cost_map)[i][j][k] = INFINITY;

    clock_t init = clock();

    // Generating the reachability tunnel surrounding the rover path
    generateTunnel(iniPos, samplePos, horizonDistance, volume_cost_map);
    clock_t endt = clock();
    double t = double(endt - init) / CLOCKS_PER_SEC;

    // End effector path planning
    FastMarching_lib::FastMarching3D pathPlanner3D;
    std::vector<base::Waypoint> *wristPath = new std::vector<base::Waypoint>;

    clock_t inip = clock();
    pathPlanner3D.planPath(volume_cost_map,
                           mapResolution,
                           zResolution,
                           iniPos,
                           samplePos,
                           wristPath);

    // Orientation (roll, pitch, yaw) of the end effector at each waypoint
    wristPath6->resize(wristPath->size(), std::vector<double>(6));

    for (int i = 0; i < wristPath->size(); i++)
    {
        (*wristPath6)[i][0] = (*wristPath)[i].position[0];
        (*wristPath6)[i][1] = (*wristPath)[i].position[1];
        (*wristPath6)[i][2] = (*wristPath)[i].position[2];

        (*wristPath6)[i][3]
            = sherpa_tt_arm->iniEEorientation[0]
              + i * (finalEEorientation[0] - sherpa_tt_arm->iniEEorientation[0])
                    / (wristPath->size() - 1);
        (*wristPath6)[i][4]
            = sherpa_tt_arm->iniEEorientation[1]
              + i * (finalEEorientation[1] - sherpa_tt_arm->iniEEorientation[1])
                    / (wristPath->size() - 1);
        (*wristPath6)[i][5]
            = sherpa_tt_arm->iniEEorientation[2]
              + i * (finalEEorientation[2] - sherpa_tt_arm->iniEEorientation[2])
                    / (wristPath->size() - 1);
    }

    // Paths inbetween assignment
    std::vector<int> *pathsAssignment = new std::vector<int>;
    computeWaypointAssignment(horizonDistance, pathsAssignment);

    // Waypoint interpolation to smooth the movements of the arm joints
    interpolatedRoverPath = new std::vector<base::Waypoint>(roverPath6->size());
    std::vector<int> *interpolatedAssignment
        = new std::vector<int>(roverPath6->size());
    computeWaypointInterpolation(
        pathsAssignment, interpolatedRoverPath, interpolatedAssignment);

    // Computing inverse kinematics
    /*First, we compute the inverse kinematics of the wrist for all the path
    Then, according to the orientation commanded, which is relative, we compute
    the last three joints of the arm, using the transform of the wrist. */

    for (int i = 0; i < interpolatedRoverPath->size(); i++)
    {
        int wristInd = (*interpolatedAssignment)[i];
        std::vector<std::vector<double>> TW2BCS(4, std::vector<double>(4));
        std::vector<std::vector<double>> TW2Wrist(4, std::vector<double>(4));
        std::vector<std::vector<double>> TBCS2Wrist(4, std::vector<double>(4));

        double x = (*interpolatedRoverPath)[i].position[0];
        double y = (*interpolatedRoverPath)[i].position[1];
        double z = (*interpolatedRoverPath)[i].position[2];
        std::vector<double> position{x, y, z};
        double roll = 0;
        double pitch = 0;
        double yaw = (*interpolatedRoverPath)[i].heading;

        TW2BCS = dot(getTraslation(position),
                     dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        position[0] = (*wristPath6)[wristInd][0];
        position[1] = (*wristPath6)[wristInd][1];
        position[2] = (*wristPath6)[wristInd][2];

        TW2Wrist = getTraslation(position);

        TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

        position[0] = TBCS2Wrist[0][3];
        position[1] = TBCS2Wrist[1][3];
        position[2] = TBCS2Wrist[2][3];
        std::vector<double> config;

        try
        {
            config = sherpa_tt_arm->getPositionJoints(position, 1, 1);

            roll = (*wristPath6)[wristInd][3];
            pitch = (*wristPath6)[wristInd][4];
            yaw = (*wristPath6)[wristInd][5];

            std::vector<double> orientation{roll, pitch, yaw};

            std::vector<double> wristJoints
                = sherpa_tt_arm->getWristJoints(config, orientation);

            config.insert(config.end(), wristJoints.begin(), wristJoints.end());
        }
        catch (std::exception &e)
        {
            std::cout<<"Inverse kinematics failed at waypoint "<<i<<"!"<<std::endl;
            return false;
        }
        // if(config == std::vector<double>(1,0))
        // armJoints->push_back((*armJoints)[armJoints->size()-1]);
        // else
        armJoints->push_back(config);
    }

    (*roverPath) = (*interpolatedRoverPath);

    clock_t endp = clock();
    double tp = double(endp - inip) / CLOCKS_PER_SEC;
    return true;
}

std::vector<std::vector<double>> *ArmPlanner::getWristPath()
{
    return wristPath6;
}

std::vector<base::Waypoint> *ArmPlanner::getInterpolatedRoverPath()
{
    return interpolatedRoverPath;
}

std::vector<std::vector<std::vector<double>>> *ArmPlanner::getVolumeCostMap()
{
    return this->volume_cost_map;
}

void ArmPlanner::generateTunnel(
    base::Waypoint iniPos,
    base::Waypoint samplePos,
    double horizonDistance,
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

    std::vector<double> *minValues = sherpa_tt_arm->minValues;
    std::vector<double> *maxValues = sherpa_tt_arm->maxValues;

    double slope = 0.5;

    // Tunnel in the first waypoint
    int tunnelSizeX
        = (int)(abs((*maxValues)[0] - (*minValues)[0]) / mapResolution + 0.5);
    int tunnelSizeY
        = (int)(abs((*maxValues)[1] - 0) / mapResolution + 0.5);
    int tunnelSizeZ
        = (int)(abs((*maxValues)[2] - (*minValues)[2]) / zResolution + 0.5);

    std::vector<double> pos{
        (*roverPath6)[0][0], (*roverPath6)[0][1], (*roverPath6)[0][2]};
    double roll = (*roverPath6)[0][3];
    double pitch = (*roverPath6)[0][4];
    double yaw = (*roverPath6)[0][5];
    std::vector<std::vector<double>> TW2BCS
        = dot(getTraslation(pos),
              dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

    for (int i = 0; i < tunnelSizeX; i++)
        for (int j = 0; j < tunnelSizeY; j++)
            for (int k = 0; k < tunnelSizeZ; k++)
            {
                double x = (*minValues)[0] + mapResolution * i;
                double y = 0 + mapResolution * j;
                double z = (*minValues)[2] + zResolution * k;
                double dist = sqrt(pow(x, 2) + pow(y, 2)
                                   + pow(z - sherpa_tt_arm->d0, 2));
                if (dist < sherpa_tt_arm->maxArmDistance)
                {
                    std::vector<std::vector<double>> TBCS2Node = {
                        {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                    pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                    if (sherpa_tt_arm->isReachable(pos))
                    {
                        std::vector<std::vector<double>> TW2Node
                            = dot(TW2BCS, TBCS2Node);

                        int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                        int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                        int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                        double cost
                            = 1
                              + slope
                                    / sherpa_tt_arm->getDistanceToCollision(pos);

                        if (ix > 0 && iy > 0 && iz > 0 && ix < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix][iz])
                                (*costMap3D)[iy][ix][iz] = cost;

                        if (ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix + 1][iz])
                                (*costMap3D)[iy][ix + 1][iz] = cost;
                        if (ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix - 1][iz])
                                (*costMap3D)[iy][ix - 1][iz] = cost;
                        if (ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1
                            && iy + 1 < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy + 1][ix][iz])
                                (*costMap3D)[iy + 1][ix][iz] = cost;
                        if (ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1
                            && iy - 1 < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy - 1][ix][iz])
                                (*costMap3D)[iy - 1][ix][iz] = cost;
                    }
                }
            }

    // Tunnel during the rover movement
    tunnelSizeY = (int)(abs((*maxValues)[1] - 0) / mapResolution + 0.5);
    tunnelSizeZ = (int)(abs((*maxValues)[2] - 0) / zResolution + 0.5);

    for (int i = 1; i < n; i++)
    {
        pos = {(*roverPath6)[i][0], (*roverPath6)[i][1], (*roverPath6)[i][2]};
        roll = (*roverPath6)[i][3];
        pitch = (*roverPath6)[i][4];
        yaw = (*roverPath6)[i][5];
        TW2BCS = dot(getTraslation(pos),
                     dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        for (int j = 0; j < tunnelSizeY; j++)
            for (int k = 0; k < tunnelSizeZ; k++)
            {
                double x = horizonDistance;
                double y = 0 + mapResolution * j;
                double z = 0 + zResolution * k;
                double dist = sqrt(pow(x, 2) + pow(y, 2)
                                   + pow(z - sherpa_tt_arm->d0, 2));
                if (dist < sherpa_tt_arm->maxArmDistance)
                {
                    std::vector<std::vector<double>> TBCS2Node = {
                        {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                    pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                    if (sherpa_tt_arm->isReachable(pos))
                    {
                        std::vector<std::vector<double>> TW2Node
                            = dot(TW2BCS, TBCS2Node);

                        int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                        int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                        int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                        double cost
                            = 1
                              + slope
                                    / sherpa_tt_arm->getDistanceToCollision(pos);

                        if (ix > 0 && iy > 0 && iz > 0 && ix < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix][iz])
                                (*costMap3D)[iy][ix][iz] = cost;

                        if (ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix + 1][iz])
                                (*costMap3D)[iy][ix + 1][iz] = cost;
                        if (ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix - 1][iz])
                                (*costMap3D)[iy][ix - 1][iz] = cost;
                        if (ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1
                            && iy + 1 < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy + 1][ix][iz])
                                (*costMap3D)[iy + 1][ix][iz] = cost;
                        if (ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1
                            && iy - 1 < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy - 1][ix][iz])
                                (*costMap3D)[iy - 1][ix][iz] = cost;
                    }
                }
            }
    }

    // Tunnel in the last waypoint
    tunnelSizeX
        = (int)(abs((*maxValues)[0] - (*minValues)[0]) / mapResolution + 0.5);
    tunnelSizeY
        = (int)(abs((*maxValues)[1] - 0) / mapResolution + 0.5);
    tunnelSizeZ
        = (int)(abs((*maxValues)[2] - (*minValues)[2]) / zResolution + 0.5);

    pos = {(*roverPath6)[roverPath6->size() - 1][0],
           (*roverPath6)[roverPath6->size() - 1][1],
           (*roverPath6)[roverPath6->size() - 1][2]};
    roll = (*roverPath6)[roverPath6->size() - 1][3];
    pitch = (*roverPath6)[roverPath6->size() - 1][4];
    yaw = (*roverPath6)[roverPath6->size() - 1][5];
    TW2BCS = dot(getTraslation(pos),
                 dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

    for (int i = 0; i < tunnelSizeX; i++)
        for (int j = 0; j < tunnelSizeY; j++)
            for (int k = 0; k < tunnelSizeZ; k++)
            {
                double x = (*minValues)[0] + mapResolution * i;
                double y = 0 + mapResolution * j;
                double z = (*minValues)[2] + zResolution * k;
                double dist = sqrt(pow(x, 2) + pow(y, 2)
                                   + pow(z - sherpa_tt_arm->d0, 2));
                if (dist < sherpa_tt_arm->maxArmDistance)
                {
                    std::vector<std::vector<double>> TBCS2Node = {
                        {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                    pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                    if (sherpa_tt_arm->isReachable(pos))
                    {
                        std::vector<std::vector<double>> TW2Node
                            = dot(TW2BCS, TBCS2Node);

                        int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                        int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                        int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                        double cost
                            = 1
                              + slope
                                    / sherpa_tt_arm->getDistanceToCollision(pos);

                        if (ix > 0 && iy > 0 && iz > 0 && ix < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix][iz])
                                (*costMap3D)[iy][ix][iz] = cost;

                        if (ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix + 1][iz])
                                (*costMap3D)[iy][ix + 1][iz] = cost;
                        if (ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1
                            && iy < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy][ix - 1][iz])
                                (*costMap3D)[iy][ix - 1][iz] = cost;
                        if (ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1
                            && iy + 1 < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy + 1][ix][iz])
                                (*costMap3D)[iy + 1][ix][iz] = cost;
                        if (ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1
                            && iy - 1 < sy - 1 && iz < sz - 1)
                            if (cost < (*costMap3D)[iy - 1][ix][iz])
                                (*costMap3D)[iy - 1][ix][iz] = cost;
                    }
                }
            }
}

void ArmPlanner::computeWaypointAssignment(double horizonDistance,
                                           std::vector<int> *pathsAssignment)
{
    std::vector<double> armBasePos;
    (*pathsAssignment) = std::vector<int>(roverPath6->size(), 0);

    for (int i = 0; i < roverPath6->size(); i++)
    {
        std::vector<double> pos{
            (*roverPath6)[i][0], (*roverPath6)[i][1], (*roverPath6)[i][2]};
        double roll = (*roverPath6)[i][3];
        double pitch = (*roverPath6)[i][4];
        double yaw = (*roverPath6)[i][5];

        std::vector<std::vector<double>> TW2BCS
            = dot(getTraslation(pos),
                  dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        for (int j = wristPath6->size() - 1; j > -1; j--)
        {
            pos = {
                (*wristPath6)[j][0], (*wristPath6)[j][1], (*wristPath6)[j][2]};
            roll = (*wristPath6)[j][3];
            pitch = (*wristPath6)[j][4];
            yaw = (*wristPath6)[j][5];

            std::vector<std::vector<double>> TW2Wrist
                = dot(getTraslation(pos),
                      dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

            std::vector<std::vector<double>> TBCS2Wrist
                = dot(getInverse(&TW2BCS), TW2Wrist);

            pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};
            std::vector<double> basePos = {0, 0, sherpa_tt_arm->d0};
            double dist = getDist3(basePos, pos);

            if (TBCS2Wrist[0][3] < horizonDistance && dist < sherpa_tt_arm->maxArmOptimalDistance)
            {
                (*pathsAssignment)[i] = j;
                break;
            }
        }
    }

    for (int i = pathsAssignment->size() - 1; i > 0; i--)
        if ((*pathsAssignment)[i] < (*pathsAssignment)[i - 1])
            (*pathsAssignment)[i - 1] = (*pathsAssignment)[i];

    (*pathsAssignment)[0] = 0;
    (*pathsAssignment)[roverPath6->size() - 1] = wristPath6->size() - 1;
}

void ArmPlanner::computeWaypointInterpolation(
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
            std::vector<base::Waypoint> newWaypoints = getLinearInterpolation(
                (*newRoverPath)[i - 1], (*newRoverPath)[i], diff - 1);
            newRoverPath->insert(newRoverPath->begin() + i,
                                 newWaypoints.begin(),
                                 newWaypoints.end());
            for (int j = 0; j < diff - 1; j++)
                newAssignment->insert(newAssignment->begin() + i + j,
                                      (*newAssignment)[i - 1] + j + 1);
            i += diff - 1;
        }
        i++;
    }
}

std::vector<base::Waypoint> ArmPlanner::getLinearInterpolation(
    base::Waypoint waypoint0,
    base::Waypoint waypoint1,
    int numberIntWaypoints)
{
    double x0 = waypoint0.position[0];
    double y0 = waypoint0.position[1];
    double yaw0 = waypoint0.heading;

    double x1 = waypoint1.position[0];
    double y1 = waypoint1.position[1];
    double yaw1 = waypoint1.heading;

    if (yaw1 - yaw0 > pi)
        yaw1 -= 2*pi;
    else if (yaw1 - yaw0 < -pi)
        yaw1 += 2*pi;

    double l;
    std::vector<double> xi(10002, 0);
    std::vector<double> yi(10002, 0);
    std::vector<double> heading(10002, 0);
    std::vector<double> laccum(10002, 0);

    xi[0] = x0;
    yi[0] = y0;
    heading[0] = yaw0;

    for (int i = 1; i < 10002; i++)
    {
        xi[i] = x0 + i * (x1 - x0) / 10001;
        yi[i] = y0 + i * (y1 - y0) / 10001;
        heading[i] = yaw0 + i * (yaw1 - yaw0) / 10001;
        
        while (heading[i] > pi)
            heading[i] -= 2 * pi;
        while (heading[i] < -pi)
            heading[i] += 2 * pi;

        l = sqrt((xi[i] - xi[i - 1]) * (xi[i] - xi[i - 1])
                 + (yi[i] - yi[i - 1]) * (yi[i] - yi[i - 1]));
        laccum[i] = laccum[i - 1] + l;
    }
    double d = laccum[10001] / (numberIntWaypoints + 1);
    
    std::vector<base::Waypoint> newWaypoints(numberIntWaypoints);
    for (int i = 0; i < numberIntWaypoints; i++)
        for (int j = 0; j < 10002; j++)
            if (laccum[j] > (i + 1) * d)
            {
                newWaypoints[i].position[0] = xi[j];
                newWaypoints[i].position[1] = yi[j];
                newWaypoints[i].position[2]
                    = (waypoint0.position[2] + waypoint1.position[2]) / 2;
                newWaypoints[i].heading = heading[j];

                break;
            }

    return newWaypoints;
}

std::vector<base::Waypoint> ArmPlanner::getCubicInterpolation(
    base::Waypoint waypoint0,
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
    std::vector<double> heading(10002, 0);
    std::vector<double> laccum(10002, 0);

    xi[0] = x0;
    yi[0] = y0;
    heading[0] = yaw0;
    for (int i = 1; i < 10002; i++)
    {
        xi[i] = x0 + i * (x1 - x0) / 10001;
        yi[i] = a * xi[i] * xi[i] * xi[i] + b * xi[i] * xi[i] + c * xi[i] + d;
        heading[i] = yaw0 + i * (yaw1 - yaw0) / 10001;
        
        while (heading[i] > pi)
            heading[i] -= 2 * pi;
        while (heading[i] < -pi)
            heading[i] += 2 * pi;

        l = sqrt((xi[i] - xi[i - 1]) * (xi[i] - xi[i - 1])
                 + (yi[i] - yi[i - 1]) * (yi[i] - yi[i - 1]));
        laccum[i] = laccum[i - 1] + l;
    }
    double dist = laccum[10001] / (numberIntWaypoints + 1);

    std::vector<base::Waypoint> newWaypoints(numberIntWaypoints);
    for (int i = 0; i < numberIntWaypoints; i++)
    {
        for (int j = 0; j < 10002; j++)
            if (laccum[j] > (i + 1) * dist)
            {
                newWaypoints[i].position[0] = xi[j];
                newWaypoints[i].position[1] = yi[j];
                newWaypoints[i].position[2]
                    = (waypoint0.position[2] + waypoint1.position[2]) / 2;
                newWaypoints[i].heading = heading[j];
                
                break;
            }
    }
    return newWaypoints;
}

double ArmPlanner::getDist3(std::vector<double> a, std::vector<double> b)
{
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2)
                + pow(a[2] - b[2], 2));
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

std::vector<double> ArmPlanner::getGaussSmoothen(std::vector<double> values,
                                                 double sigma,
                                                 int samples)
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
