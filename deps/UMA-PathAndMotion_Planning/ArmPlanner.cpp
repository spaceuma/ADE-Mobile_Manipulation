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
// Authors: Gonzalo J. Paz Delgado, J. Ricardo Sánchez Ibáñez, Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#include "ArmPlanner.h"
#include <fstream>
#include <iostream>
#include <math.h>

#define pi 3.14159265359

using namespace KinematicModel_lib;
using namespace ArmPlanner_lib;

ArmPlanner::ArmPlanner(std::string s_data_path_m, bool _approach, int _deployment)
{
    sherpa_tt_arm = new Manipulator(s_data_path_m);

    approach = _approach;
    deployment = _deployment;

    switch(deployment)
    {
        case 0:    // END
            horizonDistance = MIN_HORIZON;
            varyingHorizon = false;
            break;
        case 1:    // TRAJECTORY
            varyingHorizon = true;
            break;
        case 2:    // BEGINNING:
            horizonDistance = MAX_HORIZON;
            varyingHorizon = false;
            break;
        default:
            break;
    }
}
ArmPlanner::~ArmPlanner()
{
    ;
}

void ArmPlanner::setApproach(bool _approach)
{
    approach = _approach;
}

bool ArmPlanner::setDeployment(unsigned int _deployment)
{
    deployment = _deployment;

    switch(deployment)
    {
        case 0:    // END
            horizonDistance = MIN_HORIZON;
            return true;
            break;
        case 1:    // TRAJECTORY
            varyingHorizon = true;
            return true;
            break;
        case 2:    // BEGINNING
            horizonDistance = MAX_HORIZON;
            return true;
            break;
        default:
            return false;
            break;
    }
}

bool ArmPlanner::planArmMotion(std::vector<base::Waypoint> * roverPath,
                               const std::vector<std::vector<double>> * _DEM,
                               double _mapResolution,
                               double _zResolution,
                               base::Waypoint samplePos,
                               std::vector<std::vector<double>> * armJoints,
                               CollisionDetector * p_collision_detector)
{
    this->mapResolution = _mapResolution;
    this->zResolution = _zResolution;
    this->DEM = _DEM;

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m "
                 "Computing the Arm Motion Profile with a rover path of "
              << roverPath->size() << " waypoints" << std::endl;

    // Rover z coordinate and heading computation
    (*roverPath)[roverPath->size() - 1].heading =
        atan2(samplePos.position[1] - (*roverPath)[roverPath->size() - 1].position[1],
              samplePos.position[0] - (*roverPath)[roverPath->size() - 1].position[0]);
    roverPath6 = new std::vector<std::vector<double>>;
    roverPath6->resize(roverPath->size(), std::vector<double>(6));

    wristPath6 = new std::vector<std::vector<double>>;

    std::vector<double> heading, smoothedHeading;

    (*roverPath6)[0][0] = (*roverPath)[0].position[0];
    (*roverPath6)[0][1] = (*roverPath)[0].position[1];
    (*roverPath6)[0][2]
        //= heightGround2BCS;
        = (*DEM)[(int)((*roverPath)[0].position[1] / mapResolution + 0.5)]
                [(int)((*roverPath)[0].position[0] / mapResolution + 0.5)] +
          heightGround2BCS;

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m The "
                 "interpolated value from DEM is "
              << (*DEM)[(int)((*roverPath)[0].position[1] / mapResolution + 0.5)]
                       [(int)((*roverPath)[0].position[0] / mapResolution + 0.5)]
              << std::endl;

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m The "
                 "heightGround2BCS constant is "
              << heightGround2BCS << std::endl;

    (*roverPath6)[0][3] = 0;
    (*roverPath6)[0][4] = 0;
    (*roverPath6)[0][5] = (*roverPath)[0].heading;

    heading.push_back((*roverPath)[0].heading);

    double offset = 0;
    for(int i = 1; i < roverPath->size(); i++)
    {
        (*roverPath6)[i][0] = (*roverPath)[i].position[0];
        (*roverPath6)[i][1] = (*roverPath)[i].position[1];
        (*roverPath6)[i][2]
            //	= heightGround2BCS;
            = (*DEM)[(int)((*roverPath)[i].position[1] / mapResolution + 0.5)]
                    [(int)((*roverPath)[i].position[0] / mapResolution + 0.5)] +
              heightGround2BCS;
        (*roverPath6)[i][3] = 0;
        (*roverPath6)[i][4] = 0;
        (*roverPath6)[i][5] = (*roverPath)[i].heading;
        if((*roverPath)[i].heading - (*roverPath)[i - 1].heading > pi)
            offset -= 2 * pi;
        else if((*roverPath)[i].heading - (*roverPath)[i - 1].heading < -pi)
            offset += 2 * pi;
        heading.push_back((*roverPath)[i].heading + offset);
    }

    double sigma = 2;
    int samples = 5;
    if(roverPath->size() > samples * 2 + 1)
    {
        smoothedHeading = getGaussSmoothen(heading, sigma, samples);
        for(int i = samples / 2 + 1; i < roverPath->size() - samples / 2 - 1; i++)
        {
            (*roverPath6)[i][5] = smoothedHeading[i];
            while((*roverPath6)[i][5] > pi)
                (*roverPath6)[i][5] -= 2 * pi;
            while((*roverPath6)[i][5] < -pi)
                (*roverPath6)[i][5] += 2 * pi;
        }
    }

    // Initial arm pos computation
    std::vector<std::vector<double>> initialBCS2Wrist =
        sherpa_tt_arm->getWristTransform(sherpa_tt_arm->initialConfiguration);
    std::vector<double> roverIniPos{(*roverPath6)[0][0], (*roverPath6)[0][1], (*roverPath6)[0][2]};
    std::vector<std::vector<double>> world2Wrist =
        dot(dot(getTraslation(roverIniPos), getZrot((*roverPath6)[0][5])), initialBCS2Wrist);

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m "
                 "Rover Initial Position is ("
              << roverIniPos[0] << ", " << roverIniPos[1] << ", " << roverIniPos[2] << ") "
              << std::endl;

    base::Waypoint iniPos;
    iniPos.position[0] = world2Wrist[0][3];
    iniPos.position[1] = world2Wrist[1][3];
    iniPos.position[2] = world2Wrist[2][3];    // TODO - Check this...

    // std::cout << " \033[34m[----------] [ArmPlanner::planArmMotion()]\033[0m
    // [ArmPlanner::planArmMotion()] Elevation of Wrist is " <<
    // iniPos.position[2] << std::endl;

    // The sample position is slightly changed to fit in the reachability area
    // of the manipulator
    samplePos.position[2] = (*DEM)[(int)(samplePos.position[1] / mapResolution + 0.5)]
                                  [(int)(samplePos.position[0] / mapResolution + 0.5)] +
                            fetchingZDistance + sherpa_tt_arm->d6;    // Adding d6 to find wrist pos

    std::vector<double> pos{(*roverPath6)[roverPath6->size() - 1][0],
                            (*roverPath6)[roverPath6->size() - 1][1],
                            (*roverPath6)[roverPath6->size() - 1][2]};
    double roll = (*roverPath6)[roverPath6->size() - 1][3];
    double pitch = (*roverPath6)[roverPath6->size() - 1][4];
    double yaw = (*roverPath6)[roverPath6->size() - 1][5];

    std::vector<std::vector<double>> TW2BCS
        /*= dot(getTraslation(pos),
              dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));*/
        = dot(getTraslation(pos), getZrot(yaw));

    // std::cout << " \033[34m[----------]\033[0m [ArmPlanner::planArmMotion()]
    // Elevation of Base is " << pos[2] << std::endl; std::cout << "
    // \033[34m[----------]\033[0m [ArmPlanner::planArmMotion()] Elevation of
    // sample is " << samplePos.position[2] << std::endl;

    pos = {samplePos.position[0], samplePos.position[1], samplePos.position[2]};

    std::vector<std::vector<double>> TW2Sample = getTraslation(pos);

    std::vector<std::vector<double>> TBCS2Sample = dot(getInverse(&TW2BCS), TW2Sample);

    TBCS2Sample[0][3] = 1.4;    // 0.969;
    TBCS2Sample[1][3] = 0.7;    // 0.384;//optimalLeftDeviation;
    // TBCS2Sample[2][3] = 0.65;//heightGround2BCS -
    // sherpa_tt_arm->d6;//optimalLeftDeviation;

    TBCS2Sample[0][3] = 1.4;     // 0.969;
    TBCS2Sample[1][3] = 0.7;     // 0.384;//optimalLeftDeviation;
    TBCS2Sample[2][3] = 0.65;    // heightGround2BCS - sherpa_tt_arm->d6;//optimalLeftDeviation;

    /*
        TBCS2Sample[0][3] = 0.8;
        TBCS2Sample[1][3] += 1.25;//optimalLeftDeviation;
        TBCS2Sample[2][3] = 0.75;//optimalLeftDeviation;
     */

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m "
                 "Relative last waypoint to sample position = "
              << TBCS2Sample[0][3] << ", " << TBCS2Sample[1][3] << ", " << TBCS2Sample[2][3]
              << std::endl;

    std::cout << "Relative position sample - last waypoint = " << TBCS2Sample[0][3] << ", "
              << TBCS2Sample[1][3] << ", " << TBCS2Sample[2][3] << std::endl;

    TW2Sample = dot(TW2BCS, TBCS2Sample);

    samplePos.position[0] = TW2Sample[0][3];
    samplePos.position[1] = TW2Sample[1][3];
    samplePos.position[2] = TW2Sample[2][3];

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m New "
                 "Relative Elevation of sample is "
              << samplePos.position[2] << std::endl;

    // Cost map 3D computation
    int n = DEM->size();
    int m = (*DEM)[0].size();

    double maxz = 0;
    for(int i = 0; i < DEM->size(); i++)
    {
        for(int j = 0; j < (*DEM)[0].size(); j++)
        {
            if(((*DEM)[i][j] > maxz) && ((*DEM)[i][j] != INFINITY))
            {
                maxz = (*DEM)[i][j];
            }
        }
    }

    int l = (int)((maxz + heightGround2BCS + sherpa_tt_arm->maxZArm) / zResolution + 0.5);

    volume_cost_map = new std::vector<std::vector<std::vector<double>>>;
    volume_cost_map->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < m; j++)
        {
            for(int k = 0; k < l; k++)
            {
                (*volume_cost_map)[i][j][k] = INFINITY;
            }
        }
    }

    clock_t init = clock();

    // Smoothing heading of the rover path by inserting new waypoints
    double headingThreshold = M_PI / 12;
    smoothRoverPathHeading(headingThreshold);

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m "
                 "Reference base path heading is smoothed"
              << std::endl;

    // Generating the reachability tunnel surrounding the rover path
    unsigned int ui_noinf_nodes = generateTunnel(iniPos, samplePos, volume_cost_map);
    clock_t endt = clock();
    double t = double(endt - init) / CLOCKS_PER_SEC;

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m 3D "
                 "Cost Tunnel is generated"
              << std::endl;
    std::cout << "[MM] \033[35m[----------] "
                 "[ArmPlanner::planArmMotion()]\033[0m Tunnel uses "
              << ui_noinf_nodes << " / " << n * m * l << " nodes ("
              << ((double)ui_noinf_nodes / ((double)n * m * l)) << " % )" << std::endl;

    // End effector path planning
    FastMarching_lib::FastMarching3D pathPlanner3D;
    std::vector<base::Waypoint> * wristPath = new std::vector<base::Waypoint>;

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m "
                 "Starting 3D Wrist Path Planning"
              << std::endl;
    clock_t inip = clock();
    if(!pathPlanner3D.planPath(volume_cost_map,
                               mapResolution,
                               zResolution,
                               iniPos,
                               samplePos,
                               wristPath,
                               ui_noinf_nodes))
    {
        std::cout << "[MM] \033[35m[--WARNING-] [ArmPlanner::planArmMotion()]\033[0m "
                     "Something went wrong with 3D Wrist Path Planning, debug data:"
                  << std::endl;
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planArmMotion()]\033[0m  - IniPos = ("
                  << iniPos.position[0] << ", " << iniPos.position[1] << ", " << iniPos.position[2]
                  << ")" << std::endl;
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planArmMotion()]\033[0m  - samplePos = ("
                  << samplePos.position[0] << ", " << samplePos.position[1] << ", "
                  << samplePos.position[2] << ")" << std::endl;
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planArmMotion()]\033[0m  - mapResolution = "
                  << mapResolution << std::endl;
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planArmMotion()]\033[0m  - zResolution = "
                  << zResolution << std::endl;
        return false;
    }
    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m 3D "
                 "Wrist Path computed with "
              << wristPath->size() << " waypoints" << std::endl;

    // Orientation (roll, pitch, yaw) of the end effector at each waypoint
    wristPath6->resize(wristPath->size(), std::vector<double>(6));
    for(int i = 0; i < wristPath->size(); i++)
    {
        (*wristPath6)[i][0] = (*wristPath)[i].position[0];
        (*wristPath6)[i][1] = (*wristPath)[i].position[1];
        (*wristPath6)[i][2] = (*wristPath)[i].position[2];

        /*(*wristPath6)[i][3]
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
    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m
    End effector orientation computed" << std::endl;


    // Paths inbetween assignment
    std::vector<int> *pathsAssignment = new std::vector<int>;
    if (!computeWaypointAssignment(pathsAssignment, p_collision_detector))
    {
        std::cout << "[MM] \033[35m[--ERROR!--]
    [ArmPlanner::planArmMotion()]\033[0m ERROR Paths Assignment unfeasible" <<
    std::endl; return false;
    }
    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m
    Paths Assignment done with " << pathsAssignment->size() << " assignments" <<
    std::endl;

    // Waypoint interpolation to smooth the movements of the arm joints
    interpolatedRoverPath = new std::vector<base::Waypoint>(roverPath6->size());
    std::vector<int> *interpolatedAssignment
        = new std::vector<int>(roverPath6->size());
    computeWaypointInterpolation(
        pathsAssignment, interpolatedRoverPath, interpolatedAssignment);

    std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m
    Arm Joints Motion Smoothing done, interpolated Rover path has " <<
    interpolatedRoverPath->size() << " waypoints"  << std::endl;

    // Computing inverse kinematics
    /*First, we compute the inverse kinematics of the wrist for all the path
    Then, according to the orientation commanded, which is relative, we compute
    the last three joints of the arm, using the transform of the wrist. */

        /*Obtaining the first config*/
        int wristInd = (*interpolatedAssignment)[0];
        std::vector<std::vector<double>> TW2Wrist(4, std::vector<double>(4));
        std::vector<std::vector<double>> TBCS2Wrist(4, std::vector<double>(4));

        double x = (*interpolatedRoverPath)[0].position[0];
        double y = (*interpolatedRoverPath)[0].position[1];
        double z = (*interpolatedRoverPath)[0].position[2];

        std::vector<double> position{x, y, z};
        roll = 0;
        pitch = 0;
        yaw = (*interpolatedRoverPath)[0].heading;

        TW2BCS =
            dot(getTraslation(position), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));
        position[0] = (*wristPath6)[wristInd][0];
        position[1] = (*wristPath6)[wristInd][1];
        position[2] = (*wristPath6)[wristInd][2];

        TW2Wrist = getTraslation(position);

        TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

        position[0] = TBCS2Wrist[0][3];
        position[1] = TBCS2Wrist[1][3];
        position[2] = TBCS2Wrist[2][3];

        std::vector<double> firstConfig;
        firstConfig = sherpa_tt_arm->getPositionJoints(position, 1, 1);

        roll = (*wristPath6)[wristInd][3];
        pitch = (*wristPath6)[wristInd][4];
        yaw = (*wristPath6)[wristInd][5];

        std::vector<double> wristJoints =
            sherpa_tt_arm->getWristJoints(firstConfig, sherpa_tt_arm->iniEEorientation);

        firstConfig.insert(firstConfig.end(), wristJoints.begin(), wristJoints.end());

        /*Obtaining the last config*/
        wristInd = (*interpolatedAssignment)[interpolatedRoverPath->size() - 1];

        x = (*interpolatedRoverPath)[interpolatedRoverPath->size() - 1].position[0];
        y = (*interpolatedRoverPath)[interpolatedRoverPath->size() - 1].position[1];
        z = (*interpolatedRoverPath)[interpolatedRoverPath->size() - 1].position[2];

        position[0] = x;
        position[1] = y;
        position[2] = z;
        roll = 0;
        pitch = 0;
        yaw = (*interpolatedRoverPath)[interpolatedRoverPath->size() - 1].heading;

        TW2BCS =
            dot(getTraslation(position), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));
        position[0] = (*wristPath6)[wristInd][0];
        position[1] = (*wristPath6)[wristInd][1];
        position[2] = (*wristPath6)[wristInd][2];

        TW2Wrist = getTraslation(position);

        TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

        position[0] = TBCS2Wrist[0][3];
        position[1] = TBCS2Wrist[1][3];
        position[2] = TBCS2Wrist[2][3];

        std::vector<double> lastConfig;
        lastConfig = sherpa_tt_arm->getPositionJoints(position, 1, 1);

        wristJoints = sherpa_tt_arm->getWristJoints(lastConfig, finalEEorientation);

        lastConfig.insert(lastConfig.end(), wristJoints.begin(), wristJoints.end());

        double d_previousconfig3;
        for(int i = 0; i < interpolatedRoverPath->size(); i++)
        {
            wristInd = (*interpolatedAssignment)[i];

            x = (*interpolatedRoverPath)[i].position[0];
            y = (*interpolatedRoverPath)[i].position[1];
            z = (*interpolatedRoverPath)[i].position[2];
            position[0] = x;
            position[1] = y;
            position[2] = z;
            roll = 0;
            pitch = 0;
            yaw = (*interpolatedRoverPath)[i].heading;

            TW2BCS =
                dot(getTraslation(position), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

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
                config = sherpa_tt_arm->getPositionJoints(position, 1, 1, 0.0);
            }
            catch(std::exception & e)
            {
                std::cout << "[MM] \033[35m[----------] "
                             "[ArmPlanner::planArmMotion()]\033[0m EXCEPTION "
                             "inverse kinematic computation failed"
                          << std::endl;
                std::cout << "Arm Planner library: debug info" << std::endl;
                std::cout << " - function: sherpa_tt_arm->getPositionJoints()" << std::endl;
                std::cout << " - iteration index = " << i << std::endl;
                std::cout << " - position = " << position[0] << ", " << position[1] << ", "
                          << position[2] << std::endl;
                return false;
            }
            // std::cout << "Position is computed" << std::endl;
            std::vector<double> wristJoints;
            double d_config4 = std::max(-config[2], -1.57);

            /*        if (config[0] < 0.65)
                {
                        wristJoints
                            = { std::max(3.1416,std::max(d_previousconfig3, 1.57
               + 1.57 * (0.65 - config[0])/(0.65 - 0.377))), -1.57, -2.7 };
                }
                else */
            if(config[0] < 1.57)
            {
                wristJoints = {1.57, -1.4, -2.7};
            }
            else
            {
                wristJoints = {3.1416 - config[0], -1.4, -2.7};
            }
            d_previousconfig3 = config[3];

            /*
                if (config[0]<0.4)
                    {
                            config = sherpa_tt_arm->getPositionJoints(position,
               1, 1, 0.0);
                    }
                    catch(std::exception &e)
                    {
                            std::cout << " \033[35m[----------]
                    [ArmPlanner::planArmMotion()]\033[0m EXCEPTION inverse
               kinematic computation failed" << std::endl; std::cout << "Arm
               Planner library: debug info" << std::endl; std::cout << " -
               function: sherpa_tt_arm->getPositionJoints()" << std::endl;
               std::cout << " - iteration index = " << i << std::endl; std::cout
               << " - position = " << position[0] << ", " << position[1] << ", "
               << position[2] << std::endl; return false;
                    }
                    //std::cout << "Position is computed" << std::endl;
                        std::vector<double> wristJoints;
                    //double d_config4 = std::max(-config[2],-1.57);

                        if (config[0] < 0.65)
                    {
                            wristJoints
                                = {
               std::max(3.1416,std::max(d_previousconfig3, 1.57 + 1.57
                    * (0.65 - config[0])/(0.65 - 0.377))), -1.57, -2.7 };
                    }
                    else
                    if (config[0] < 1.57)
                    {
                            wristJoints
                                = { 1.57, -1.4, -2.7 };
                    }
                    else
                    {
                            wristJoints
                                = { 3.1416 - config[0], -1.4, -2.7 };
                    }
                        d_previousconfig3 = config[3];


                    if (config[0]<0.4)
                        {
                            //d_config4 = (0.7 - config[0]) * 5.0;
                            wristJoints
                                = { 0.0, 0.0, -2.7 };
                        }
                    else
                    {
                            wristJoints
                                = { 0.0, d_config4, -2.7 };
                    }

                        config.insert(config.end(), wristJoints.begin(),
                    wristJoints.end());*/
            try
            {
                config = sherpa_tt_arm->getPositionJoints(position, 1, 1);

                for(int j = 3; j < 6; j++)
                {
                    double angleDiff = lastConfig[j] - firstConfig[j];
                    if(firstConfig[j] > lastConfig[j])
                    {
                        if(abs(angleDiff) < 2 * M_PI - abs(angleDiff))
                        {
                            config.push_back(firstConfig[j] -
                                             i * abs(angleDiff) /
                                                 (interpolatedRoverPath->size() - 1));
                        }
                        else
                        {
                            config.push_back(firstConfig[j] +
                                             i * (2 * M_PI - abs(angleDiff)) /
                                                 (interpolatedRoverPath->size() - 1));
                        }
                    }
                    else
                    {
                        if(abs(angleDiff) > 2 * M_PI - abs(angleDiff))
                        {
                            config.push_back(firstConfig[j] -
                                             i * (2 * M_PI - abs(angleDiff)) /
                                                 (interpolatedRoverPath->size() - 1));
                        }
                        else
                        {
                            config.push_back(firstConfig[j] +
                                             i * abs(angleDiff) /
                                                 (interpolatedRoverPath->size() - 1));
                        }
                    }
                }

                for(int j = 0; j < config.size(); j++)
                {
                    while(config[j] > M_PI)
                        config[j] -= 2 * M_PI;
                    while(config[j] < -M_PI)
                        config[j] += 2 * M_PI;
                }
            }
            catch(std::exception & e)
            {
                std::cout << "Inverse kinematics failed at waypoint " << i << "!" << std::endl;
                return false;
            }
            // if(config == std::vector<double>(1,0))
            // armJoints->push_back((*armJoints)[armJoints->size()-1]);
            // else
            if(p_collision_detector->isColliding(config))
            {
                std::cout << "[MM] \033[35m[--ERROR!--] "
                             "[ArmPlanner::planArmMotion()]\033[0m ERROR at sample "
                          << i << std::endl;
                std::cout << " Joints = " << config[0];
                std::cout << " " << config[1];
                std::cout << " " << config[2];
                std::cout << " " << config[3];
                std::cout << " " << config[4];
                std::cout << " " << config[5] << std::endl;
                * / config = (*armJoints)[armJoints->size() - 1];
                // return false;
            }
            armJoints->push_back(config);
        }

        std::cout << "[MM] \033[35m[----------] [ArmPlanner::planArmMotion()]\033[0m "
                     "Arm Profile computed with "
                  << armJoints->size() << " samples" << std::endl;

        (*roverPath) = (*interpolatedRoverPath);

        clock_t endp = clock();
        double tp = double(endp - inip) / CLOCKS_PER_SEC;

        std::cout << "[MM] \033[1;35m[----------] "
                     "[ArmPlanner::planArmMotion()]\033[0m Done Planning Arm "
                     "Motion in "
                  << tp << " seconds" << std::endl;

        return true;
    }

    bool ArmPlanner::planAtomicOperation(double d_mapResolution_m,
                                         double _zResolution,
                                         std::vector<double> initialArmConfiguration,
                                         std::vector<double> goalArmConfiguration,
                                         std::vector<std::vector<double>> * armJoints,
                                         std::vector<double> * timeProfile,
                                         int mode)
    {
        this->mapResolution = d_mapResolution_m;
        this->zResolution = _zResolution;

        // Rover z coordinate and heading computation
        wristPath6 = new std::vector<std::vector<double>>;

        // Initial arm pos computation
        std::vector<std::vector<double>> TBCS2Wrist =
            sherpa_tt_arm->getWristTransform(initialArmConfiguration);

        bool isTunnelPermisive = false;

        std::vector<double> pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};
        if(sherpa_tt_arm->isReachable(pos, 0) == 1)
        {
            // std::cout<<"WARNING[planAtomicOperation]: initial arm
            // configuration is outside security area\n";
            isTunnelPermisive = true;
        }
        else if(sherpa_tt_arm->isReachable(pos, 0) == 0)
        {
            std::cout << "ERROR[planAtomicOperation]: initial arm configuration is "
                         "colliding or unreachable\n";
            return false;
        }

        std::vector<double> relativePos = sherpa_tt_arm->getRelativePosition(pos);

        base::Waypoint iniPos;
        iniPos.position[0] = relativePos[0];
        iniPos.position[1] = relativePos[1];
        iniPos.position[2] = relativePos[2];

        // Final arm pos computation
        TBCS2Wrist = sherpa_tt_arm->getWristTransform(goalArmConfiguration);

        pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};
        if(sherpa_tt_arm->isReachable(pos, 0) == 1)
        {
            // std::cout<<"WARNING[planAtomicOperation]: goal arm configuration
            // is outside security area\n";
            isTunnelPermisive = true;
        }
        else if(sherpa_tt_arm->isReachable(pos, 0) == 0)
        {
            std::cout << "ERROR[planAtomicOperation]: goal arm configuration will "
                         "lead to collision or is unreachable\n";
            return false;
        }

        relativePos = sherpa_tt_arm->getRelativePosition(pos);

        base::Waypoint goalPos;
        goalPos.position[0] = relativePos[0];
        goalPos.position[1] = relativePos[1];
        goalPos.position[2] = relativePos[2];

        // Rover pose should be in the center of the new volume cost map
        pos = {0, 0, 0};
        relativePos = sherpa_tt_arm->getRelativePosition(pos);
        std::vector<double> roverPose6 = {relativePos[0], relativePos[1], relativePos[2], 0, 0, 0};

        std::vector<std::vector<double>> TW2BCS = getTraslation(relativePos);

        // Cost map 3D computation
        std::vector<double> sizes = sherpa_tt_arm->getReachabilityMapSize();
        int n = round(sizes[0] / mapResolution);
        int m = round(sizes[1] / mapResolution);
        int l = round(sizes[2] / zResolution);

        volume_cost_map = new std::vector<std::vector<std::vector<double>>>;
        volume_cost_map->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

        for(int i = 0; i < n; i++)
            for(int j = 0; j < m; j++)
                for(int k = 0; k < l; k++)
                    (*volume_cost_map)[i][j][k] = INFINITY;

        // Generating the reachability tunnel surrounding the rover path
        generateReachabilityTunnel(
            iniPos, goalPos, roverPose6, isTunnelPermisive, volume_cost_map, mode);

        // End effector path planning
        FastMarching_lib::FastMarching3D * pathPlanner3D =
            new FastMarching_lib::FastMarching3D(0.25);
        std::vector<base::Waypoint> * wristPath = new std::vector<base::Waypoint>;

        clock_t inip = clock();
        pathPlanner3D->planPath(volume_cost_map,
                                mapResolution,
                                zResolution,
                                iniPos,
                                goalPos,
                                wristPath,
                                10000);    // TODO: Change this

        wristPath6->resize(wristPath->size(), std::vector<double>(6));

        for(int i = 0; i < wristPath->size(); i++)
        {
            (*wristPath6)[i][0] = (*wristPath)[i].position[0];
            (*wristPath6)[i][1] = (*wristPath)[i].position[1];
            (*wristPath6)[i][2] = (*wristPath)[i].position[2];
        }
        // Computing inverse kinematics

        for(int i = 0; i < wristPath->size(); i++)
        {
            std::vector<std::vector<double>> TW2Wrist(4, std::vector<double>(4));
            std::vector<std::vector<double>> TBCS2Wrist(4, std::vector<double>(4));

            pos[0] = (*wristPath)[i].position[0];
            pos[1] = (*wristPath)[i].position[1];
            pos[2] = (*wristPath)[i].position[2];

            TW2Wrist = getTraslation(pos);

            TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

            pos[0] = TBCS2Wrist[0][3];
            pos[1] = TBCS2Wrist[1][3];
            pos[2] = TBCS2Wrist[2][3];
            std::vector<double> config;

            try
            {
                config = sherpa_tt_arm->getPositionJoints(pos, 1, 1);
                double joint4 =
                    initialArmConfiguration[3] +
                    i * (goalArmConfiguration[3] - initialArmConfiguration[3]) / wristPath->size();
                double joint5 =
                    initialArmConfiguration[4] +
                    i * (goalArmConfiguration[4] - initialArmConfiguration[4]) / wristPath->size();
                double joint6 =
                    initialArmConfiguration[5] +
                    i * (goalArmConfiguration[5] - initialArmConfiguration[5]) / wristPath->size();

                config.push_back(joint4);
                config.push_back(joint5);
                config.push_back(joint6);
            }
            catch(std::exception & e)
            {
                std::cout << "Inverse kinematics failed at waypoint " << i << "!" << std::endl;
                return false;
            }
            armJoints->push_back(config);
        }

        for(int i = 0; i < wristPath->size(); i++)
            (*wristPath6)[i] = sherpa_tt_arm->getAbsolutePosition((*wristPath6)[i]);

        (*timeProfile) = getTimeProfile(armJoints);
        return true;
    }

    bool ArmPlanner::planAtomicOperation(double d_mapResolution_m,
                                         double _zResolution,
                                         std::vector<double> initialArmConfiguration,
                                         base::Waypoint goalEEPosition,
                                         std::vector<double> goalEEOrientation,
                                         std::vector<std::vector<double>> * armJoints,
                                         std::vector<double> * timeProfile)
    {
        this->mapResolution = d_mapResolution_m;
        this->zResolution = _zResolution;

        // Rover z coordinate and heading computation
        wristPath6 = new std::vector<std::vector<double>>;

        // Initial arm pos computation
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planAtomicOperation()]\033[0m Getting "
                     "Transformation Matrix from rover base to goal"
                  << std::endl;
        std::vector<std::vector<double>> TBCS2Wrist =
            sherpa_tt_arm->getWristTransform(initialArmConfiguration);

        bool isTunnelPermisive = false;

        std::vector<double> pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};

        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planAtomicOperation()]\033[0m Checking "
                     "current arm configuration"
                  << std::endl;

        if(sherpa_tt_arm->isReachable(pos, 0) == 1)
        {
            // std::cout<<"WARNING[planAtomicOperation]: initial arm
            // configuration is outside security area\n";
            isTunnelPermisive = true;
        }
        else if(sherpa_tt_arm->isReachable(pos, 0) == 0)
        {
            std::cout << "[MM] \033[1;31m[----------] "
                         "[ArmPlanner::planAtomicOperation()]\033[0m Initial "
                         "arm configuration is either colliding or unreachable"
                      << std::endl;
            return false;
        }

        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planAtomicOperation()]\033[0m Current arm "
                     "configuration is valid"
                  << std::endl;

        std::vector<double> relativePos = sherpa_tt_arm->getRelativePosition(pos);

        base::Waypoint iniPos;
        iniPos.position[0] = relativePos[0];
        iniPos.position[1] = relativePos[1];
        iniPos.position[2] = relativePos[2];

        // Final arm pos computation
        std::vector<double> goalPosition = {
            goalEEPosition.position[0], goalEEPosition.position[1], goalEEPosition.position[2]};
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planAtomicOperation()]\033[0m Getting "
                     "configuration to goal position"
                  << std::endl;
        std::vector<double> goalArmConfiguration = {0, 0, 0, 0, 0, 0};
        try
        {
            goalArmConfiguration =
                sherpa_tt_arm->getManipJoints(goalPosition, goalEEOrientation, 1, 1);
        }
        catch(std::exception & e)
        {
            std::cout << "[MM] \033[1;31m[----------] "
                         "[ArmPlanner::planAtomicOperation()]\033[0m Goal arm "
                         "configuration is unfeasible"
                      << std::endl;
            return false;
        }
        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planAtomicOperation()]\033[0m Getting "
                     "Transformation Matrix from rover base to goal"
                  << std::endl;
        TBCS2Wrist = sherpa_tt_arm->getWristTransform(goalArmConfiguration);

        pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};

        if(sherpa_tt_arm->isReachable(pos, 0) == 1)
        {
            // std::cout<<"WARNING[planAtomicOperation]: goal arm configuration
            // is outside security area\n";
            isTunnelPermisive = true;
        }
        else if(sherpa_tt_arm->isReachable(pos, 0) == 0)
        {
            std::cout << "[MM] \033[1;31m[----------] "
                         "[ArmPlanner::planAtomicOperation()]\033[0m Goal arm "
                         "configuration is either colliding or unreachable"
                      << std::endl;
            return false;
        }

        std::cout << "[MM] \033[35m[----------] "
                     "[ArmPlanner::planAtomicOperation()]\033[0m Both origin "
                     "and goal arm configurations checked"
                  << std::endl;

        relativePos = sherpa_tt_arm->getRelativePosition(pos);

        base::Waypoint goalPos;
        goalPos.position[0] = relativePos[0];
        goalPos.position[1] = relativePos[1];
        goalPos.position[2] = relativePos[2];

        // Rover pose should be in the center of the new volume cost map
        pos = {0, 0, 0};
        relativePos = sherpa_tt_arm->getRelativePosition(pos);
        std::vector<double> roverPose6 = {relativePos[0], relativePos[1], relativePos[2], 0, 0, 0};

        std::vector<std::vector<double>> TW2BCS = getTraslation(relativePos);

        // Cost map 3D computation
        std::vector<double> sizes = sherpa_tt_arm->getReachabilityMapSize();
        int n = round(sizes[0] / mapResolution);
        int m = round(sizes[1] / mapResolution);
        int l = round(sizes[2] / zResolution);

        volume_cost_map = new std::vector<std::vector<std::vector<double>>>;
        volume_cost_map->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

        for(int i = 0; i < n; i++)
            for(int j = 0; j < m; j++)
                for(int k = 0; k < l; k++)
                    (*volume_cost_map)[i][j][k] = INFINITY;

        // Generating the reachability tunnel surrounding the rover path
        generateReachabilityTunnel(iniPos, goalPos, roverPose6, isTunnelPermisive, volume_cost_map);

        // End effector path planning
        FastMarching_lib::FastMarching3D * pathPlanner3D =
            new FastMarching_lib::FastMarching3D(0.25);
        std::vector<base::Waypoint> * wristPath = new std::vector<base::Waypoint>;

        clock_t inip = clock();
        pathPlanner3D->planPath(volume_cost_map,
                                mapResolution,
                                zResolution,
                                iniPos,
                                goalPos,
                                wristPath,
                                10000);    // TODO: Change this

        wristPath6->resize(wristPath->size(), std::vector<double>(6));

        for(int i = 0; i < wristPath->size(); i++)
        {
            (*wristPath6)[i][0] = (*wristPath)[i].position[0];
            (*wristPath6)[i][1] = (*wristPath)[i].position[1];
            (*wristPath6)[i][2] = (*wristPath)[i].position[2];
        }
        // Computing inverse kinematics

        for(int i = 0; i < wristPath->size(); i++)
        {
            std::vector<std::vector<double>> TW2Wrist(4, std::vector<double>(4));
            std::vector<std::vector<double>> TBCS2Wrist(4, std::vector<double>(4));

            pos[0] = (*wristPath)[i].position[0];
            pos[1] = (*wristPath)[i].position[1];
            pos[2] = (*wristPath)[i].position[2];

            TW2Wrist = getTraslation(pos);

            TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

            pos[0] = TBCS2Wrist[0][3];
            pos[1] = TBCS2Wrist[1][3];
            pos[2] = TBCS2Wrist[2][3];
            std::vector<double> config;

            try
            {
                config = sherpa_tt_arm->getPositionJoints(pos, 1, 1);
                double joint4 =
                    initialArmConfiguration[3] +
                    i * (goalArmConfiguration[3] - initialArmConfiguration[3]) / wristPath->size();
                double joint5 =
                    initialArmConfiguration[4] +
                    i * (goalArmConfiguration[4] - initialArmConfiguration[4]) / wristPath->size();
                double joint6 =
                    initialArmConfiguration[5] +
                    i * (goalArmConfiguration[5] - initialArmConfiguration[5]) / wristPath->size();

                config.push_back(joint4);
                config.push_back(joint5);
                config.push_back(joint6);
            }
            catch(std::exception & e)
            {
                std::cout << "Inverse kinematics failed at waypoint " << i << "!" << std::endl;
                return false;
            }
            armJoints->push_back(config);
        }

        for(int i = 0; i < wristPath->size(); i++)
            (*wristPath6)[i] = sherpa_tt_arm->getAbsolutePosition((*wristPath6)[i]);

        (*timeProfile) = getTimeProfile(armJoints);
        return true;
    }

    bool ArmPlanner::planAtomicOperation(const std::vector<std::vector<double>> * _DEM,
                                         double _mapResolution,
                                         double _zResolution,
                                         base::Waypoint roverWaypoint,
                                         base::Waypoint initialEEPosition,
                                         base::Waypoint goalEEPosition,
                                         std::vector<std::vector<double>> * armJoints,
                                         std::vector<double> * timeProfile)
    {
        this->mapResolution = _mapResolution;
        this->zResolution = _zResolution;
        this->DEM = _DEM;

        // Rover z coordinate and heading computation
        std::vector<double> roverPose6(6);

        roverPose6[0] = roverWaypoint.position[0];
        roverPose6[1] = roverWaypoint.position[1];
        roverPose6[2] = (*DEM)[(int)(roverWaypoint.position[1] / mapResolution + 0.5)]
                              [(int)(roverWaypoint.position[0] / mapResolution + 0.5)] +
                        heightGround2BCS;
        roverPose6[3] = 0;
        roverPose6[4] = 0;
        roverPose6[5] = roverWaypoint.heading;

        roverPath6 = new std::vector<std::vector<double>>;
        roverPath6->push_back(roverPose6);

        // Initial arm pos computation
        std::vector<double> pos = {initialEEPosition.position[0],
                                   initialEEPosition.position[1],
                                   initialEEPosition.position[2]};
        std::vector<double> EE2Wrist = {0, 0, sherpa_tt_arm->d6};
        std::vector<std::vector<double>> TBCS2Wrist =
            dot(getTraslation(pos), getTraslation(EE2Wrist));

        bool isTunnelPermisive = false;

        pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};
        if(sherpa_tt_arm->isReachable(pos, 0) == 1)
        {
            // std::cout<<"WARNING[planAtomicOperation]: initial end effector
            // position is outside security area\n";
            isTunnelPermisive = true;
        }
        else if(sherpa_tt_arm->isReachable(pos, 0) == 0)
        {
            std::cout << "ERROR[planAtomicOperation]: initial end effector "
                         "position is colliding or unreachable\n";
            return false;
        }

        pos = {roverPose6[0], roverPose6[1], roverPose6[2]};
        double roll = roverPose6[3];
        double pitch = roverPose6[4];
        double yaw = roverPose6[5];

        std::vector<std::vector<double>> TW2BCS =
            dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        std::vector<std::vector<double>> TW2Wrist = dot(TW2BCS, TBCS2Wrist);

        base::Waypoint iniPos;
        iniPos.position[0] = TW2Wrist[0][3];
        iniPos.position[1] = TW2Wrist[1][3];
        iniPos.position[2] = TW2Wrist[2][3];

        // Final arm pos computation
        pos = {goalEEPosition.position[0], goalEEPosition.position[1], goalEEPosition.position[2]};
        TBCS2Wrist = dot(getTraslation(pos), getTraslation(EE2Wrist));

        pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};
        if(sherpa_tt_arm->isReachable(pos, 0) == 1)
        {
            // std::cout<<"WARNING[planAtomicOperation]: goal end effector
            // position is outside security area\n";
            isTunnelPermisive = true;
        }
        else if(sherpa_tt_arm->isReachable(pos, 0) == 0)
        {
            std::cout << "ERROR[planAtomicOperation]: goal end effector position "
                         "will lead to collision or is unreachable\n";
            return false;
        }

        TW2Wrist = dot(TW2BCS, TBCS2Wrist);

        base::Waypoint goalPos;
        goalPos.position[0] = TW2Wrist[0][3];
        goalPos.position[1] = TW2Wrist[1][3];
        goalPos.position[2] = TW2Wrist[2][3];

        // Cost map 3D computation
        int n = DEM->size();
        int m = (*DEM)[0].size();

        double maxz = 0;
        for(int i = 0; i < DEM->size(); i++)
            for(int j = 0; j < (*DEM)[0].size(); j++)
                if((*DEM)[i][j] > maxz) maxz = (*DEM)[i][j];

        int l = (int)((maxz + heightGround2BCS + sherpa_tt_arm->maxZArm) / zResolution + 0.5);

        volume_cost_map = new std::vector<std::vector<std::vector<double>>>;
        volume_cost_map->resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

        for(int i = 0; i < n; i++)
            for(int j = 0; j < m; j++)
                for(int k = 0; k < l; k++)
                    (*volume_cost_map)[i][j][k] = INFINITY;

        // Generating the reachability tunnel surrounding the rover path
        generateReachabilityTunnel(iniPos, goalPos, roverPose6, isTunnelPermisive, volume_cost_map);

        // End effector path planning
        FastMarching_lib::FastMarching3D * pathPlanner3D =
            new FastMarching_lib::FastMarching3D(0.25);
        std::vector<base::Waypoint> * wristPath = new std::vector<base::Waypoint>;

        clock_t inip = clock();
        pathPlanner3D->planPath(volume_cost_map,
                                mapResolution,
                                zResolution,
                                iniPos,
                                goalPos,
                                wristPath,
                                10000);    // TODO: Change this

        wristPath6->resize(wristPath->size(), std::vector<double>(6));

        for(int i = 0; i < wristPath->size(); i++)
        {
            (*wristPath6)[i][0] = (*wristPath)[i].position[0];
            (*wristPath6)[i][1] = (*wristPath)[i].position[1];
            (*wristPath6)[i][2] = (*wristPath)[i].position[2];
        }
        // Computing inverse kinematics

        for(int i = 0; i < wristPath->size(); i++)
        {
            std::vector<std::vector<double>> TW2Wrist(4, std::vector<double>(4));
            std::vector<std::vector<double>> TBCS2Wrist(4, std::vector<double>(4));

            pos[0] = (*wristPath)[i].position[0];
            pos[1] = (*wristPath)[i].position[1];
            pos[2] = (*wristPath)[i].position[2];

            TW2Wrist = getTraslation(pos);

            TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

            pos[0] = TBCS2Wrist[0][3];
            pos[1] = TBCS2Wrist[1][3];
            pos[2] = TBCS2Wrist[2][3];
            std::vector<double> config;

            try
            {
                config = sherpa_tt_arm->getPositionJoints(pos, 1, 1);

                double roll = 0;
                double pitch = pi;
                double yaw = 0;

                std::vector<double> orientation{roll, pitch, yaw};

                std::vector<double> wristJoints =
                    sherpa_tt_arm->getWristJoints(config, orientation);

                config.insert(config.end(), wristJoints.begin(), wristJoints.end());
            }
            catch(std::exception & e)
            {
                std::cout << "Inverse kinematics failed at waypoint " << i << "!" << std::endl;
                return false;
            }
            armJoints->push_back(config);
        }

        (*timeProfile) = getTimeProfile(armJoints);
        return true;
    }

    std::vector<std::vector<double>> * ArmPlanner::getWristPath()
    {
        return wristPath6;
    }

    std::vector<base::Waypoint> * ArmPlanner::getInterpolatedRoverPath()
    {
        return interpolatedRoverPath;
    }

    std::vector<std::vector<std::vector<double>>> * ArmPlanner::getVolumeCostMap()
    {
        return this->volume_cost_map;
    }

    unsigned int ArmPlanner::generateTunnel(base::Waypoint iniPos,
                                            base::Waypoint samplePos,
                                            std::vector<std::vector<std::vector<double>>> *
                                                costMap3D)
    {
        // Returns the number of non-infinite cost nodes

        int n = roverPath6->size();
        int sx = (*costMap3D).size();
        int sy = (*costMap3D)[0].size();
        int sz = (*costMap3D)[0][0].size();

        unsigned int ui_noninf_nodes = 0;

        std::vector<std::vector<std::vector<int>>> * tunnelLabel =
            new std::vector<std::vector<std::vector<int>>>;
        tunnelLabel->resize(sx, std::vector<std::vector<int>>(sy, std::vector<int>(sz, 2 * n)));

        std::vector<int> goal(3, 0);
        std::vector<int> start(3, 0);

        start[0] = (int)(iniPos.position[0] / mapResolution + 0.5);
        start[1] = (int)(iniPos.position[1] / mapResolution + 0.5);
        start[2] = (int)(iniPos.position[2] / zResolution + 0.5);
        (*costMap3D)[start[1]][start[0]][start[2]] = 1;
        (*tunnelLabel)[start[1]][start[0]][start[2]] = n;

        goal[0] = (int)(samplePos.position[0] / mapResolution + 0.5);
        goal[1] = (int)(samplePos.position[1] / mapResolution + 0.5);
        goal[2] = (int)(samplePos.position[2] / zResolution + 0.5);
        (*costMap3D)[goal[1]][goal[0]][goal[2]] = 1;
        (*tunnelLabel)[goal[1]][goal[0]][goal[2]] = n;

        std::vector<double> * minValues = sherpa_tt_arm->minValues;
        std::vector<double> * maxValues = sherpa_tt_arm->maxValues;

        double slope = 0.3 * approach;

        // Tunnel in the first waypoint
        if(varyingHorizon) horizonDistance = MIN_HORIZON;

        int tunnelSizeX = (int)(abs(horizonDistance - (*minValues)[0]) / mapResolution + 0.5);
        int tunnelSizeY = (int)(abs((*maxValues)[1] - (*minValues)[1]) / mapResolution + 0.5);
        int tunnelSizeZ = (int)(abs((*maxValues)[2] - (*minValues)[2]) / zResolution + 0.5);

        std::vector<double> pos{(*roverPath6)[0][0], (*roverPath6)[0][1], (*roverPath6)[0][2]};
        double roll = (*roverPath6)[0][3];
        double pitch = (*roverPath6)[0][4];
        double yaw = (*roverPath6)[0][5];
        std::vector<std::vector<double>> TW2BCS =
            dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        bool b_iscostassigned = false;

        for(int i = 0; i < tunnelSizeX; i++)
        {
            for(int j = 0; j < tunnelSizeY; j++)
            {
                for(int k = 0; k < tunnelSizeZ; k++)
                {
                    double x = (*minValues)[0] + mapResolution * i;
                    double y = (*minValues)[1] + mapResolution * j;
                    double z = (*minValues)[2] + zResolution * k;
                    double dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z - sherpa_tt_arm->d0, 2));
                    b_iscostassigned = false;
                    if(dist < sherpa_tt_arm->maxArmDistance)
                    {
                        std::vector<std::vector<double>> TBCS2Node = {
                            {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                        pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                        if(sherpa_tt_arm->isReachable(pos, 1) == 2)
                        // if ((sherpa_tt_arm->isReachable(pos,1) ==
                        // 2)&&(sherpa_tt_arm->getDistanceToCollision(pos,1) >
                        // 0.05))
                        {
                            std::vector<std::vector<double>> TW2Node = dot(TW2BCS, TBCS2Node);

                            int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                            int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                            int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                            double cost = 1 + slope / sherpa_tt_arm->getDistanceToCollision(pos, 1);

                            // std::cout << "Cost = " << cost << " and dist = "
                            // << sherpa_tt_arm->getDistanceToCollision(pos) <<
                            // std::endl;
                            if(ix > 0 && iy > 0 && iz > 0 && ix < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    (*costMap3D)[iy][ix][iz] = cost;
                                    (*tunnelLabel)[iy][ix][iz] = 1;
                                }
                            }
                            if(ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix + 1][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix + 1][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    (*costMap3D)[iy][ix + 1][iz] = cost;
                                    (*tunnelLabel)[iy][ix + 1][iz] = 1;
                                }
                            }
                            if(ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix - 1][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix - 1][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    (*costMap3D)[iy][ix - 1][iz] = cost;
                                    (*tunnelLabel)[iy][ix - 1][iz] = 1;
                                }
                            }
                            if(ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy + 1][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy + 1][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    (*costMap3D)[iy + 1][ix][iz] = cost;
                                    (*tunnelLabel)[iy + 1][ix][iz] = 1;
                                }
                            }
                            if(ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy - 1][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy - 1][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    (*costMap3D)[iy - 1][ix][iz] = cost;
                                    (*tunnelLabel)[iy - 1][ix][iz] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Tunnel during the rover movement
        tunnelSizeY = (int)(abs((*maxValues)[1] - (*minValues)[1]) / mapResolution + 0.5);
        tunnelSizeZ = (int)(abs((*maxValues)[2] - (*minValues)[2]) / zResolution + 0.5);

        for(int i = 1; i < n; i++)
        {
            pos = {(*roverPath6)[i][0], (*roverPath6)[i][1], (*roverPath6)[i][2]};
            roll = (*roverPath6)[i][3];
            pitch = (*roverPath6)[i][4];
            yaw = (*roverPath6)[i][5];
            TW2BCS = dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

            for(int j = 0; j < tunnelSizeY; j++)
            {
                for(int k = 0; k < tunnelSizeZ; k++)
                {
                    if(varyingHorizon)
                        horizonDistance =
                            (MAX_HORIZON - MIN_HORIZON) * i / roverPath6->size() + MIN_HORIZON;
                    double x = horizonDistance;
                    double y = (*minValues)[1] + mapResolution * j;
                    double z = (*minValues)[2] + zResolution * k;
                    double dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z - sherpa_tt_arm->d0, 2));
                    b_iscostassigned = false;
                    if(dist < sherpa_tt_arm->maxArmDistance)
                    {
                        std::vector<std::vector<double>> TBCS2Node = {
                            {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                        pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                        if(sherpa_tt_arm->isReachable(pos, 1) == 2)
                        // if ((sherpa_tt_arm->isReachable(pos,1) ==
                        // 2)&&(sherpa_tt_arm->getDistanceToCollision(pos,1) >
                        // 0.05))
                        {
                            std::vector<std::vector<double>> TW2Node = dot(TW2BCS, TBCS2Node);

                            int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                            int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                            int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                            double cost = 1 + slope / sherpa_tt_arm->getDistanceToCollision(pos, 1);

                            if(ix > 0 && iy > 0 && iz > 0 && ix < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix, iy, iz, i - n / 4);
                                    (*costMap3D)[iy][ix][iz] = cost;
                                    (*tunnelLabel)[iy][ix][iz] = i;
                                }
                            }
                            if(ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix + 1][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix + 1][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix + 1, iy, iz, i - n / 4);
                                    (*costMap3D)[iy][ix + 1][iz] = cost;
                                    (*tunnelLabel)[iy][ix + 1][iz] = i;
                                }
                            }
                            if(ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix - 1][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix - 1][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix - 1, iy, iz, i - n / 4);
                                    (*costMap3D)[iy][ix - 1][iz] = cost;
                                    (*tunnelLabel)[iy][ix - 1][iz] = i;
                                }
                            }
                            if(ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy + 1][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy + 1][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix, iy + 1, iz, i - n / 4);
                                    (*costMap3D)[iy + 1][ix][iz] = cost;
                                    (*tunnelLabel)[iy + 1][ix][iz] = i;
                                }
                            }
                            if(ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy - 1][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy - 1][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix, iy - 1, iz, i - n / 4);
                                    (*costMap3D)[iy - 1][ix][iz] = cost;
                                    (*tunnelLabel)[iy - 1][ix][iz] = i;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Tunnel in the last waypoint
        tunnelSizeX = (int)(abs((*maxValues)[0] - horizonDistance) / mapResolution + 0.5);
        tunnelSizeY = (int)(abs((*maxValues)[1] - (*minValues)[1]) / mapResolution + 0.5);
        tunnelSizeZ = (int)(abs((*maxValues)[2] - (*minValues)[2]) / zResolution + 0.5);

        pos = {(*roverPath6)[roverPath6->size() - 1][0],
               (*roverPath6)[roverPath6->size() - 1][1],
               (*roverPath6)[roverPath6->size() - 1][2]};
        roll = (*roverPath6)[roverPath6->size() - 1][3];
        pitch = (*roverPath6)[roverPath6->size() - 1][4];
        yaw = (*roverPath6)[roverPath6->size() - 1][5];
        TW2BCS = dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        for(int i = 0; i < tunnelSizeX; i++)
        {
            for(int j = 0; j < tunnelSizeY; j++)
            {
                for(int k = 0; k < tunnelSizeZ; k++)
                {
                    double x = horizonDistance + mapResolution * i;
                    double y = (*minValues)[1] + mapResolution * j;
                    double z = (*minValues)[2] + zResolution * k;
                    double dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z - sherpa_tt_arm->d0, 2));
                    b_iscostassigned = false;
                    if(dist < sherpa_tt_arm->maxArmDistance)
                    {
                        std::vector<std::vector<double>> TBCS2Node = {
                            {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                        pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                        if(sherpa_tt_arm->isReachable(pos, 1) == 2)
                        // if ((sherpa_tt_arm->isReachable(pos,1) ==
                        // 2)&&(sherpa_tt_arm->getDistanceToCollision(pos,1) >
                        // 0.05))
                        {
                            std::vector<std::vector<double>> TW2Node = dot(TW2BCS, TBCS2Node);

                            int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                            int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                            int iz = (int)(TW2Node[2][3] / zResolution + 0.5);
                            double cost = 1 + slope / sherpa_tt_arm->getDistanceToCollision(pos, 1);

                            if(ix > 0 && iy > 0 && iz > 0 && ix < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(tunnelLabel, costMap3D, ix, iy, iz, n / 5);
                                    (*costMap3D)[iy][ix][iz] = cost;
                                    (*tunnelLabel)[iy][ix][iz] = n;
                                }
                            }
                            if(ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix + 1][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix + 1][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix + 1, iy, iz, n / 5);
                                    (*costMap3D)[iy][ix + 1][iz] = cost;
                                    (*tunnelLabel)[iy][ix + 1][iz] = n;
                                }
                            }
                            if(ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy][ix - 1][iz])
                                {
                                    if(isinf((*costMap3D)[iy][ix - 1][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix - 1, iy, iz, n / 5);
                                    (*costMap3D)[iy][ix - 1][iz] = cost;
                                    (*tunnelLabel)[iy][ix - 1][iz] = n;
                                }
                            }
                            if(ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy + 1][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy + 1][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix, iy + 1, iz, n / 5);
                                    (*costMap3D)[iy + 1][ix][iz] = cost;
                                    (*tunnelLabel)[iy + 1][ix][iz] = n;
                                }
                            }
                            if(ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1 &&
                               iz < sz - 1)
                            {
                                if(cost < (*costMap3D)[iy - 1][ix][iz])
                                {
                                    if(isinf((*costMap3D)[iy - 1][ix][iz]))
                                    {
                                        b_iscostassigned = true;
                                        ui_noninf_nodes++;
                                    }
                                    checkIntersections(
                                        tunnelLabel, costMap3D, ix, iy - 1, iz, n / 5);
                                    (*costMap3D)[iy - 1][ix][iz] = cost;
                                    (*tunnelLabel)[iy - 1][ix][iz] = n;
                                }
                            }
                        }
                    }
                }
            }
        }
        return ui_noninf_nodes;
    }

    void ArmPlanner::generateReachabilityTunnel(
        base::Waypoint iniPos,
        base::Waypoint goalPos,
        std::vector<double> roverPose6,
        bool isTunnelPermisive,
        std::vector<std::vector<std::vector<double>>> * costMap3D,
        int mode)
    {
        int sx = (*costMap3D).size();
        int sy = (*costMap3D)[0].size();
        int sz = (*costMap3D)[0][0].size();

        std::vector<int> goal(3, 0);
        std::vector<int> start(3, 0);

        start[0] = (int)(iniPos.position[0] / mapResolution + 0.5);
        start[1] = (int)(iniPos.position[1] / mapResolution + 0.5);
        start[2] = (int)(iniPos.position[2] / zResolution + 0.5);
        (*costMap3D)[start[1]][start[0]][start[2]] = 1;

        goal[0] = (int)(goalPos.position[0] / mapResolution + 0.5);
        goal[1] = (int)(goalPos.position[1] / mapResolution + 0.5);
        goal[2] = (int)(goalPos.position[2] / zResolution + 0.5);
        (*costMap3D)[goal[1]][goal[0]][goal[2]] = 1;

        std::vector<double> * minValues = sherpa_tt_arm->minValues;
        std::vector<double> * maxValues = sherpa_tt_arm->maxValues;

        double slope = 0.5;

        // Tunnel in the last waypoint
        int tunnelSizeX = (int)(abs((*maxValues)[0] - (*minValues)[0]) / mapResolution + 0.5);
        int tunnelSizeY = (int)(abs((*maxValues)[1] - (*minValues)[1]) / mapResolution + 0.5);
        int tunnelSizeZ = (int)(abs((*maxValues)[2] - (*minValues)[2]) / zResolution + 0.5);

        std::vector<double> pos = {roverPose6[0], roverPose6[1], roverPose6[2]};
        double roll = roverPose6[3];
        double pitch = roverPose6[4];
        double yaw = roverPose6[5];
        std::vector<std::vector<double>> TW2BCS =
            dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

        for(int i = 0; i < tunnelSizeX; i++)
            for(int j = 0; j < tunnelSizeY; j++)
                for(int k = 0; k < tunnelSizeZ; k++)
                {
                    double x = (*minValues)[0] + mapResolution * i;
                    double y = (*minValues)[1] + mapResolution * j;
                    double z = (*minValues)[2] + zResolution * k;
                    double dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z - sherpa_tt_arm->d0, 2));
                    if(dist < sherpa_tt_arm->maxArmDistance)
                    {
                        std::vector<std::vector<double>> TBCS2Node = {
                            {1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

                        pos = {TBCS2Node[0][3], TBCS2Node[1][3], TBCS2Node[2][3]};

                        int reachability = sherpa_tt_arm->isReachable(pos, 0);
                        if(reachability == 2 || (reachability == 1 && isTunnelPermisive))
                        {
                            std::vector<std::vector<double>> TW2Node = dot(TW2BCS, TBCS2Node);

                            int ix = (int)(TW2Node[0][3] / mapResolution + 0.5);
                            int iy = (int)(TW2Node[1][3] / mapResolution + 0.5);
                            int iz = (int)(TW2Node[2][3] / zResolution + 0.5);

                            double cost;
                            if(reachability == 2)
                            {
                                cost = 1 + slope / sherpa_tt_arm->getDistanceToCollision(pos, mode);
                                // std::cout << "Cost = " << cost << " and dist
                                // = "
                                // << sherpa_tt_arm->getDistanceToCollision(pos)
                                // << std::endl;
                            }
                            else
                                cost = 10;
                            if(ix > 0 && iy > 0 && iz > 0 && ix < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                                if(cost < (*costMap3D)[iy][ix][iz]) (*costMap3D)[iy][ix][iz] = cost;

                            if(ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                                if(cost < (*costMap3D)[iy][ix + 1][iz])
                                    (*costMap3D)[iy][ix + 1][iz] = cost;
                            if(ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1 &&
                               iz < sz - 1)
                                if(cost < (*costMap3D)[iy][ix - 1][iz])
                                    (*costMap3D)[iy][ix - 1][iz] = cost;
                            if(ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1 &&
                               iz < sz - 1)
                                if(cost < (*costMap3D)[iy + 1][ix][iz])
                                    (*costMap3D)[iy + 1][ix][iz] = cost;
                            if(ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1 &&
                               iz < sz - 1)
                                if(cost < (*costMap3D)[iy - 1][ix][iz])
                                    (*costMap3D)[iy - 1][ix][iz] = cost;
                        }
                    }
                }
    }

    bool ArmPlanner::computeWaypointAssignment(std::vector<int> * pathsAssignment,
                                               CollisionDetector * p_collision_detector)
    {
        std::vector<double> armBasePos;
        (*pathsAssignment) = std::vector<int>(roverPath6->size(), 0);

        int i_maxindex = roverPath6->size() - 1, j_maxindex = wristPath6->size() - 1;

        for(int i = 0; i <= i_maxindex; i++)
        {
            if(varyingHorizon)
            {
                horizonDistance =
                    (MAX_HORIZON - MIN_HORIZON) * i / roverPath6->size() + MIN_HORIZON;
            }

            std::vector<double> pos{(*roverPath6)[i][0], (*roverPath6)[i][1], (*roverPath6)[i][2]};
            double roll = (*roverPath6)[i][3];
            double pitch = (*roverPath6)[i][4];
            double yaw = (*roverPath6)[i][5];

            std::vector<std::vector<double>> TW2BCS =
                dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

            for(int j = wristPath6->size() - 1; j > -1; j--)
            {
                pos = {(*wristPath6)[j][0], (*wristPath6)[j][1], (*wristPath6)[j][2]};
                roll = (*wristPath6)[j][3];
                pitch = (*wristPath6)[j][4];
                yaw = (*wristPath6)[j][5];

                std::vector<std::vector<double>> TW2Wrist =
                    dot(getTraslation(pos), dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll))));

                std::vector<std::vector<double>> TBCS2Wrist = dot(getInverse(&TW2BCS), TW2Wrist);

                pos = {TBCS2Wrist[0][3], TBCS2Wrist[1][3], TBCS2Wrist[2][3]};

                std::vector<double> config;
                try
                {
                    config = sherpa_tt_arm->getPositionJoints(pos, 1, 1, 0.0);
                    std::vector<double> basePos = {0, 0, sherpa_tt_arm->d0};
                    double dist = getDist3(basePos, pos);
                    if(TBCS2Wrist[0][3] < horizonDistance &&
                       (!p_collision_detector->isWristColliding(config)) &&
                       dist < sherpa_tt_arm->maxArmOptimalDistance)
                    {
                        // std::cout << "DEBUG: rover path index " << i << "has
                        // associated Wrist index " << j << " and horizon
                        // distance is " << horizonDistance << " and dist is "
                        // << dist << std::endl;
                        (*pathsAssignment)[i] = j;
                        break;
                    }
                }
                catch(std::exception & e)
                {
                    /*std::cout << "Arm Planner library: EXCEPTION inverse
                kinematic computation in assignation failed" << std::endl;
                std::cout << "Arm Planner library: debug info" << std::endl;
                std::cout << " - Rover Path index = " << i << std::endl;
                std::cout << " - Wrist Path index = " << j << std::endl;
                std::cout << " - position = " << pos[0] << ", " << pos[1] << ",
                " << pos[2] << std::endl; return false;*/
                    // return 0;
                }
            }
        }

        for(int i = i_maxindex; i > 0; i--)
        {
            if((*pathsAssignment)[i] < (*pathsAssignment)[i - 1])
            {
                (*pathsAssignment)[i - 1] = (*pathsAssignment)[i];
            }
            //	i_maxindex = i;
        }

        (*pathsAssignment)[0] = 0;
        (*pathsAssignment)[roverPath6->size() - 1] = wristPath6->size() - 1;
        return true;
    }

    void ArmPlanner::computeWaypointInterpolation(const std::vector<int> * pathsAssignment,
                                                  std::vector<base::Waypoint> * newRoverPath,
                                                  std::vector<int> * newAssignment)
    {
        for(int i = 0; i < pathsAssignment->size(); i++)
        {
            (*newRoverPath)[i].position[0] = (*roverPath6)[i][0];
            (*newRoverPath)[i].position[1] = (*roverPath6)[i][1];
            (*newRoverPath)[i].position[2] = (*roverPath6)[i][2];
            (*newRoverPath)[i].heading = (*roverPath6)[i][5];

            (*newAssignment)[i] = (*pathsAssignment)[i];
        }

        int i = 1;
        while(i < newRoverPath->size())
        {
            int diff = (*newAssignment)[i] - (*newAssignment)[i - 1];
            if(diff > 5)
            {
                std::vector<base::Waypoint> newWaypoints =
                    getLinearInterpolation((*newRoverPath)[i - 1], (*newRoverPath)[i], diff - 1);
                newRoverPath->insert(
                    newRoverPath->begin() + i, newWaypoints.begin(), newWaypoints.end());
                for(int j = 0; j < diff - 1; j++)
                    newAssignment->insert(newAssignment->begin() + i + j,
                                          (*newAssignment)[i - 1] + j + 1);
                i += diff - 1;
            }
            i++;
        }
    }

    void ArmPlanner::smoothRoverPathHeading(double headingThreshold)
    {
        int i = 1;
        while(i < roverPath6->size())
        {
            double diff = abs((*roverPath6)[i][5] - (*roverPath6)[i - 1][5]);
            if(diff > 2 * M_PI) diff = diff - 2 * M_PI;
            // std::cout<<"Diff: "<<diff<<", threshold:
            // "<<headingThreshold<<std::endl;
            if(diff > headingThreshold)
            {
                std::vector<std::vector<double>> newWaypoints = getLinearInterpolation(
                    (*roverPath6)[i - 1], (*roverPath6)[i], round(diff / headingThreshold) + 1);
                roverPath6->insert(
                    roverPath6->begin() + i, newWaypoints.begin(), newWaypoints.end());
                i += round(diff / headingThreshold) + 1;
            }
            i++;
        }
    }

    std::vector<base::Waypoint> ArmPlanner::getLinearInterpolation(
        base::Waypoint waypoint0, base::Waypoint waypoint1, int numberIntWaypoints)
    {
        double x0 = waypoint0.position[0];
        double y0 = waypoint0.position[1];
        double yaw0 = waypoint0.heading;

        double x1 = waypoint1.position[0];
        double y1 = waypoint1.position[1];
        double yaw1 = waypoint1.heading;

        if(yaw1 - yaw0 > pi)
            yaw1 -= 2 * pi;
        else if(yaw1 - yaw0 < -pi)
            yaw1 += 2 * pi;

        double l;
        std::vector<double> xi(10002, 0);
        std::vector<double> yi(10002, 0);
        std::vector<double> heading(10002, 0);
        std::vector<double> laccum(10002, 0);

        xi[0] = x0;
        yi[0] = y0;
        heading[0] = yaw0;

        for(int i = 1; i < 10002; i++)
        {
            xi[i] = x0 + i * (x1 - x0) / 10001;
            yi[i] = y0 + i * (y1 - y0) / 10001;
            heading[i] = yaw0 + i * (yaw1 - yaw0) / 10001;

            while(heading[i] > pi)
                heading[i] -= 2 * pi;
            while(heading[i] < -pi)
                heading[i] += 2 * pi;

            l = sqrt((xi[i] - xi[i - 1]) * (xi[i] - xi[i - 1]) +
                     (yi[i] - yi[i - 1]) * (yi[i] - yi[i - 1]));
            laccum[i] = laccum[i - 1] + l;
        }
        double d = laccum[10001] / (numberIntWaypoints + 1);

        std::vector<base::Waypoint> newWaypoints(numberIntWaypoints);
        for(int i = 0; i < numberIntWaypoints; i++)
            for(int j = 0; j < 10002; j++)
                if(laccum[j] > (i + 1) * d)
                {
                    newWaypoints[i].position[0] = xi[j];
                    newWaypoints[i].position[1] = yi[j];
                    newWaypoints[i].position[2] =
                        (waypoint0.position[2] + waypoint1.position[2]) / 2;
                    newWaypoints[i].heading = heading[j];

                    break;
                }

        return newWaypoints;
    }

    std::vector<std::vector<double>> ArmPlanner::getLinearInterpolation(
        std::vector<double> waypoint0, std::vector<double> waypoint1, int numberIntWaypoints)
    {
        double x0 = waypoint0[0];
        double y0 = waypoint0[1];
        double yaw0 = waypoint0[5];

        double x1 = waypoint1[0];
        double y1 = waypoint1[1];
        double yaw1 = waypoint1[5];

        if(yaw1 - yaw0 > pi)
            yaw1 -= 2 * pi;
        else if(yaw1 - yaw0 < -pi)
            yaw1 += 2 * pi;

        double l;
        std::vector<double> xi(10002, 0);
        std::vector<double> yi(10002, 0);
        std::vector<double> heading(10002, 0);
        std::vector<double> laccum(10002, 0);

        xi[0] = x0;
        yi[0] = y0;
        heading[0] = yaw0;

        for(int i = 1; i < 10002; i++)
        {
            xi[i] = x0 + i * (x1 - x0) / 10001;
            yi[i] = y0 + i * (y1 - y0) / 10001;
            heading[i] = yaw0 + i * (yaw1 - yaw0) / 10001;

            while(heading[i] > pi)
                heading[i] -= 2 * pi;
            while(heading[i] < -pi)
                heading[i] += 2 * pi;

            l = sqrt((xi[i] - xi[i - 1]) * (xi[i] - xi[i - 1]) +
                     (yi[i] - yi[i - 1]) * (yi[i] - yi[i - 1]));
            laccum[i] = laccum[i - 1] + l;
        }
        double d = laccum[10001] / (numberIntWaypoints + 1);

        std::vector<std::vector<double>> newWaypoints(numberIntWaypoints, std::vector<double>(6));
        for(int i = 0; i < numberIntWaypoints; i++)
            for(int j = 0; j < 10002; j++)
                if(laccum[j] > (i + 1) * d)
                {
                    newWaypoints[i][0] = xi[j];
                    newWaypoints[i][1] = yi[j];
                    newWaypoints[i][2] = (waypoint0[2] + waypoint1[2]) / 2;
                    newWaypoints[i][5] = heading[j];

                    break;
                }

        return newWaypoints;
    }

    std::vector<base::Waypoint> ArmPlanner::getCubicInterpolation(
        base::Waypoint waypoint0, base::Waypoint waypoint1, int numberIntWaypoints)
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
        for(int i = 1; i < 10002; i++)
        {
            xi[i] = x0 + i * (x1 - x0) / 10001;
            yi[i] = a * xi[i] * xi[i] * xi[i] + b * xi[i] * xi[i] + c * xi[i] + d;
            heading[i] = yaw0 + i * (yaw1 - yaw0) / 10001;

            while(heading[i] > pi)
                heading[i] -= 2 * pi;
            while(heading[i] < -pi)
                heading[i] += 2 * pi;

            l = sqrt((xi[i] - xi[i - 1]) * (xi[i] - xi[i - 1]) +
                     (yi[i] - yi[i - 1]) * (yi[i] - yi[i - 1]));
            laccum[i] = laccum[i - 1] + l;
        }
        double dist = laccum[10001] / (numberIntWaypoints + 1);

        std::vector<base::Waypoint> newWaypoints(numberIntWaypoints);
        for(int i = 0; i < numberIntWaypoints; i++)
        {
            for(int j = 0; j < 10002; j++)
                if(laccum[j] > (i + 1) * dist)
                {
                    newWaypoints[i].position[0] = xi[j];
                    newWaypoints[i].position[1] = yi[j];
                    newWaypoints[i].position[2] =
                        (waypoint0.position[2] + waypoint1.position[2]) / 2;
                    newWaypoints[i].heading = heading[j];

                    break;
                }
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
        if(samples % 2 == 0)
        {
            doubleCenter = true;
            samples--;
        }
        int steps = (samples - 1) / 2;
        double stepSize = (3 * sigma) / steps;

        for(int i = steps; i >= 1; i--)
        {
            v.push_back(getGaussValue(sigma, i * stepSize * -1));
        }

        v.push_back(getGaussValue(sigma, 0));
        if(doubleCenter)
        {
            v.push_back(getGaussValue(sigma, 0));
        }

        for(int i = 1; i <= steps; i++)
        {
            v.push_back(getGaussValue(sigma, i * stepSize));
        }

        assert(v.size() == samples);

        return v;
    }

    std::vector<double> ArmPlanner::getGaussSmoothen(
        std::vector<double> values, double sigma, int samples)
    {
        std::vector<double> out;
        auto kernel = getGaussKernel(samples, sigma);
        double kernelSum = 0;
        for(int i = 0; i < kernel.size(); i++)
            kernelSum += kernel[i];
        int sampleSide = samples / 2;
        int valueIdx = samples / 2 + 1;
        unsigned long ubound = values.size();
        for(unsigned long i = 0; i < ubound; i++)
        {
            double sample = 0;
            int sampleCtr = 0;
            for(long j = i - sampleSide; j <= i + sampleSide; j++)
            {
                if(j > 0 && j < ubound)
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

    void ArmPlanner::checkIntersections(std::vector<std::vector<std::vector<int>>> * tunnelLabel,
                                        std::vector<std::vector<std::vector<double>>> * tunnelCost,
                                        int ix,
                                        int iy,
                                        int iz,
                                        int threshold)
    {
        /*
            int sx = (*tunnelLabel).size();
            int sy = (*tunnelLabel)[0].size();
            int sz = (*tunnelLabel)[0][0].size();

            if (ix + 1 > 0 && iy > 0 && iz > 0 && ix + 1 < sx - 1 && iy < sy - 1
                && iz < sz - 1)
                if ((*tunnelLabel)[iy][ix + 1][iz] < threshold)
                {
                    //std::cout<<"Detected intersection with label
           "<<(*tunnelLabel)[iy][ix + 1][iz]<<" and threshold
           "<<threshold<<"\n";
                    (*tunnelCost)[iy][ix + 1][iz] = INFINITY;
                }
            if (ix - 1 > 0 && iy > 0 && iz > 0 && ix - 1 < sx - 1 && iy < sy - 1
                && iz < sz - 1)
                if ((*tunnelLabel)[iy][ix - 1][iz] < threshold)
                {
                    //std::cout<<"Detected intersection with label
           "<<(*tunnelLabel)[iy][ix - 1][iz]<<" and threshold
           "<<threshold<<"\n";
                    (*tunnelCost)[iy][ix - 1][iz] = INFINITY;
                }
            if (ix > 0 && iy + 1 > 0 && iz > 0 && ix < sx - 1 && iy + 1 < sy - 1
                && iz < sz - 1)
                if ((*tunnelLabel)[iy + 1][ix][iz] < threshold)
                {
                    //std::cout<<"Detected intersection with label
           "<<(*tunnelLabel)[iy+1][ix][iz]<<" and threshold "<<threshold<<"\n";
                    (*tunnelCost)[iy + 1][ix][iz] = INFINITY;
                }
            if (ix > 0 && iy - 1 > 0 && iz > 0 && ix < sx - 1 && iy - 1 < sy - 1
                && iz < sz - 1)
                if ((*tunnelLabel)[iy - 1][ix][iz] < threshold)
                {
                    //std::cout<<"Detected intersection with label
           "<<(*tunnelLabel)[iy-1][ix][iz]<<" and threshold "<<threshold<<"\n";
                    (*tunnelCost)[iy - 1][ix][iz] = INFINITY;
                }
            if (ix > 0 && iy > 0 && iz + 1 > 0 && ix < sx - 1 && iy < sy - 1
                && iz + 1 < sz - 1)
                if ((*tunnelLabel)[iy][ix][iz + 1] < threshold)
                {
                    //std::cout<<"Detected intersection with label
           "<<(*tunnelLabel)[iy][ix][iz+1]<<" and threshold "<<threshold<<"\n";
                    (*tunnelCost)[iy][ix][iz + 1] = INFINITY;
                }
            if (ix > 0 && iy > 0 && iz - 1 > 0 && ix < sx - 1 && iy < sy - 1
                && iz - 1 < sz - 1)
                if ((*tunnelLabel)[iy][ix][iz - 1] < threshold)
                {
                    //std::cout<<"Detected intersection with label
           "<<(*tunnelLabel)[iy][ix][iz-1]<<" and threshold "<<threshold<<"\n";
                    (*tunnelCost)[iy][ix][iz - 1] = INFINITY;
                }
        */
    }

    double ArmPlanner::getTimeArmJointMovement(
        double initialPosition, double goalPosition, int armJointNumber)
    {
        return abs(goalPosition - initialPosition) /
               sherpa_tt_arm->armJointsMaxSpeed[armJointNumber - 1];
    }

    double ArmPlanner::getMaxTimeArmMovement(std::vector<double> initialConfiguration,
                                             std::vector<double> goalConfiguration)
    {
        double maxTime = 0;
        for(int i = 0; i < initialConfiguration.size(); i++)
        {
            double jointTime =
                getTimeArmJointMovement(initialConfiguration[i], goalConfiguration[i], i + 1);
            if(jointTime > maxTime) maxTime = jointTime;
        }

        return maxTime;
    }

    std::vector<double> ArmPlanner::getTimeProfile(std::vector<std::vector<double>> * armProfile)
    {
        std::vector<double> times;
        times.push_back(0);
        for(int i = 1; i < armProfile->size(); i++)
        {
            times.push_back(times[i - 1] +
                            getMaxTimeArmMovement((*armProfile)[i - 1], (*armProfile)[i]));
        }
        return times;
    }

    void ArmPlanner::computeArmProfileGaussSmoothening(
        const std::vector<std::vector<double>> * armProfile,
        std::vector<std::vector<double>> * smoothedArmProfile,
        double sigma,
        int samples)
    {
        if(samples % 2 == 0)
        {
            samples++;
            std::cout << "WARNING [computeArmProfileGaussSmoothening]: number of "
                         "samples should be odd, changing to "
                      << samples << std::endl;
        }

        (*smoothedArmProfile) = (*armProfile);

        for(int i = 0; i < (*armProfile)[0].size(); i++)
        {
            std::vector<double> jointProfile;
            for(int j = 0; j < armProfile->size(); j++)
            {
                jointProfile.push_back((*armProfile)[j][i]);
            }

            for(int j = 1; j < armProfile->size(); j++)
            {
                if(jointProfile[j] - jointProfile[j - 1] > M_PI)
                {
                    for(int k = j; k < armProfile->size(); k++)
                    {
                        jointProfile[k] -= 2 * M_PI;
                    }
                }
                else if(jointProfile[j] - jointProfile[j - 1] < -M_PI)
                {
                    for(int k = j; k < armProfile->size(); k++)
                    {
                        jointProfile[k] += 2 * M_PI;
                    }
                }
            }

            std::vector<double> smoothedJointProfile =
                getGaussSmoothen(jointProfile, sigma, samples);

            for(int j = 0; j < smoothedJointProfile.size(); j++)
            {
                while(smoothedJointProfile[j] > M_PI)
                    smoothedJointProfile[j] -= 2 * M_PI;
                while(smoothedJointProfile[j] < -M_PI)
                    smoothedJointProfile[j] += 2 * M_PI;
            }

            for(int j = (samples / 2 + samples % 2);
                j < armProfile->size() - (samples / 2 + samples % 2);
                j++)
                (*smoothedArmProfile)[j][i] = smoothedJointProfile[j];
        }
    }
