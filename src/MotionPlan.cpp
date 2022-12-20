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

#include "MotionPlan.h"

MotionPlan::MotionPlan(MobileManipMap * pmmmap_m,
                       std::string s_urdf_path_m,
                       unsigned int ui_deployment)
{
    this->pmm_map = pmmmap_m;
    this->d_zres = 0.8 * this->pmm_map->getResolution();    // d_zres_m;
    std::cout << "[MM] \033[35m[----------] [MotionPlan::MotionPlan()]\033[0m Z-Resolution is "
              << this->d_zres << " meters" << std::endl;
    this->s_urdf_path = s_urdf_path_m;
    this->p_arm_planner = new ArmPlanner(s_urdf_path_m, true, ui_deployment);
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m);
    this->b_is_retrieval_computed = false;
    this->b_is_initialization_computed = false;

    std::cout << "[MM] \033[1;35m[----------] [MotionPlan::MotionPlan()]\033[0m Motion Plan "
                 "successfully constructed"
              << std::endl;
}

MotionPlan::MotionPlan(MobileManipMap * pmmmap_m,
                       std::string s_urdf_path_m,
                       std::vector<Waypoint> & vw_rover_path_m,
                       std::vector<std::vector<double>> & m_arm_motion_profile)
{
    this->pmm_map = pmmmap_m;
    this->d_zres = 0.8 * this->pmm_map->getResolution();    // d_zres_m;
    std::cout << "[MM] \033[35m[----------] [MotionPlan::MotionPlan()]\033[0m Z-Resolution is "
              << this->d_zres << " meters" << std::endl;
    this->s_urdf_path = s_urdf_path_m;
    this->vw_rover_path.clear();
    this->p_arm_planner = new ArmPlanner(s_urdf_path_m, false, 1);
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m);
    for(uint i = 0; i < vw_rover_path_m.size(); i++)
    {
        this->vw_rover_path.push_back(vw_rover_path_m[i]);
    }

    std::vector<double> row;
    this->vvd_arm_motion_profile.clear();
    for(uint j = 0; j < m_arm_motion_profile.size(); j++)
    {
        for(uint i = 0; i < m_arm_motion_profile[0].size(); i++)
        {
            row.push_back(m_arm_motion_profile[j][i]);
        }
        this->vvd_arm_motion_profile.push_back(row);
        row.clear();
    }
    std::cout << "[MM] \033[1;35m[----------] [MotionPlan::MotionPlan()]\033[0m Motion Plan "
                 "successfully constructed"
              << std::endl;
}

void MotionPlan::setDeployment(unsigned int ui_deployment)
{
    this->p_arm_planner->setDeployment(ui_deployment);
}

unsigned int MotionPlan::computeRoverBasePathPlanning(base::Waypoint rover_position)
{
    std::vector<std::vector<double>> costMap;
    if(this->pmm_map->isOutside(rover_position))
    {
        return 1;
    }
    if(this->pmm_map->isObstacle(rover_position))
    {
        return 2;
    }
    if(!this->pmm_map->isSampleLoaded())
    {
        return 3;
    }

    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeRoverBasePathPlanning()]\033[0m "
                 "Getting Cost Map from MMMap"
              << std::endl;
    this->pmm_map->getCostMap(costMap);
    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeRoverBasePathPlanning()]\033[0m "
                 "Got the Cost Map"
              << std::endl;
    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeRoverBasePathPlanning()]\033[0m "
                 "Starting Path Planning with biFM"
              << std::endl;

    if(this->bifm_planner.planPath(&costMap,    // this->pmm_map->getCostMap(),
                                   this->pmm_map->getResolution(),
                                   rover_position,
                                   this->pmm_map->getSample(),
                                   &(this->vw_rover_path)))
    {
        if(isSmoothPath())
        {
            std::cout
                << "[MM] \033[1;35m[----------] "
                   "[MotionPlan::computeRoverBasePathPlanning()]\033[0m Computed path with biFM"
                << std::endl;
            return 0;
        }
        else
        {
            std::cout << "[MM] \033[1;35m[----------] "
                         "[MotionPlan::computeRoverBasePathPlanning()]\033[0m Path is degenerated, "
                         "input cost map not smooth enough"
                      << std::endl;
            return 5;
        }
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] "
                     "[MotionPlan::computeRoverBasePathPlanning()]\033[0m Changing to (uni)FM"
                  << std::endl;
        if(this->fm_planner.planPath(&costMap,    // this->pmm_map->getCostMap(),
                                     this->pmm_map->getResolution(),
                                     rover_position,
                                     this->pmm_map->getSample(),
                                     &(this->vw_rover_path)))
        {
            if(isSmoothPath())
            {
                std::cout << "[MM] \033[1;35m[----------] "
                             "[MotionPlan::computeRoverBasePathPlanning()]\033[0m Computed path "
                             "with (uni)FM"
                          << std::endl;
                return 0;
            }
            else
            {
                std::cout << "[MM] \033[1;35m[----------] "
                             "[MotionPlan::computeRoverBasePathPlanning()]\033[0m Path is "
                             "degenerated, input cost map not smooth enough"
                          << std::endl;
                return 5;
            }
        }
        else
        {
            std::cout << "[MM] \033[1;35m[----------] "
                         "[MotionPlan::computeRoverBasePathPlanning()]\033[0m There is no feasible "
                         "path to obtain"
                      << std::endl;
            return 4;
        }
    }
}

bool MotionPlan::isPathColliding()
{
    return this->pmm_map->checkObstacles(this->vw_rover_path);
}

bool MotionPlan::isSmoothPath()
{
    if(this->vw_rover_path.size() < 3)
    {
        return false;
    }
    double d_segmentX1, d_segmentY1, d_segmentX2, d_segmentY2, d_norm1, d_norm2, d_diffheading;
    for(unsigned int i = 2; i < vw_rover_path.size() - 1;
        i++)    // Final Waypoint is sometimes tricky due to FM computation, so it
                // is left anyways
    {
        d_segmentX1 = this->vw_rover_path[i].position[0] - this->vw_rover_path[i - 1].position[0];
        d_segmentX2 =
            this->vw_rover_path[i - 1].position[0] - this->vw_rover_path[i - 2].position[0];
        d_segmentY1 = this->vw_rover_path[i].position[1] - this->vw_rover_path[i - 1].position[1];
        d_segmentY2 =
            this->vw_rover_path[i - 1].position[1] - this->vw_rover_path[i - 2].position[1];
        d_norm1 = sqrt(pow(d_segmentX1, 2) + pow(d_segmentY1, 2));
        d_norm2 = sqrt(pow(d_segmentX2, 2) + pow(d_segmentY2, 2));
        d_diffheading =
            acos((d_segmentX1 * d_segmentX2 + d_segmentY1 * d_segmentY2) / (d_norm1 * d_norm2)) *
            180.0 / 3.1416;
        if(d_diffheading > 45.0)
        {
            return false;
        }
    }
    return true;
}

bool MotionPlan::shortenPathForFetching()
{
    // TODO- Make this cut a shorter distance, taking into account tol_position
    // from Waypoint Navigation
    unsigned int endWaypoint = this->fetching_pose_estimator.getFetchWaypointIndex(
        &(this->vw_rover_path), this->pmm_map->getMinReach(), this->pmm_map->getMaxReach());
    if(endWaypoint == 0)
    {
        std::cout << "[MM] \033[35m[----------] [MotionPlan::shortenPathForFetching()]\033[0m "
                     "Cannot Shorten the path"
                  << std::endl;
        return false;
    }
    else
    {
        unsigned int i_eraseIndex = endWaypoint + 1;
        std::cout << "[MM] \033[35m[----------] [MotionPlan::shortenPathForFetching()]\033[0m "
                     "Shortening the path to waypoint "
                  << endWaypoint << std::endl;
        if(endWaypoint < this->vw_rover_path.size() - 3)
        {
            double d_segmentX = this->vw_rover_path[i_eraseIndex].position[0] -
                                this->vw_rover_path[endWaypoint].position[0];
            double d_segmentY = this->vw_rover_path[i_eraseIndex].position[1] -
                                this->vw_rover_path[endWaypoint].position[1];
            double d_norm = sqrt(pow(d_segmentX, 2) + pow(d_segmentY, 2));
            while((i_eraseIndex < this->vw_rover_path.size() - 2) &&
                  (d_norm < 0.1))    // TODO-Introduce here configurable
                                     // position tolerance
            {
                i_eraseIndex += 1;
                d_segmentX = this->vw_rover_path[i_eraseIndex].position[0] -
                             this->vw_rover_path[endWaypoint].position[0];
                d_segmentY = this->vw_rover_path[i_eraseIndex].position[1] -
                             this->vw_rover_path[endWaypoint].position[1];
                d_norm = sqrt(pow(d_segmentX, 2) + pow(d_segmentY, 2));
            }
            this->vw_rover_path.erase(this->vw_rover_path.begin() + i_eraseIndex,
                                      this->vw_rover_path.end());
        }
        else
        {
            this->vw_rover_path.erase(this->vw_rover_path.begin() + endWaypoint + 1,
                                      this->vw_rover_path.end());
        }
    }
    std::cout << "[MM] \033[1;35m[----------] [MotionPlan::shortenPathForFetching()]\033[0m Path "
                 "is shortened"
              << std::endl;
    return true;
}

void MotionPlan::addSampleWaypointToPath()
{
    base::Waypoint w_sample = this->pmm_map->getSample();
    this->vw_rover_path.push_back(w_sample);
}

unsigned int MotionPlan::computeArmProfilePlanning()
{
    this->b_is_retrieval_computed = false;
    this->b_is_initialization_computed = false;
    if(!this->pmm_map->isSampleLoaded())
    {
        std::cout << "[MM] \033[1;33m[----------] [MotionPlan::computeArmProfilePlanning()]\033[0m "
                     "Sample was not loaded "
                  << std::endl;
        return 3;
    }

    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeArmProfilePlanning()]\033[0m "
                 "Getting Elevation Map from MMMap class"
              << std::endl;
    std::vector<std::vector<double>> elevationMap;
    // TODO: Check this! Using a pointer would be much better
    this->pmm_map->getElevationMapToZero(elevationMap);
    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeArmProfilePlanning()]\033[0m Got "
                 "Elevation Map from MMMap class"
              << std::endl;

    std::vector<base::Waypoint> vw_reference_path;

    unsigned int ui_deployment = 2;    // TODO: make this take the true current one

    bool minimize_arm_movement = true;    // TODO: make this take the true current one
    double min_arm_moving_time = 35.21899630340112;
    double rover_speed = 0.1;
    double distance_start_movement = min_arm_moving_time * rover_speed;
    double dist_to_goal = 0;
    unsigned int first_arm_waypoint = 0;

    for(uint i = this->vw_rover_path.size() - 1; i > 1; i--)
    {
        dist_to_goal += (vw_rover_path[i].position - vw_rover_path[i - 1].position).norm();
        if(dist_to_goal > distance_start_movement)
        {
            first_arm_waypoint = i;
            break;
        }
    }

    while(ui_deployment >= 0)
    {
        if(minimize_arm_movement)
        {
            vw_reference_path.resize(this->vw_rover_path.size() - first_arm_waypoint);
            for(uint i = 0; i < vw_reference_path.size(); i++)
            {
                vw_reference_path[i] = this->vw_rover_path[i + first_arm_waypoint];
            }
        }
        else
        {
            vw_reference_path.resize(this->vw_rover_path.size());
            for(uint i = 0; i < vw_reference_path.size(); i++)
            {
                vw_reference_path[i] = this->vw_rover_path[i];
            }
        }
        this->vvd_arm_motion_profile.clear();

        if(this->p_arm_planner->planArmMotion(&vw_reference_path,
                                              &elevationMap,
                                              this->pmm_map->getResolution(),
                                              this->d_zres,
                                              this->pmm_map->getSample(),
                                              &(this->vvd_arm_motion_profile),
                                              this->p_collision_detector))
        {
            // std::cout << " Done " << std::endl;
            //  Initialization
            if(this->isArmProfileSafe(this->vvd_arm_motion_profile))
            {
                if(minimize_arm_movement)
                {
                    std::cout << "[MM] \033[35m[----------] "
                                 "[MotionPlan::computeArmProfilePlanning()]\033[0m Rover Reference "
                                 "Path waypoints = "
                              << vw_reference_path.size() << std::endl;
                    this->vw_rover_path.resize(first_arm_waypoint);
                    this->vw_rover_path.insert(this->vw_rover_path.end(),
                                               vw_reference_path.begin(),
                                               vw_reference_path.end());
                    for(uint i = 0; i < first_arm_waypoint; i++)
                    {
                        this->vw_rover_path[i].position[2] =
                            elevationMap[(int)(this->vw_rover_path[i].position[1] /
                                                   this->pmm_map->getResolution() +
                                               0.5)][(int)(this->vw_rover_path[i].position[0] /
                                                               this->pmm_map->getResolution() +
                                                           0.5)] +
                            p_arm_planner->heightGround2BCS;
                        this->vvd_arm_motion_profile.insert(this->vvd_arm_motion_profile.begin(),
                                                            this->vvd_arm_motion_profile[0]);
                    }
                }
                else
                {
                    std::cout << "[MM] \033[35m[----------] "
                                 "[MotionPlan::computeArmProfilePlanning()]\033[0m Rover Reference "
                                 "Path waypoints = "
                              << vw_reference_path.size() << std::endl;
                    this->vw_rover_path.resize(vw_reference_path.size());
                    for(uint i = 0; i < vw_reference_path.size(); i++)
                    {
                        this->vw_rover_path[i] = vw_reference_path[i];
                    }
                }
                std::cout << "[MM] \033[35m[----------] "
                             "[MotionPlan::computeArmProfilePlanning()]\033[0m first_arm_waypoint"
                          << first_arm_waypoint << std::endl;
                std::cout
                    << "[MM] \033[35m[----------] [MotionPlan::computeArmProfilePlanning()]\033[0m "
                       "Rover Path waypoints = "
                    << this->vw_rover_path.size() << std::endl;
                std::cout
                    << "[MM] \033[35m[----------] [MotionPlan::computeArmProfilePlanning()]\033[0m "
                       "Raw Profile is safe, with "
                    << this->vvd_arm_motion_profile.size() << " samples" << std::endl;
                if(this->vvd_arm_motion_profile.size() > this->ui_gauss_numsamples * 2)
                {
                    this->p_arm_planner->computeArmProfileGaussSmoothening(
                        &(this->vvd_arm_motion_profile),
                        &(this->vvd_smoothed_arm_motion_profile),
                        this->d_gauss_sigma,
                        this->ui_gauss_numsamples);
                    if(this->isArmProfileSafe(this->vvd_smoothed_arm_motion_profile))
                    {
                        this->vvd_arm_motion_profile.clear();
                        std::vector<double> row;
                        for(uint j = 0; j < vvd_smoothed_arm_motion_profile.size(); j++)
                        {
                            for(uint i = 0; i < vvd_smoothed_arm_motion_profile[0].size(); i++)
                            {
                                row.push_back(vvd_smoothed_arm_motion_profile[j][i]);
                            }
                            this->vvd_arm_motion_profile.push_back(row);
                            row.clear();
                        }
                        this->vvd_arm_motion_profile = this->vvd_smoothed_arm_motion_profile;
                    }
                }
                std::cout << "[MM] \033[1;35m[----------] "
                             "[MotionPlan::computeArmProfilePlanning()]\033[0m Arm Motion Profile "
                             "Computed and Smoothed"
                          << std::endl;
                return 0;
            }
            else
            {
                std::cout << "[MM] \033[1;31m[----------] "
                             "[MotionPlan::computeArmProfilePlanning()]\033[0m Raw Profile is not "
                             "safe, with "
                          << this->vvd_arm_motion_profile.size() << " samples" << std::endl;
                if(minimize_arm_movement)
                {
                    std::cout
                        << "[MM] \033[1;31m[----------] "
                           "[MotionPlan::computeArmProfilePlanning()]\033[0m Deployment policy "
                        << ui_deployment << " not valid" << std::endl;
                    if(ui_deployment > 0)
                    {
                        ui_deployment -= 1;
                        std::cout << "[MM] \033[1;31m[----------] "
                                     "[MotionPlan::computeArmProfilePlanning()]\033[0m Trying with "
                                     "deployment "
                                  << ui_deployment << std::endl;
                        this->setDeployment(ui_deployment);
                        std::cout << "[MM] \033[35m[----------] "
                                     "[MotionPlan::computeArmProfilePlanning()]\033[0m Trying "
                                     "again witht Tunnel from the beginning"
                                  << std::endl;
                        minimize_arm_movement = false;
                    }
                    else
                    {
                        std::cout << "[MM] \033[35m[----------] "
                                     "[MotionPlan::computeArmProfilePlanning()]\033[0m Arm Profile "
                                     "could be not computed using any of the deployment approaches"
                                  << std::endl;
                        return 1;
                    }
                }
                else
                {
                    std::cout << "[MM] \033[35m[----------] "
                                 "[MotionPlan::computeArmProfilePlanning()]\033[0m Now trying with "
                                 "minimization of arm movements"
                              << std::endl;
                    minimize_arm_movement = true;
                }
            }
        }
        else
        {
            if(minimize_arm_movement)
            {
                std::cout << "[MM] \033[1;31m[----------] "
                             "[MotionPlan::computeArmProfilePlanning()]\033[0m Deployment policy "
                          << ui_deployment << " not valid" << std::endl;
                if(ui_deployment > 0)
                {
                    ui_deployment -= 1;
                    std::cout << "[MM] \033[1;31m[----------] "
                                 "[MotionPlan::computeArmProfilePlanning()]\033[0m Trying with "
                                 "deployment "
                              << ui_deployment << std::endl;
                    this->setDeployment(ui_deployment);
                    std::cout << "[MM] \033[35m[----------] "
                                 "[MotionPlan::computeArmProfilePlanning()]\033[0m Trying again "
                                 "witht Tunnel from the beginning"
                              << std::endl;
                    minimize_arm_movement = false;
                }
                else
                {
                    std::cout << "[MM] \033[35m[----------] "
                                 "[MotionPlan::computeArmProfilePlanning()]\033[0m Arm Profile "
                                 "could be not computed using any of the deployment approaches"
                              << std::endl;
                    return 2;
                }
            }
            else
            {
                std::cout
                    << "[MM] \033[35m[----------] [MotionPlan::computeArmProfilePlanning()]\033[0m "
                       "Now trying with minimization of arm movements"
                    << std::endl;
                minimize_arm_movement = true;
            }
        }
    }

    return 2;
}

unsigned int MotionPlan::computeArmDeployment(const base::Waypoint & w_goal,
                                              const std::vector<double> & vd_orientation_goal,
                                              const std::vector<double> & vd_arm_readings)
{
    this->vvd_init_arm_profile.clear();
    this->vd_init_time_profile.clear();
    this->b_is_initialization_computed = false;

    double dxyres = this->pmm_map->getResolution();
    if((dxyres <= 0.0) || (this->d_zres <= 0.0))
    {
        std::cout << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m "
                     "Resolution values are not valid"
                  << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m XY res = "
            << dxyres << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m Z res = "
            << this->d_zres << std::endl;
        return 3;
    }

    if(((w_goal.position[1] >= 0.0) && (vd_arm_readings[0] <= 0.0)) ||
       ((w_goal.position[1] <= 0.0) && (vd_arm_readings[0] >= 0.0)))
    {
        std::cout << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m Goal "
                     "position hemisphere is not the same as arm's "
                  << std::endl;
        return 4;
    }

    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeArmDeployment()]\033[0m Starting "
                 "to plan the arm deployment"
              << std::endl;
    if(this->p_arm_planner->planAtomicOperation(dxyres,
                                                this->d_zres,
                                                vd_arm_readings,
                                                w_goal,
                                                vd_orientation_goal,
                                                &(this->vvd_init_arm_profile),
                                                &(this->vd_init_time_profile)))
    {
        if(this->vvd_init_arm_profile.empty())
        {
            this->vvd_init_arm_profile.push_back(vd_arm_readings);
            this->vd_init_time_profile.push_back(0.0);
        }
        if(this->isArmProfileSafe(this->vvd_init_arm_profile))
        {
            // std::cout << "The size of the deployment profile is "  <<
            // this->vvd_init_arm_profile.size() << std::endl;
            this->b_is_initialization_computed = true;
            std::cout << "[MM] \033[1;35m[----------] [MotionPlan::computeArmDeployment()]\033[0m "
                         "The arm deployment is calculated with a profile size of "
                      << this->vvd_init_arm_profile.size() << " samples" << std::endl;
            return 0;
        }
        else
        {
            std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmDeployment()]\033[0m "
                         "Calculated profile for arm deployment is not safe"
                      << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmDeployment()]\033[0m Arm "
                     "deployment computation not feasible"
                  << std::endl;
        return 2;
    }
}

unsigned int MotionPlan::computeArmDeployment(const std::vector<double> & vd_arm_goal,
                                              const std::vector<double> & vd_arm_readings)
{
    this->vvd_init_arm_profile.clear();
    this->vd_init_time_profile.clear();
    double dxyres = this->pmm_map->getResolution();

    if((dxyres <= 0.0) || (this->d_zres <= 0.0))
    {
        std::cout << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m "
                     "Resolution values are not valid"
                  << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m XY res = "
            << dxyres << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m Z res = "
            << this->d_zres << std::endl;
        return 3;
    }

    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeArmDeployment()]\033[0m Starting "
                 "to plan the arm deployment"
              << std::endl;
    if(this->p_arm_planner->planAtomicOperation(dxyres,
                                                this->d_zres,
                                                vd_arm_readings,
                                                vd_arm_goal,
                                                &(this->vvd_init_arm_profile),
                                                &(this->vd_init_time_profile)))
    {
        if(this->isArmProfileSafe(this->vvd_init_arm_profile))
        {
            std::cout << "[MM] \033[1;35m[----------] [MotionPlan::computeArmDeployment()]\033[0m "
                         "The arm deployment is calculated with a profile size of "
                      << this->vvd_init_arm_profile.size() << " samples" << std::endl;
            this->b_is_initialization_computed = true;
            return 0;
        }
        else
        {
            std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmDeployment()]\033[0m "
                         "Calculated profile for arm deployment is not safe"
                      << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmDeployment()]\033[0m Arm "
                     "deployment computation not feasible"
                  << std::endl;
        return 2;
    }
}

unsigned int MotionPlan::computeArmDeployment(int i_segment_m,    // TODO - Remove this...
                                              const std::vector<double> & vd_arm_readings)
{
    this->vvd_init_arm_profile.clear();
    this->b_is_initialization_computed = false;
    // Here we deploy not to 0, but to a more advanced segment
    i_segment_m = min(0, (int)(this->vvd_arm_motion_profile.size() - 1));
    // i_segment_m = min(40, (int)(this->vvd_arm_motion_profile.size()-1));

    // TODO: check if elevationMap is properly formatted
    // TODO: check if i_segment_m is valid!!
    // std::cout << "The segment is " << i_segment_m << std::endl;
    /*for (uint i = 0; i < 6; i++)
    {
        std::cout << " Actual Joint " << i << " is " << vd_arm_readings[i]
                  << std::endl;
    }
    for (uint i = 0; i < 6; i++)
    {
        std::cout << " Goal Joint " << i << " is "
                  << this->vvd_arm_motion_profile[i_segment_m][i] << std::endl;
    }*/

    double dxyres = this->pmm_map->getResolution();
    if((dxyres <= 0.0) || (this->d_zres <= 0.0))
    {
        std::cout << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m "
                     "Resolution values are not valid"
                  << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m XY res = "
            << dxyres << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmDeployment()]\033[0m Z res = "
            << this->d_zres << std::endl;
        return 3;
    }

    if(this->p_arm_planner->planAtomicOperation(dxyres,
                                                this->d_zres,
                                                vd_arm_readings,
                                                this->vvd_arm_motion_profile[i_segment_m],
                                                &(this->vvd_init_arm_profile),
                                                &(this->vd_init_time_profile)))
    {
        if(this->isArmProfileSafe(this->vvd_init_arm_profile))
        {
            std::cout << "[MM] \033[1;35m[----------] [MotionPlan::computeArmDeployment()]\033[0m "
                         "The arm deployment is calculated with a profile size of "
                      << this->vvd_init_arm_profile.size() << " samples" << std::endl;
            this->b_is_initialization_computed = true;
            return 0;
        }
        else
        {
            std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmDeployment()]\033[0m "
                         "Calculated profile for arm deployment is not safe"
                      << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmDeployment()]\033[0m Arm "
                     "deployment computation not feasible"
                  << std::endl;
        return 2;
    }
}

unsigned int MotionPlan::computeArmRetrieval(const std::vector<double> & vd_init, int mode)
{
    // mode: 0 = atomic, 1 = coupled
    this->vvd_retrieval_arm_profile.clear();
    this->b_is_retrieval_computed = false;
    double dxyres = this->pmm_map->getResolution();
    if((dxyres <= 0.0) || (this->d_zres <= 0.0))
    {
        std::cout << "[MM] \033[1;31m[----------] [MotionPlan::computeArmRetrieval()]\033[0m "
                     "Resolution values are not valid"
                  << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmRetrieval()]\033[0m XY res = "
            << dxyres << std::endl;
        std::cout
            << "[MM] \033[1;31m[----------] [MotionPlan::computeArmRetrieval()]\033[0m Z res = "
            << this->d_zres << std::endl;
        return 3;
    }

    std::cout << "[MM] \033[35m[----------] [MotionPlan::computeArmRetrieval()]\033[0m Starting to "
                 "plan the arm retrieval"
              << std::endl;
    if(this->p_arm_planner->planAtomicOperation(dxyres,
                                                this->d_zres,
                                                vd_init,
                                                this->vd_retrieval_position,
                                                &(this->vvd_retrieval_arm_profile),
                                                &(this->vd_retrieval_time_profile),
                                                mode))
    {
        if(this->vvd_retrieval_arm_profile.empty())
        {
            this->vvd_retrieval_arm_profile.push_back(vd_init);
            this->vd_retrieval_time_profile.push_back(0.0);
        }
        if(this->isArmProfileSafe(this->vvd_retrieval_arm_profile))
        {
            std::cout << "[MM] \033[1;35m[----------] [MotionPlan::computeArmRetrieval()]\033[0m "
                         "The arm retrieval is calculated with a profile size of "
                      << this->vvd_retrieval_arm_profile.size() << " samples" << std::endl;
            this->b_is_retrieval_computed = true;
            return 0;
        }
        else
        {
            std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmRetrieval()]\033[0m "
                         "Calculated profile for arm retrieval is not safe"
                      << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "[MM] \033[1;31m[--ERROR---] [MotionPlan::computeArmRetrieval()]\033[0m Arm "
                     "retrieval computation not feasible"
                  << std::endl;
        return 2;
    }
}

unsigned int MotionPlan::computeAtomicOperation()
{
    this->vvd_arm_motion_profile.clear();

    this->vw_rover_path.clear();

    std::vector<std::vector<double>> elevationMap;
    this->pmm_map->getElevationMapToZero(elevationMap);

    // TODO - Change this function to use real rover pose, initial and goal EE
    // poses
    std::vector<double> goalArmConfiguration = {
        0.797367, -1.5487, 2.48548, -2.23731, 1.14413, -0.484318};
    std::vector<double> initialArmConfiguration = {
        0.367174, -0.206004, 1.13099, 0, 0.645814, 0.367174};

    base::Waypoint goal_waypoint;
    goal_waypoint.position[0] = 1;
    goal_waypoint.position[1] = 1;
    goal_waypoint.position[2] = 1;

    std::vector<double> goal_orientation = {0, 0, 0};

    if(this->p_arm_planner->planAtomicOperation(this->pmm_map->getResolution(),
                                                this->d_zres,
                                                initialArmConfiguration,
                                                goalArmConfiguration,
                                                &(this->vvd_arm_motion_profile),
                                                &(this->vd_atomic_time_profile)))
    {
        if(this->isArmProfileSafe(this->vvd_arm_motion_profile))
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 2;
    }
}

bool MotionPlan::isArmProfileSafe(const std::vector<std::vector<double>> & vvd_profile_m)
{
    for(unsigned int i = 0; i < vvd_profile_m.size(); i++)
    {
        /*std::cout << " Joints = " << vvd_profile_m[i][0];
            std::cout << " " << vvd_profile_m[i][1];
            std::cout << " " << vvd_profile_m[i][2];
            std::cout << " " << vvd_profile_m[i][3];
            std::cout << " " << vvd_profile_m[i][4];
            std::cout << " " << vvd_profile_m[i][5];*/
        if(this->p_collision_detector->isColliding(vvd_profile_m[i]))
        {
            std::cout << "ERROR at sample " << i << std::endl;
            std::cout << " Joints = " << vvd_profile_m[i][0];
            std::cout << " " << vvd_profile_m[i][1];
            std::cout << " " << vvd_profile_m[i][2];
            std::cout << " " << vvd_profile_m[i][3];
            std::cout << " " << vvd_profile_m[i][4];
            std::cout << " " << vvd_profile_m[i][5];
            return false;
        }
    }
    return true;
}

std::vector<base::Waypoint> * MotionPlan::getRoverPath()
{
    return &(this->vw_rover_path);
}

unsigned int MotionPlan::getNumberWaypoints()
{
    return this->vw_rover_path.size();
}

unsigned int MotionPlan::getNumberDeploymentSamples()
{
    return this->vvd_init_arm_profile.size();
}

unsigned int MotionPlan::getNumberRetrievalSamples()
{
    return this->vvd_retrieval_arm_profile.size();
}

std::vector<std::vector<double>> * MotionPlan::getWristPath()
{
    return this->p_arm_planner->getWristPath();
}

std::vector<std::vector<double>> * MotionPlan::getCoupledArmMotionProfile()
{
    return &(this->vvd_arm_motion_profile);
}

std::vector<double> * MotionPlan::getBackArmMotionProfile()
{
    return &(this->vvd_arm_motion_profile.back());
}

std::vector<std::vector<double>> * MotionPlan::getInitArmMotionProfile()
{
    return &(this->vvd_init_arm_profile);
}

std::vector<double> * MotionPlan::getBackInitArmMotionProfile()
{
    return &(this->vvd_init_arm_profile.back());
}

bool MotionPlan::isInitArmMotionProfileEmpty()
{
    return this->vvd_init_arm_profile.empty();
}

std::vector<double> * MotionPlan::getInitArmTimeProfile()
{
    return &(this->vd_init_time_profile);
}

std::vector<std::vector<double>> * MotionPlan::getRetrievalArmMotionProfile()
{
    return &(this->vvd_retrieval_arm_profile);
}

std::vector<double> * MotionPlan::getRetrievalArmTimeProfile()
{
    return &(this->vd_retrieval_time_profile);
}

std::vector<double> * MotionPlan::getAtomicTimeProfile()
{
    return &(this->vd_atomic_time_profile);
}

std::vector<std::vector<std::vector<double>>> * MotionPlan::get3DCostMap()
{
    return this->p_arm_planner->getVolumeCostMap();
}
