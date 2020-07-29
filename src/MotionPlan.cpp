#include "MotionPlan.h"

MotionPlan::MotionPlan(MobileManipMap *pmmmap_m,
                       double d_zres_m,
                       std::string s_urdf_path_m)
{
    this->pmm_map = pmmmap_m;
    this->d_zres = d_zres_m;
    this->s_urdf_path = s_urdf_path_m;
    this->p_arm_planner = new ArmPlanner(s_urdf_path_m, false, 0);
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m);
    this->b_is_retrieval_computed = false;
    this->b_is_initialization_computed = false;

    this->vd_retrieval_position.resize(6);
    this->vd_retrieval_position[0] = 1.571;
    this->vd_retrieval_position[1] = -1.83;
    this->vd_retrieval_position[2] = 2.79;
    this->vd_retrieval_position[3] = 0.0;
    this->vd_retrieval_position[4] = -0.5;
    this->vd_retrieval_position[5] = 2.354;
}

MotionPlan::MotionPlan(MobileManipMap *pmmmap_m,
                       double d_zres_m,
                       std::string s_urdf_path_m,
                       std::vector<Waypoint> &vw_rover_path_m,
                       std::vector<std::vector<double>> &m_arm_motion_profile)
{
    this->pmm_map = pmmmap_m;
    this->d_zres = d_zres_m;
    this->s_urdf_path = s_urdf_path_m;
    this->vw_rover_path.clear();
    this->p_arm_planner = new ArmPlanner(s_urdf_path_m, false, 1);
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m);
    for (uint i = 0; i < vw_rover_path_m.size(); i++)
    {
        this->vw_rover_path.push_back(vw_rover_path_m[i]);
    }

    std::vector<double> row;
    this->vvd_arm_motion_profile.clear();
    for (uint j = 0; j < m_arm_motion_profile.size(); j++)
    {
        for (uint i = 0; i < m_arm_motion_profile[0].size(); i++)
        {
            row.push_back(m_arm_motion_profile[j][i]);
        }
        this->vvd_arm_motion_profile.push_back(row);
        row.clear();
    }
}

unsigned int MotionPlan::computeRoverBasePathPlanning(
    base::Waypoint rover_position)
{
    std::vector<std::vector<double>> costMap;
    if (this->pmm_map->isOutside(rover_position))
    {
        return 1;
    }
    if (this->pmm_map->isObstacle(rover_position))
    {
        return 2;
    }
    if (!this->pmm_map->isSampleLoaded())
    {
        return 3;
    }
    this->pmm_map->getCostMap(costMap);
    //    std::cout << "Rover pos is " << rover_position.position[0] << ", " <<
    //    rover_position.position[1] << std::endl;
    this->w_rover_pos = rover_position;
    if (this->bi_fast_marching.planPath(&costMap,
                                        this->pmm_map->getResolution(),
                                        rover_position,
                                        this->pmm_map->getSample(),
                                        &(this->vw_rover_path)))
    {
        if (isSmoothPath())
        {
            return 0;
        }
        else
        {
            return 5;
        }
    }
    else
    {
        return 4;
    }
}

bool MotionPlan::isSmoothPath()
{
    if (this->vw_rover_path.size() < 3)
    {
        return false;
    }
    double d_segmentX1, d_segmentY1, d_segmentX2, d_segmentY2, d_norm1, d_norm2,
        d_diffheading;
    for (int i = 2; i < vw_rover_path.size() - 1;
         i++) // Final Waypoint is sometimes tricky due to FM computation, so it
              // is left anyways
    {
        d_segmentX1 = this->vw_rover_path[i].position[0]
                      - this->vw_rover_path[i - 1].position[0];
        d_segmentX2 = this->vw_rover_path[i - 1].position[0]
                      - this->vw_rover_path[i - 2].position[0];
        d_segmentY1 = this->vw_rover_path[i].position[1]
                      - this->vw_rover_path[i - 1].position[1];
        d_segmentY2 = this->vw_rover_path[i - 1].position[1]
                      - this->vw_rover_path[i - 2].position[1];
        d_norm1 = sqrt(pow(d_segmentX1, 2) + pow(d_segmentY1, 2));
        d_norm2 = sqrt(pow(d_segmentX2, 2) + pow(d_segmentY2, 2));
        d_diffheading
            = acos((d_segmentX1 * d_segmentX2 + d_segmentY1 * d_segmentY2)
                   / (d_norm1 * d_norm2))
              * 180.0 / 3.1416;
        if (d_diffheading > 45.0)
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
    int endWaypoint = this->fetching_pose_estimator.getFetchWaypointIndex(
        &(this->vw_rover_path));
    if (endWaypoint == 0)
    {
        if (this->vw_rover_path.size() > 2)
        {
            this->vw_rover_path.erase(this->vw_rover_path.begin() + 1,
                                      this->vw_rover_path.end());
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        int i_eraseIndex = endWaypoint + 1;
        // std::cout << "The path size is " << this->vw_rover_path.size() <<
        // std::endl; std::cout << "The endWaypoint is " << endWaypoint <<
        // std::endl;
        if (endWaypoint < this->vw_rover_path.size() - 3)
        {
            double d_segmentX = this->vw_rover_path[i_eraseIndex].position[0]
                                - this->vw_rover_path[endWaypoint].position[0];
            double d_segmentY = this->vw_rover_path[i_eraseIndex].position[1]
                                - this->vw_rover_path[endWaypoint].position[1];
            double d_norm = sqrt(pow(d_segmentX, 2) + pow(d_segmentY, 2));
            while ((i_eraseIndex < this->vw_rover_path.size() - 2)
                   && (d_norm < 0.1)) // TODO-Introduce here configurable
                                      // position tolerance
            {
                i_eraseIndex += 1;
                d_segmentX = this->vw_rover_path[i_eraseIndex].position[0]
                             - this->vw_rover_path[endWaypoint].position[0];
                d_segmentY = this->vw_rover_path[i_eraseIndex].position[1]
                             - this->vw_rover_path[endWaypoint].position[1];
                d_norm = sqrt(pow(d_segmentX, 2) + pow(d_segmentY, 2));
            }
            std::cout << "The erase index is " << i_eraseIndex << std::endl;
            this->vw_rover_path.erase(this->vw_rover_path.begin()
                                          + i_eraseIndex,
                                      this->vw_rover_path.end());
        }
        else
        {
            this->vw_rover_path.erase(this->vw_rover_path.begin() + endWaypoint
                                          + 1,
                                      this->vw_rover_path.end());
        }
        // Here we shorten the path as well at the beginning
        /*int i_path_index = 0;
        if (this->vw_rover_path.size()>3)
        {
            double d_segmentX = this->vw_rover_path[i_path_index].position[0] -
        this->vw_rover_path[0].position[0]; double d_segmentY =
        this->vw_rover_path[i_path_index].position[1] -
        this->vw_rover_path[0].position[1]; double d_norm =
        sqrt(pow(d_segmentX,2)+pow(d_segmentY,2)); while ((i_path_index <
        this->vw_rover_path.size() - 2)&&(d_norm < 0.5))//TODO-Introduce here
        configurable position tolerance
            {
                    i_path_index += 1;
                    d_segmentX = this->vw_rover_path[i_path_index].position[0] -
        this->vw_rover_path[0].position[0]; d_segmentY =
        this->vw_rover_path[i_path_index].position[1] -
        this->vw_rover_path[0].position[1]; d_norm =
        sqrt(pow(d_segmentX,2)+pow(d_segmentY,2));
            }
            std::cout << "The initial erase index is " << i_path_index <<
        std::endl; this->vw_rover_path.erase(this->vw_rover_path.begin(),
                                  this->vw_rover_path.begin() + i_path_index);

        }*/
    }
    return true;
}

void MotionPlan::addTurningWaypoint(double d_dev)
{
    if (this->vw_rover_path.size() <=2)
    {
        d_dev /= 2.0;
    }

    base::Waypoint w_end, w_sample, w_turn;
    w_end = this->vw_rover_path.back();
    w_sample = this->pmm_map->getSample();
    std::cout << "WAYPOINT END DATA: " <<  w_end.position[0] << " m, " << w_end.position[1] << " m, " << w_end.heading * 180.0/3.1416 << " deg" << std::endl;
    std::cout << "WAYPOINT SAMPLE DATA: " <<  w_sample.position[0] << " m, " << w_sample.position[1] << " m, " << w_sample.heading * 180.0/3.1416 << " deg" << std::endl;
    double dx,dy,dnx,dny;
    dx = w_sample.position[1] - w_end.position[1];
    dy = w_end.position[0] - w_sample.position[0];
    dnx = dx / sqrt(pow(dx,2) + pow(dy,2));
    dny = dy / sqrt(pow(dx,2) + pow(dy,2));
    w_turn.position[0] = w_sample.position[0] + dx*d_dev;
    w_turn.position[1] = w_sample.position[1] + dy*d_dev;
    //w_turn.heading = atan2(w_turn.position[1] - w_end.position[1], w_turn.position[0] - w_end.position[0]);
    w_turn.heading = w_end.heading;
    std::cout << "WAYPOINT TURN DATA: " <<  w_turn.position[0] << " m, " << w_turn.position[1] << " m, " << w_turn.heading * 180.0/3.1416 << " deg" << std::endl;
    this->vw_rover_path.push_back(w_turn);
}

unsigned int MotionPlan::computeArmProfilePlanning()
{
    // TODO - Create here several profiles: init, coupled, sweeping and
    // retrieval
    this->b_is_retrieval_computed = false;
    this->b_is_initialization_computed = false;
    if (!this->pmm_map->isSampleLoaded())
    {
        return 3;
    }
    std::vector<std::vector<double>> elevationMap;
    this->pmm_map->getElevationMapToZero(elevationMap);
    if (this->p_arm_planner->planArmMotion(&(this->vw_rover_path),
                                           &elevationMap,
                                           this->pmm_map->getResolution(),
                                           this->d_zres,
                                           this->pmm_map->getSample(),
                                           &(this->vvd_arm_motion_profile)))
    {
        std::cout << " Done " << std::endl;
        // Initialization
        if (this->isArmProfileSafe(this->vvd_arm_motion_profile))
        {
            std::cout << "Raw Profile is safe, with "
                      << this->vvd_arm_motion_profile.size() << " samples"
                      << std::endl;
            if (this->vvd_arm_motion_profile.size()
                > this->i_gauss_numsamples * 2)
            {
                this->p_arm_planner->computeArmProfileGaussSmoothening(
                    &(this->vvd_arm_motion_profile),
                    &(this->vvd_smoothed_arm_motion_profile),
                    this->d_gauss_sigma,
                    this->i_gauss_numsamples);
                if (this->isArmProfileSafe(
                        this->vvd_smoothed_arm_motion_profile))
                {
                    this->vvd_arm_motion_profile.clear();
                    std::vector<double> row;
                    for (uint j = 0; j < vvd_smoothed_arm_motion_profile.size();
                         j++)
                    {
                        for (uint i = 0;
                             i < vvd_smoothed_arm_motion_profile[0].size();
                             i++)
                        {
                            row.push_back(
                                vvd_smoothed_arm_motion_profile[j][i]);
                        }
                        this->vvd_arm_motion_profile.push_back(row);
                        row.clear();
                    }
                    this->vvd_arm_motion_profile
                        = this->vvd_smoothed_arm_motion_profile;
                }
            }
            return 0;
        }
        else
        {
            std::cout << "Raw Profile is not safe, with "
                      << this->vvd_arm_motion_profile.size() << " samples"
                      << std::endl;
            return 1;
        }
    }
    else
    {
        return 2;
    }
}

unsigned int MotionPlan::computeArmDeployment(
    const base::Waypoint &w_goal,
    const std::vector<double> &vd_orientation_goal,
    const std::vector<double> &vd_arm_readings)
{
    this->vvd_init_arm_profile.clear();
    this->vd_init_time_profile.clear();
    this->b_is_initialization_computed = false;
    std::vector<std::vector<double>> elevationMap;
    if (!this->pmm_map->getElevationMapToZero(elevationMap))
    {
        return 3;
    }

    double dxyres = this->pmm_map->getResolution();
    if ((dxyres <= 0.0)||(this->d_zres <= 0.0))
    {
        return 3;
    }

    if (((w_goal.position[1] >= 0.0)&&(vd_arm_readings[0] <= 0.0))||((w_goal.position[1] <= 0.0)&&(vd_arm_readings[0] >= 0.0)))
    {
        std::cout << "Goal position hemisphere is not the same as arm's" << std::endl;
        return 4; 
    }


    base::Waypoint current_waypoint;
    current_waypoint.position[0] = dxyres * (double)elevationMap[0].size() / 2.0; // TODO - Fix This
    current_waypoint.position[1] = dxyres * (double)elevationMap.size() / 2.0;
    current_waypoint.position[2]
        = elevationMap[(int)(current_waypoint.position[1]
                                 / this->pmm_map->getResolution()
                             + 0.5)][(int)(current_waypoint.position[0]
                                               / this->pmm_map->getResolution()
                                           + 0.5)]
          + p_arm_planner->heightGround2BCS;
    current_waypoint.heading = 0;
    std::cout << "About to compute the deployment" << std::endl;
    if (this->p_arm_planner->planAtomicOperation(
            dxyres,
            this->d_zres,
            vd_arm_readings,
            w_goal,
            vd_orientation_goal,
            &(this->vvd_init_arm_profile),
            &(this->vd_init_time_profile)))
    { // TODO - This may return a segmentation fault, maybe because of a non
      // initialized elevation map...
        if (this->isArmProfileSafe(this->vvd_init_arm_profile))
        {
            this->b_is_initialization_computed = true;
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

unsigned int MotionPlan::computeArmDeployment(
    const std::vector<double> &vd_arm_goal,
    const std::vector<double> &vd_arm_readings)
{
    this->vvd_init_arm_profile.clear();
    this->vd_init_time_profile.clear();
    this->b_is_initialization_computed = false;
    std::vector<std::vector<double>> elevationMap;
    if (!this->pmm_map->getElevationMapToZero(elevationMap))
    {
        return 3;
    }
    // TODO: check if elevationMap is properly formatted
    // TODO: check if i_segment_m is valid!!
    double dxyres = this->pmm_map->getResolution();
    std::cout << " Resolution is " << dxyres << std::endl;
    std::cout << " Z-res is " << this->d_zres << std::endl;
    base::Waypoint current_waypoint;
    current_waypoint.position[0] = 3; // TODO - Fix This
    current_waypoint.position[1] = 3;
    current_waypoint.position[2]
        = elevationMap[(int)(current_waypoint.position[1]
                                 / this->pmm_map->getResolution()
                             + 0.5)][(int)(current_waypoint.position[0]
                                               / this->pmm_map->getResolution()
                                           + 0.5)]
          + p_arm_planner->heightGround2BCS;
    current_waypoint.heading = 0;

    if (this->p_arm_planner->planAtomicOperation(
            dxyres,
            this->d_zres,
            vd_arm_readings,
            vd_arm_goal,
            &(this->vvd_init_arm_profile),
            &(this->vd_init_time_profile)))
    { // TODO - This may return a segmentation fault, maybe because of a non
      // initialized elevation map...
        if (this->isArmProfileSafe(this->vvd_init_arm_profile))
        {
            this->b_is_initialization_computed = true;
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

unsigned int MotionPlan::computeArmDeployment(
    int i_segment_m,
    const std::vector<double> &vd_arm_readings)
{
    this->vvd_init_arm_profile.clear();
    this->b_is_initialization_computed = false;
    std::vector<std::vector<double>> elevationMap;
    if (!this->pmm_map->getElevationMapToZero(elevationMap))
    {
        return 3;
    }
    // TODO: check if elevationMap is properly formatted
    // TODO: check if i_segment_m is valid!!
    std::cout << "The segment is " << i_segment_m << std::endl;
    for (uint i = 0; i < 6; i++)
    {
        std::cout << " Actual Joint " << i << " is " << vd_arm_readings[i]
                  << std::endl;
    }
    for (uint i = 0; i < 6; i++)
    {
        std::cout << " Goal Joint " << i << " is "
                  << this->vvd_arm_motion_profile[i_segment_m][i] << std::endl;
    }

    double dxyres = this->pmm_map->getResolution();
    std::cout << " Resolution is " << dxyres << std::endl;
    std::cout << " Z-res is " << this->d_zres << std::endl;

    if (this->p_arm_planner->planAtomicOperation(
            dxyres,
            this->d_zres,
            vd_arm_readings,
            this->vvd_arm_motion_profile[i_segment_m],
            &(this->vvd_init_arm_profile),
            &(this->vd_init_time_profile)))
    { // TODO - This may return a segmentation fault, maybe because of a non
      // initialized elevation map...
        if (this->isArmProfileSafe(this->vvd_init_arm_profile))
        {
            this->b_is_initialization_computed = true;
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

unsigned int MotionPlan::computeArmRetrieval(const std::vector<double> &vd_init)
{
    this->vvd_retrieval_arm_profile.clear();
    this->b_is_retrieval_computed = false;
    std::vector<std::vector<double>> elevationMap;
    if (!this->pmm_map->getElevationMapToZero(elevationMap))
    {
        return 3;
    }
    double dxyres = this->pmm_map->getResolution();
        if ((dxyres <= 0.0)||(this->d_zres <= 0.0))
    {
        return 3;
    }
    base::Waypoint current_waypoint;
    current_waypoint.position[0] = dxyres * (double)elevationMap[0].size() / 2.0; // TODO - Fix This
    current_waypoint.position[1] = dxyres * (double)elevationMap.size() / 2.0;
    current_waypoint.position[2]
        = elevationMap[(int)(current_waypoint.position[1]
                                 / this->pmm_map->getResolution()
                             + 0.5)][(int)(current_waypoint.position[0]
                                               / this->pmm_map->getResolution()
                                           + 0.5)]
          + p_arm_planner->heightGround2BCS;
    current_waypoint.heading = 0;

    if (this->p_arm_planner->planAtomicOperation(
            dxyres,
            this->d_zres,
            vd_init,
            this->vd_retrieval_position,
            &(this->vvd_retrieval_arm_profile),
            &(this->vd_retrieval_time_profile)))
    { // TODO - This may return a segmentation fault, maybe because of a non
      // initialized elevation map...
        if (this->isArmProfileSafe(this->vvd_retrieval_arm_profile))
        {
            this->b_is_retrieval_computed = true;
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

unsigned int MotionPlan::computeAtomicOperation()
{
    this->vvd_arm_motion_profile.clear();

    this->vw_rover_path.clear();

    std::vector<std::vector<double>> elevationMap;
    this->pmm_map->getElevationMapToZero(elevationMap);

    // TODO - Change this function to use real rover pose, initial and goal EE
    // poses
    std::vector<double> goalArmConfiguration
        = {0.797367, -1.5487, 2.48548, -2.23731, 1.14413, -0.484318};
    std::vector<double> initialArmConfiguration
        = {0.367174, -0.206004, 1.13099, 0, 0.645814, 0.367174};

    base::Waypoint goal_waypoint;
    goal_waypoint.position[0] = 1;
    goal_waypoint.position[1] = 1;
    goal_waypoint.position[2] = 1;

    std::vector<double> goal_orientation = {0, 0, 0};

    if (this->p_arm_planner->planAtomicOperation(
            this->pmm_map->getResolution(),
            this->d_zres,
            initialArmConfiguration,
            goalArmConfiguration,
            &(this->vvd_arm_motion_profile),
            &(this->vd_time_profile)))
    {
        if (this->isArmProfileSafe(this->vvd_arm_motion_profile))
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

bool MotionPlan::isArmProfileSafe(
    const std::vector<std::vector<double>> &vvd_profile_m)
{
    for (int i = 0; i < vvd_profile_m.size(); i++)
    {
        if (this->p_collision_detector->isColliding(vvd_profile_m[i]))
        {
            std::cout << "ERROR at sample " << i << std::endl;
            std::cout << " Joint 1 = " << vvd_profile_m[i][0];
            std::cout << " Joint 2 = " << vvd_profile_m[i][1];
            std::cout << " Joint 3 = " << vvd_profile_m[i][2];
            std::cout << " Joint 4 = " << vvd_profile_m[i][3];
            std::cout << " Joint 5 = " << vvd_profile_m[i][4];
            std::cout << " Joint 6 = " << vvd_profile_m[i][5];
            return false;
        }
    }
    return true;
}

std::vector<base::Waypoint> *MotionPlan::getRoverPath()
{
    return &(this->vw_rover_path);
}

unsigned int MotionPlan::getNumberWaypoints()
{
    return this->vw_rover_path.size();
}

std::vector<std::vector<double>> *MotionPlan::getWristPath()
{
    return this->p_arm_planner->getWristPath();
}

std::vector<std::vector<double>> *MotionPlan::getArmMotionProfile()
{
    return &(this->vvd_arm_motion_profile);
}

std::vector<double> *MotionPlan::getBackArmMotionProfile()
{
    return &(this->vvd_arm_motion_profile.back());
}

std::vector<std::vector<double>> *MotionPlan::getInitArmMotionProfile()
{
    return &(this->vvd_init_arm_profile);
}

std::vector<double> *MotionPlan::getBackInitArmMotionProfile()
{
    return &(this->vvd_init_arm_profile.back());
}

std::vector<double> *MotionPlan::getInitArmTimeProfile()
{
    return &(this->vd_init_time_profile);
}

std::vector<std::vector<double>> *MotionPlan::getRetrievalArmMotionProfile()
{
    return &(this->vvd_retrieval_arm_profile);
}

std::vector<double> *MotionPlan::getRetrievalArmTimeProfile()
{
    return &(this->vd_retrieval_time_profile);
}

std::vector<double> *MotionPlan::getTimeProfile()
{
    return &(this->vd_time_profile);
}

std::vector<std::vector<std::vector<double>>> *MotionPlan::get3DCostMap()
{
    return this->p_arm_planner->getVolumeCostMap();
}

void MotionPlan::setArmGaussFilter(double sigma, int numsamples)
{
    if (sigma > 0.0)
    {
        this->d_gauss_sigma = sigma;
    }
    if (numsamples > 0) // TODO - Check if odd number
    {
        this->i_gauss_numsamples = numsamples;
    }
}

bool MotionPlan::isRetrievalComputed()
{
    return b_is_retrieval_computed;
}

bool MotionPlan::isInitializationComputed()
{
    return b_is_initialization_computed;
}
