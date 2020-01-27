#include "MotionPlan.h"

MotionPlan::MotionPlan(MobileManipMap * pmmmap_m, double d_zres_m)
{
    this->pmm_map = pmmmap_m;
    this->d_zres = d_zres_m;
}

MotionPlan::MotionPlan(std::vector<Waypoint> &vw_rover_path_m,
                       std::vector<std::vector<double>> &m_arm_motion_profile)
{
    this->vw_rover_path.clear();
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

unsigned int MotionPlan::executeRoverBasePathPlanning(base::Waypoint rover_position,
                                              base::Waypoint sample)
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
    if (this->pmm_map->isOutside(sample))
    {
        return 3;
    }
    if (this->pmm_map->isObstacle(sample))
    {
        return 4;
    }
    this->pmm_map->getCostMap(costMap);
    this->bi_fast_marching.planPath(&costMap,
                                    this->pmm_map->getResolution(),
                                    rover_position,
                                    sample,
                                    &(this->vw_rover_path));
    this->w_sample_pos = sample;
    return 0;
}

bool MotionPlan::shortenPathForFetching()
{
    //TODO- Make this cut a shorter distance, taking into account tol_position from Waypoint Navigation
    int endWaypoint = this->fetching_pose_estimator.getFetchWaypointIndex(
        &(this->vw_rover_path));
    if (endWaypoint == 0)
    {
        return false;
    }
    else
    {
        this->vw_rover_path.erase(this->vw_rover_path.begin() + endWaypoint + 1,
                              this->vw_rover_path.end());
    }
    return true;
}

void MotionPlan::executeEndEffectorPlanning()
{
    this->vvd_arm_motion_profile.clear();
    std::vector<std::vector<double>> elevationMap;
    this->pmm_map->getElevationMap(elevationMap);
    this->arm_planner.planArmMotion(&(this->vw_rover_path),
                                    &elevationMap,
                                    this->pmm_map->getResolution(),
                                    this->d_zres,
                                    this->w_sample_pos,
                                    &(this->vvd_arm_motion_profile));
}

std::vector<base::Waypoint> *MotionPlan::getRoverPath()
{
    return &(this->vw_rover_path);
}

std::vector<std::vector<double>> *MotionPlan::getEndEffectorPath()
{
    return this->arm_planner.getEEPath();
}

std::vector<std::vector<double>> *MotionPlan::getArmMotionProfile()
{
    return &(this->vvd_arm_motion_profile);
}

std::vector<std::vector<std::vector<double>>> * MotionPlan::get3DCostMap()
{
    return this->arm_planner.getVolumeCostMap();
}
