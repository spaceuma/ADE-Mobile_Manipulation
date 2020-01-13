#include "MotionPlan.h"

MotionPlan::MotionPlan()
{
    // TODO - implement MotionPlan::MotionPlan
}

void MotionPlan::updateMotionPlan(std::vector<Waypoint> newRoverPath,
                                  std::vector<Joints> newJointsProfile)
{
    // TODO - implement MotionPlan::updateMotionPlan
    this->vw_rover_path = newRoverPath;
    this->vj_joints_profile = newJointsProfile;
}

void MotionPlan::executeRoverBasePathPlanning(MobileManipMap *inputMap,
                                              base::Waypoint rover_position,
                                              base::Waypoint sample)
{
    std::vector<std::vector<double>> costMap;
    // inputMap->getSamplingCostMap( costMap, sample );
    inputMap->getCostMap(costMap);
    this->bi_fast_marching.planPath(&costMap,
                                    inputMap->getResolution(),
                                    rover_position,
                                    sample,
                                    &(this->vw_rover_path));
    this->w_sample_pos = sample;
}

int MotionPlan::shortenPathForFetching()
{
    // FetchingPoseEstimator_lib::FetchingPoseEstimator dummyFetchPosePlanner;
    int endWaypoint = this->fetching_pose_estimator.getFetchWaypointIndex(
        &(this->vw_rover_path));
    this->vw_rover_path.erase(this->vw_rover_path.begin() + endWaypoint + 1,
                              this->vw_rover_path.end());
    return endWaypoint;
}

void MotionPlan::executeEndEffectorPlanning(MobileManipMap *inputMap,
                                            double zResolution)
{
    this->vvd_arm_motion_profile.clear();
    std::vector<std::vector<double>> elevationMap;
    inputMap->getElevationMap(elevationMap);
    this->arm_planner.planArmMotion(&(this->vw_rover_path),
                                    &elevationMap,
                                    inputMap->getResolution(),
                                    zResolution,
                                    this->w_sample_pos,
                                    &(this->vvd_arm_motion_profile));
}

std::vector<base::Waypoint> *MotionPlan::getPath()
{
    return &(this->vw_rover_path);
}

std::vector<std::vector<double>> *MotionPlan::getArmMotionProfile()
{
    return &(this->vvd_arm_motion_profile);
}

void MotionPlan::setArmMotionProfile(
    std::vector<std::vector<double>> &m_arm_motion_profile)
{
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

void MotionPlan::setPath(std::vector<Waypoint> &vw_path)
{
    this->vw_rover_path.clear();
    for (uint i = 0; i < vw_path.size(); i++)
    {
        this->vw_rover_path.push_back(vw_path[i]);
    }
}
