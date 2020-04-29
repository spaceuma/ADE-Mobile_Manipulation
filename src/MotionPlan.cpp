#include "MotionPlan.h"

MotionPlan::MotionPlan(MobileManipMap * pmmmap_m, double d_zres_m, std::string s_urdf_path_m)
{
    this->pmm_map = pmmmap_m;
    this->d_zres = d_zres_m;
    this->s_urdf_path = s_urdf_path_m;
    this->p_arm_planner = new ArmPlanner(s_urdf_path_m,false,0); 
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m); 
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

unsigned int MotionPlan::computeRoverBasePathPlanning(base::Waypoint rover_position)
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
    std::cout << "Rover pos is " << rover_position.position[0] << ", " << rover_position.position[1] << std::endl;
    if(this->bi_fast_marching.planPath(&costMap,
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
    double d_segmentX1, d_segmentY1, d_segmentX2, d_segmentY2, d_norm1, d_norm2, d_diffheading;
    for (int i = 2; i < vw_rover_path.size()-1; i++) // Final Waypoint is sometimes tricky due to FM computation, so it is left anyways
    {
        d_segmentX1 = this->vw_rover_path[i].position[0] - this->vw_rover_path[i-1].position[0]; 
        d_segmentX2 = this->vw_rover_path[i-1].position[0] - this->vw_rover_path[i-2].position[0]; 
        d_segmentY1 = this->vw_rover_path[i].position[1] - this->vw_rover_path[i-1].position[1];
        d_segmentY2 = this->vw_rover_path[i-1].position[1] - this->vw_rover_path[i-2].position[1];
	d_norm1 = sqrt(pow(d_segmentX1,2)+pow(d_segmentY1,2));
	d_norm2 = sqrt(pow(d_segmentX2,2)+pow(d_segmentY2,2));
        d_diffheading = acos((d_segmentX1*d_segmentX2 + d_segmentY1*d_segmentY2)/(d_norm1*d_norm2))*180.0/3.1416;
        if (d_diffheading > 45.0)
        {
            return false; 
        } 
    }
    return true; 
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
        int i_eraseIndex = endWaypoint + 1;
	std::cout << "The path size is " << this->vw_rover_path.size() << std::endl;
	std::cout << "The endWaypoint is " << endWaypoint << std::endl;
	if (endWaypoint < this->vw_rover_path.size()-3)
        {
	    double d_segmentX = this->vw_rover_path[i_eraseIndex].position[0] - this->vw_rover_path[endWaypoint].position[0];
	    double d_segmentY = this->vw_rover_path[i_eraseIndex].position[1] - this->vw_rover_path[endWaypoint].position[1];
	    double d_norm = sqrt(pow(d_segmentX,2)+pow(d_segmentY,2));
            while ((i_eraseIndex < this->vw_rover_path.size() - 2)&&(d_norm < 0.1))//TODO-Introduce here configurable position tolerance
	    {
                i_eraseIndex += 1;
                d_segmentX = this->vw_rover_path[i_eraseIndex].position[0] - this->vw_rover_path[endWaypoint].position[0];
                d_segmentY = this->vw_rover_path[i_eraseIndex].position[1] - this->vw_rover_path[endWaypoint].position[1];
                d_norm = sqrt(pow(d_segmentX,2)+pow(d_segmentY,2));
	    }
	    std::cout << "The erase index is " << i_eraseIndex << std::endl;
            this->vw_rover_path.erase(this->vw_rover_path.begin() + i_eraseIndex,
                              this->vw_rover_path.end());
        }
	else
        {
            this->vw_rover_path.erase(this->vw_rover_path.begin() + endWaypoint + 1,
                              this->vw_rover_path.end());
        }
    }
    return true;
}

unsigned int MotionPlan::computeArmProfilePlanning()
{
  //TODO - Create here several profiles: init, coupled, sweeping and retrieval
    if (!this->pmm_map->isSampleLoaded())
    {
        return 3;
    }
    this->vvd_arm_motion_profile.clear();
    std::vector<std::vector<double>> elevationMap;
    this->pmm_map->getElevationMapToZero(elevationMap);
    if(this->p_arm_planner->planArmMotion(&(this->vw_rover_path),
                                    &elevationMap,
                                    this->pmm_map->getResolution(),
                                    this->d_zres,
                                    this->pmm_map->getSample(),
                                    &(this->vvd_arm_motion_profile)))
    {
        if(this->isArmProfileSafe())
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

bool MotionPlan::isArmProfileSafe()
{
    for (int i = 0; i < this->vvd_arm_motion_profile.size(); i++)
    {
        if (this->p_collision_detector->isColliding(this->vvd_arm_motion_profile[i]))
	{
            std::cout << "ERROR at sample " << i << std::endl;
            std::cout << " Joint 1 = " << this->vvd_arm_motion_profile[i][0];
            std::cout << " Joint 2 = " << this->vvd_arm_motion_profile[i][1];
            std::cout << " Joint 3 = " << this->vvd_arm_motion_profile[i][2];
            std::cout << " Joint 4 = " << this->vvd_arm_motion_profile[i][3];
            std::cout << " Joint 5 = " << this->vvd_arm_motion_profile[i][4];
            std::cout << " Joint 6 = " << this->vvd_arm_motion_profile[i][5];
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

std::vector<std::vector<std::vector<double>>> * MotionPlan::get3DCostMap()
{
    return this->p_arm_planner->getVolumeCostMap();
}
