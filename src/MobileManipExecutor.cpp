#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"

MobileManipExecutor::MobileManipExecutor()
{
}

MobileManipExecutor::MobileManipExecutor(MotionPlan &currentMotionPlan)
{
    this->p_motion_plan = &currentMotionPlan;
    // Extract the rover path
    std::vector<base::Waypoint> *rover_path = this->p_motion_plan->getRoverPath();

    // Set the path into the Waypoint Navigation class
    this->vpw_path.clear();
    for (size_t i = 0; i < rover_path->size(); i++)
    {
        rover_path->at(i).tol_position = 0.1;
        this->vpw_path.push_back(&rover_path->at(i));
    }
    this->waypoint_navigation.setTrajectory(this->vpw_path);

    // Extract and store the joints profile
    std::vector<std::vector<double>> *pvvd_arm_motion_profile
        = this->p_motion_plan->getArmMotionProfile();
    this->vvd_arm_motion_profile.clear();
    std::vector<double> row;
    row.clear();
    for (uint j = 0; j < pvvd_arm_motion_profile->size(); j++)
    {
        for (uint i = 0; i < (*pvvd_arm_motion_profile)[0].size(); i++)
        {
            row.push_back((*pvvd_arm_motion_profile)[j][i]);
        }
        this->vvd_arm_motion_profile.push_back(row);
        row.clear();
    }
}

void MobileManipExecutor::updateMotionPlan(MotionPlan &newMotionPlan)
{
    this->p_motion_plan = &newMotionPlan;
}

bool MobileManipExecutor::isRoverWithinCorridor(Pose rover_pose)
{
    // TODO - implement MobileManipExecutor::isRoverWithinCorridor
    throw "Not yet implemented";
}

bool MobileManipExecutor::isArmColliding()
{
    // TODO - implement MobileManipExecutor::isArmColliding
    throw "Not yet implemented";
}

bool MobileManipExecutor::isFinished()
{
    return waypoint_navigation.getNavigationState() == TARGET_REACHED;
}

unsigned int MobileManipExecutor::getRoverCommand(Pose rover_pose, MotionCommand &mc_m)
{
    waypoint_navigation.setPose(rover_pose);
    waypoint_navigation.update(mc_m);
    this->navstate_current = waypoint_navigation.getNavigationState();
    if ((this->navstate_current != DRIVING)&&(this->navstate_current != ALIGNING))
    {
	if (this->navstate_current == TARGET_REACHED)
	{
            return 1;
	}
	if (this->navstate_current == OUT_OF_BOUNDARIES)
	{
            return 2;
	}
        return 3; 
    }
    return 0;
}

void MobileManipExecutor::getArmCommand(Joints &j_next_arm_command)
{
    // TODO - implement MobileManipExecutor::getArmCommand
    if (this->isFinished())
    {
        // Sampling procedure
    }
    else
    {
        this->vd_next_arm_config.clear();
        // this->vd_next_arm_config.resize();
        for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
        {
            this->vd_next_arm_config.push_back(
                j_next_arm_command.m_jointStates[i].m_position);
            // this->vd_next_arm_config.push_back(0.0);
            // std::cout << "The position at joint " << i << " is " <<
            // this->vd_current_arm_config[i] << std::endl;
        }
        this->i_current_segment = this->waypoint_navigation.getCurrentSegment();
        coupled_control.selectNextManipulatorPosition(
            this->i_current_segment,
            &(this->vvd_arm_motion_profile),
            &(this->vd_next_arm_config),
            false);
        for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
        {
            j_next_arm_command.m_jointStates[i].m_position
                = vd_next_arm_config[i];
        }
    }
}
