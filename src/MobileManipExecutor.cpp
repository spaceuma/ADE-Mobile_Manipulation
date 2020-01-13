#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"

MobileManipExecutor::MobileManipExecutor()
{
}

MobileManipExecutor::MobileManipExecutor(MotionPlan &currentMotionPlan)
{
    this->p_motion_plan = &currentMotionPlan;
    std::vector<base::Waypoint> *rover_path = this->p_motion_plan->getPath();
    this->vpw_path.clear();

    for (size_t i = 0; i < rover_path->size(); i++)
    {
        rover_path->at(i).tol_position = 0.1;
        this->vpw_path.push_back(&rover_path->at(i));
    }
    this->waypoint_navigation.setTrajectory(this->vpw_path);
    this->waypoint_navigation.setNavigationState(DRIVING);
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
    std::cout << " Arm motion profile of " << vvd_arm_motion_profile[0].size()
              << " joints and " << vvd_arm_motion_profile.size()
              << " samples is loaded " << std::endl;
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

MotionCommand MobileManipExecutor::getRoverCommand(Pose rover_pose)
{
    // TODO - implement MobileManipExecutor::getRoverCommand
    waypoint_navigation.setPose(rover_pose);
    waypoint_navigation.update(this->motion_command);
    return this->motion_command;
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
        std::cout << "The current segment is " << i_current_segment
                  << std::endl;
        if (i_current_segment == 102)
        {
            for (uint j = 0; j < vvd_arm_motion_profile.size(); j++)
            {
                std::cout << " Arm Profile at " << j << " is ";
                for (uint i = 0; i < vvd_arm_motion_profile[0].size(); i++)
                {
                    std::cout << vvd_arm_motion_profile[j][i] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
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
