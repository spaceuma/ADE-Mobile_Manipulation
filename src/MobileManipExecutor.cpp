#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"

MobileManipExecutor::MobileManipExecutor(MotionPlan* presentMotionPlan, const Joints &j_present_readings, std::string s_urdf_path_m )
{
    this->initializeArmVariables(j_present_readings); 
    this->p_motion_plan = presentMotionPlan;
    this->updateMotionPlan();
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m); 
    if (!isArmColliding())
    {
        this->armstate = INITIALIZING;
    }
    else
    {
        this->armstate = FORBIDDEN_POS; 
    }
    this->b_first_retrieval_point_reached = false;
    this->b_second_retrieval_point_reached = false;
    this->j_first_retrieval_position.m_jointStates.resize(6);
    this->j_second_retrieval_position.m_jointStates.resize(6);
    this->j_first_retrieval_position.m_jointStates[0].m_position = 0.5;
    this->j_first_retrieval_position.m_jointStates[1].m_position = -1.3;
    this->j_first_retrieval_position.m_jointStates[2].m_position = 1.8;
    this->j_first_retrieval_position.m_jointStates[3].m_position = 0.0;
    this->j_first_retrieval_position.m_jointStates[4].m_position = -0.5;
    this->j_first_retrieval_position.m_jointStates[5].m_position = 2.3562;
    this->j_second_retrieval_position.m_jointStates[0].m_position = 0.39;
    this->j_second_retrieval_position.m_jointStates[1].m_position = -1.83;
    this->j_second_retrieval_position.m_jointStates[2].m_position = 2.79;
    this->j_second_retrieval_position.m_jointStates[3].m_position = 0.0;
    this->j_second_retrieval_position.m_jointStates[4].m_position = -0.5;
    this->j_second_retrieval_position.m_jointStates[5].m_position = 2.3562;
}

void MobileManipExecutor::initializeArmVariables(const Joints &j_present_readings)
{
    double d_val;
    this->vd_arm_previous_command.resize(6);
    this->vd_arm_present_command.resize(6);
    this->vd_arm_present_readings.resize(6);
    this->vd_arm_abs_speed.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        d_val = j_present_readings.m_jointStates[i].m_position; 
        this->vd_arm_previous_command[i] = d_val;
	this->vd_arm_present_command[i] = d_val;
	this->vd_arm_present_readings[i] = d_val;
    }

}

void MobileManipExecutor::updateMotionPlan()
{
    // Extract the rover path
    std::vector<base::Waypoint> *rover_path = this->p_motion_plan->getRoverPath();

    // Set the path into the Waypoint Navigation class
    this->vpw_path.resize(rover_path->size());
    for (size_t i = 0; i < rover_path->size(); i++)
    {
        rover_path->at(i).tol_position = 0.1;
        this->vpw_path.at(i) = (&rover_path->at(i));
    }
    this->waypoint_navigation.configureTol(0.1,45.0/180.0*3.1416);//tolpos,tolheading
    this->waypoint_navigation.setTrajectory(this->vpw_path);
    // Extract and store the joints profile
    this->pvvd_arm_motion_profile
        = this->p_motion_plan->getArmMotionProfile();

    this->b_first_retrieval_point_reached = false;
    this->b_second_retrieval_point_reached = false;

}


bool MobileManipExecutor::isRoverFinished()
{
    return waypoint_navigation.getNavigationState() == TARGET_REACHED;
}

unsigned int MobileManipExecutor::getCoupledCommand(Pose &rover_pose, const Joints &j_arm_present_readings_m, MotionCommand &mc_m, Joints &j_next_arm_command_m)
{
    // Getting Rover Command
    waypoint_navigation.setPose(rover_pose);
    waypoint_navigation.update(mc_m);
   

    // Evaluating state of Rover Path Following
    this->navstate = waypoint_navigation.getNavigationState();

    if ((this->navstate != DRIVING)&&(this->navstate != ALIGNING)&&(this->navstate != TARGET_REACHED))
    {
        mc_m = this->getZeroRoverCommand();
        j_next_arm_command_m = j_arm_present_readings_m;
	if (this->navstate == OUT_OF_BOUNDARIES)
	{
            return 3;
	}
	else
	{
            return 4; 
	}
    }
    // TODO - Modify these configurable variables properly
    double gain = 10.0;
    double max_speed = 0.005;

    // Getting Arm Command
    bool b_isFinal = this->updateArmCommandAndPose(j_next_arm_command_m, j_arm_present_readings_m);
    std::cout << "The Current Segment is " << this->waypoint_navigation.getCurrentSegment() << " and the path size is " << this->vpw_path.size() << std::endl;
    std::cout << "The state is " << this->armstate << std::endl;
    switch (this->armstate)
    {
        case INITIALIZING:
            if (this->isArmColliding())
	    {
                mc_m = this->getZeroRoverCommand();
		this->armstate = FORBIDDEN_POS;
		return 6;
	    }
	    if (this->isArmReady(j_next_arm_command_m, j_arm_present_readings_m))
	    {
                this->armstate = READY; 
            }
            mc_m = this->getZeroRoverCommand();
	    return 0;
	case FORBIDDEN_POS:
            mc_m = this->getZeroRoverCommand();
	    if (!this->isArmColliding())
	    {
                
		this->armstate = INITIALIZING;
		return 0;
	    }
	    else
	    {
		return 5;
	    }
	case READY:
            this->armstate = COUPLED_MOVING;
	    return 1;
	case COUPLED_MOVING:
	    if (this->isArmColliding())
	    {
                mc_m = this->getZeroRoverCommand();
		this->armstate = FORBIDDEN_POS;
		return 6;
	    }
            if (!isArmFollowing(j_next_arm_command_m, j_arm_present_readings_m))
	    {
                mc_m = this->getZeroRoverCommand();
	        return 7;
	    }
    	    std::cout << "\033[32m[----------]\033[0m [INFO] Rover Motion Command before MotionControl is (translation speed = " << mc_m.m_speed_ms
		  << " m/s, rotation speed = " << mc_m.m_turnRate_rads << " rad/s)" << " and the maneuvre type is "<< mc_m.m_manoeuvreType << std::endl;

            this->coupled_control.modifyMotionCommand(gain, vd_arm_present_command, vd_arm_previous_command, max_speed, vd_arm_abs_speed, mc_m); 

    	    std::cout << "\033[32m[----------]\033[0m [INFO] Rover Motion Command before fixing is (translation speed = " << mc_m.m_speed_ms
		  << " m/s, rotation speed = " << mc_m.m_turnRate_rads << " rad/s)" << " and the maneuvre type is "<< mc_m.m_manoeuvreType << std::endl;
            fixMotionCommand(mc_m);// This sets the maneuver as Point Turn if needed
    	    std::cout << "\033[32m[----------]\033[0m [INFO] Final Rover Motion Command is (translation speed = " << mc_m.m_speed_ms
		  << " m/s, rotation speed = " << mc_m.m_turnRate_rads << " rad/s)" << " and the maneuvre type is "<< mc_m.m_manoeuvreType << std::endl;
	    std::cout << "VAlue of b_isfinal is " << b_isFinal << std::endl;
	    if(this->navstate == TARGET_REACHED)
	    {
                mc_m = this->getZeroRoverCommand();
	    }
	    if ((b_isFinal)&&(this->navstate == TARGET_REACHED)&&(this->isArmReady(j_next_arm_command_m, j_arm_present_readings_m)))
	    { 
                mc_m = this->getZeroRoverCommand();
		this->armstate = SAMPLING_POS;
		return 2;
	    }
	    return 1;
	case SAMPLING_POS:
            mc_m = this->getZeroRoverCommand();
	    return 2;
    }
}

unsigned int MobileManipExecutor::getRetrievalCommand(const Joints &j_arm_present_readings_m, Joints &j_next_arm_command_m)
{
    if (j_next_arm_command_m.m_jointNames.empty())
    {
        j_next_arm_command_m.m_jointNames.resize(6);
        j_next_arm_command_m.m_jointNames[0] = "arm_joint_1";
        j_next_arm_command_m.m_jointNames[1] = "arm_joint_2";
        j_next_arm_command_m.m_jointNames[2] = "arm_joint_3";
        j_next_arm_command_m.m_jointNames[3] = "arm_joint_4";
        j_next_arm_command_m.m_jointNames[4] = "arm_joint_5";
        j_next_arm_command_m.m_jointNames[5] = "arm_joint_6";
    }
    //TODO - Improve this method
   if (this->b_first_retrieval_point_reached)
   {
        if (this->b_second_retrieval_point_reached)
	{
	    return 2;
	}
        if (isArmReady(this->j_second_retrieval_position, j_arm_present_readings_m))
	{
            this->b_second_retrieval_point_reached = true;
	    std::cout << "Second Retrieval Point Reached!" << std::endl;
	    return 1;
	}
	else
	{
            for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
            {
                j_next_arm_command_m.m_jointStates[i].m_position
                    = this->j_second_retrieval_position.m_jointStates[i].m_position;
            }
            return 1;
        }
   }
   else
   {
      if (isArmReady(this->j_first_retrieval_position,j_arm_present_readings_m))
      {
        this->b_first_retrieval_point_reached = true;
	std::cout << "First Retrieval Point Reached!" << std::endl;
        for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
            {
                j_next_arm_command_m.m_jointStates[i].m_position
                    = this->j_second_retrieval_position.m_jointStates[i].m_position;
            }
	return 1;
      }
      else
      {
        for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
            {
                j_next_arm_command_m.m_jointStates[i].m_position
                    = this->j_first_retrieval_position.m_jointStates[i].m_position;
            }
	return 0;
      } 
   }
}


void MobileManipExecutor::fixMotionCommand(MotionCommand &mc_m)
{
    if ((abs(mc_m.m_speed_ms) < 0.0000001)&&(abs(mc_m.m_turnRate_rads) > 0.0000001))
    {
        mc_m.m_manoeuvreType = 1;
    }
    else
    {
        if ((abs(mc_m.m_speed_ms) > 0.0000001)&&(abs(mc_m.m_turnRate_rads) > 0.0000001))
        {
            mc_m.m_curvature_radm = mc_m.m_turnRate_rads / mc_m.m_speed_ms; 
            mc_m.m_manoeuvreType = 0;
	}
	else
	{
	    if((abs(mc_m.m_speed_ms) > 0.0000001)&&(abs(mc_m.m_turnRate_rads) < 0.0000001))

	    {
	        mc_m.m_curvature_radm = 0.0;
                mc_m.m_manoeuvreType = 0;
	    }
	    else
	    {
               mc_m = getZeroRoverCommand(); 
	    }
	}
    }
    
}

bool MobileManipExecutor::isArmSafe(const Joints &j_present_joints_m)
{
    //TODO: put in other function isColliding to check collisions 
    if (j_present_joints_m.m_jointStates[0].m_position < 0.35)
    {
        return false;
    }
    return true;
}

bool MobileManipExecutor::isArmColliding()
{
    return this->p_collision_detector->isColliding(this->vd_arm_present_readings);
}

bool MobileManipExecutor::isArmReady(const Joints &j_next_command, const Joints &j_present_joints)
{
    for (uint i = 0; i < 6; i++)
    {
        if (abs(j_next_command.m_jointStates[i].m_position-j_present_joints.m_jointStates[i].m_position) > 0.05)
	{
            return false;
	}	
    }
 
    return true;
}

bool MobileManipExecutor::isArmFollowing(const Joints &j_next_command, const Joints &j_present_joints)
{
    double d_deg2rad = 3.1416/180.0;
    bool isMoving = true;
    for (uint i = 0; i < 6; i++)
    {
	std::cout << "In joint " << i << " the previous command is " << this->vd_arm_previous_command[i] << " and the current pos is " << j_present_joints.m_jointStates[i].m_position << std::endl; 
        if (abs(this->vd_arm_previous_command[i] - j_present_joints.m_jointStates[i].m_position) > this->vd_arm_posmargin[i])//TODO - ADhoc threshold in radians
        {
            isMoving = false;
	    std::cout << "Error = " << abs(this->vd_arm_previous_command[i] - j_present_joints.m_jointStates[i].m_position) << " rad" << std::endl;
	}
    }
    return isMoving;
}

void MobileManipExecutor::getSamplingCommand(const Joints &j_arm_present_readings_m, Joints &j_next_arm_command_m)
{

}

void MobileManipExecutor::getAtomicCommand()
{

}

MotionCommand MobileManipExecutor::getZeroRoverCommand()
{
    MotionCommand mc_zero;
    mc_zero.m_manoeuvreType = 0; //0: Ackermann, 1: PointTurn
    mc_zero.m_curvature_radm = 0.0; //in radians/meter
    mc_zero.m_speed_ms = 0.0;  //in meters/seconds
    mc_zero.m_turnRate_rads = 0.0; //in radians/seconds
    return mc_zero;
}

bool MobileManipExecutor::updateArmCommandAndPose(Joints &j_next_arm_command, const Joints &j_present_joints_m)
{
    if (j_next_arm_command.m_jointNames.empty())
    {
        j_next_arm_command.m_jointNames.resize(6);
        j_next_arm_command.m_jointNames[0] = "arm_joint_1";
        j_next_arm_command.m_jointNames[1] = "arm_joint_2";
        j_next_arm_command.m_jointNames[2] = "arm_joint_3";
        j_next_arm_command.m_jointNames[3] = "arm_joint_4";
        j_next_arm_command.m_jointNames[4] = "arm_joint_5";
        j_next_arm_command.m_jointNames[5] = "arm_joint_6";
    }
    if (j_next_arm_command.m_jointStates.empty())
    {
        j_next_arm_command.m_jointStates.resize(6);
    }

    for (uint i = 0; i < 6; i++)
    {
        this->vd_arm_present_readings[i] = j_present_joints_m.m_jointStates[i].m_position;
    }


    int i_pos_index;
    switch(this->armstate)
    {
        case INITIALIZING:
            i_pos_index = this->waypoint_navigation.getCurrentSegment();
	    break;
        case FORBIDDEN_POS:
	    for (uint i = 0; i < 6; i++)// TODO - make this a function to stop the arm
            {
                j_next_arm_command.m_jointStates[i].m_position = j_present_joints_m.m_jointStates[i].m_position; 
            }
            return false;
	case READY:
	    i_pos_index = 0;
	    break;
	case COUPLED_MOVING:
            i_pos_index = this->waypoint_navigation.getCurrentSegment();
	    break;
	case SAMPLING_POS:
	    return true; 
    }

    for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
    {
        this->vd_arm_previous_command[i] = this->vd_arm_present_command[i];
    }
        
    bool b_isFinal = coupled_control.selectNextManipulatorPosition(
        this->waypoint_navigation.getCurrentSegment(),
        this->pvvd_arm_motion_profile,
        &(this->vd_arm_present_command),
        true);
    for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
    {
        j_next_arm_command.m_jointStates[i].m_position
            = vd_arm_present_command[i];
    }
    return b_isFinal;
}

