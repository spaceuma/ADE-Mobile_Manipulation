#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"

MobileManipExecutor::MobileManipExecutor(MotionPlan* presentMotionPlan, const Joints &j_present_readings)
{
    this->initializeArmVariables(j_present_readings); 
    this->p_motion_plan = presentMotionPlan;
    this->updateMotionPlan();
    this->armstate = INITIALIZING;
}

void MobileManipExecutor::initializeArmVariables(const Joints &j_present_readings)
{
    double d_val;
    this->vd_arm_previous_command.resize(6);
    this->vd_arm_present_command.resize(6);
    this->vd_arm_previous_readings.resize(6);
    this->vd_arm_present_readings.resize(6);
    this->vd_arm_abs_speed.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        d_val = j_present_readings.m_jointStates[i].m_position; 
        this->vd_arm_previous_command[i] = d_val;
	this->vd_arm_present_command[i] = d_val;
        this->vd_arm_previous_readings[i] = d_val;
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
    this->waypoint_navigation.setTrajectory(this->vpw_path);

    // Extract and store the joints profile
    this->pvvd_arm_motion_profile
        = this->p_motion_plan->getArmMotionProfile();


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
    double gain = 1.0;
    int saturation = 0;
    double max_speed = 2.0;

    // Getting Arm Command
    bool b_isFinal = this->getArmCommand(j_next_arm_command_m);
    std::cout << "The Current Segment is " << this->waypoint_navigation.getCurrentSegment() << " and the path size is " << this->vpw_path.size() << std::endl;
    switch (this->armstate)
    {
        case INITIALIZING:
            if (this->isArmReady(j_next_arm_command_m, j_arm_present_readings_m))
	    {
                this->armstate = READY; 
            } 
            mc_m = this->getZeroRoverCommand();
	    return 0;
	case READY:
            this->armstate = COUPLED_MOVING;
	    return 1;
	case COUPLED_MOVING:
            /*if (!isArmWorking(j_next_arm_command_m, j_arm_present_readings_m))
	    {
                mc_m = this->getZeroRoverCommand();
	        return 5;
	    }*/
    	    std::cout << "\033[32m[----------]\033[0m [INFO] Rover Motion Command before MotionControl is (translation speed = " << mc_m.m_speed_ms
		  << " m/s, rotation speed = " << mc_m.m_turnRate_rads << " rad/s)" << " and the maneuvre type is "<< mc_m.m_manoeuvreType << std::endl;

            this->coupled_control.manipulatorMotionControl(gain, saturation, max_speed, vd_arm_present_command, vd_arm_previous_command, vd_arm_abs_speed); 
            if (saturation == 1)
	    {
                this->coupled_control.modifyMotionCommand(max_speed, vd_arm_abs_speed, mc_m); 
	    }
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

bool MobileManipExecutor::isArmReady(const Joints &j_next_command, const Joints &j_present_joints)
{
    for (uint i = 0; i < 6; i++)
    {
        if (abs(j_next_command.m_jointStates[i].m_position-j_present_joints.m_jointStates[i].m_position) > 0.05)
	{
            return false;
	}	
    }
    for (uint i = 0; i<6; i++)
    {
	this->vd_arm_previous_command[i] = j_next_command.m_jointStates[i].m_position; 
    } 
    return true;
}

bool MobileManipExecutor::isArmWorking(const Joints &j_next_command, const Joints &j_present_joints)
{
    double d_deg2rad = 3.1416/180.0;
    bool isMoving = true;
    for (uint i = 0; i < 6; i++)
    {
        if (abs(this->vd_arm_previous_command[i] - j_present_joints.m_jointStates[i].m_position) > 0.1)//TODO - ADhoc threshold in radians
        {
            isMoving = false;
	}
	this->vd_arm_previous_command[i] = j_next_command.m_jointStates[i].m_position; 
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

bool MobileManipExecutor::getArmCommand(Joints &j_next_arm_command)
{
    int i_pos_index;
    switch(this->armstate)
    {
        case INITIALIZING:
            i_pos_index = this->waypoint_navigation.getCurrentSegment();
	    break;
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
        this->vd_arm_previous_command[i] = this->vd_arm_present_readings[i];
    }
        
    bool b_isFinal = coupled_control.selectNextManipulatorPosition(
        this->waypoint_navigation.getCurrentSegment(),
        this->pvvd_arm_motion_profile,
        &(this->vd_arm_present_readings),
        true);
    for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
    {
        j_next_arm_command.m_jointStates[i].m_position
            = vd_arm_present_readings[i];
    }
    return b_isFinal;
}

