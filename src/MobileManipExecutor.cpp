#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "mmFileManager.h"

MobileManipExecutor::MobileManipExecutor(MotionPlan* presentMotionPlan, const Joints &j_present_readings, std::string s_urdf_path_m )
{
    this->initializeArmVariables(j_present_readings); 
    this->p_motion_plan = presentMotionPlan;
    this->updateMotionPlan();
    this->armstate = INITIALIZING;
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m); 
    this->pvvd_arm_sweeping_profile = new std::vector<std::vector<double>>;
    this->pvd_arm_sweeping_times = new std::vector<double>;
    readMatrixFile(s_urdf_path_m + "/sweepingProfile.txt", (*this->pvvd_arm_sweeping_profile));
    readVectorFile(s_urdf_path_m + "/sweepingTimes.txt", (*this->pvd_arm_sweeping_times));

    this->vd_retrieval_position.resize(6);
    this->vd_retrieval_position[0] = 0.45;
    this->vd_retrieval_position[1] = -1.83;
    this->vd_retrieval_position[2] = 2.79;
    this->vd_retrieval_position[3] = 0.0;
    this->vd_retrieval_position[4] = -0.5;
    this->vd_retrieval_position[5] = 2.3562;
    
    this->b_first_retrieval_point_reached = false;
    this->b_second_retrieval_point_reached = false;
    this->j_first_retrieval_position.m_jointStates.resize(6);
    this->j_second_retrieval_position.m_jointStates.resize(6);
    this->j_first_retrieval_position.m_jointStates[0].m_position = 0.5;
    this->j_first_retrieval_position.m_jointStates[1].m_position = -1.0;
    this->j_first_retrieval_position.m_jointStates[2].m_position = 1.8;
    this->j_first_retrieval_position.m_jointStates[3].m_position = 0.0;
    this->j_first_retrieval_position.m_jointStates[4].m_position = -0.5;
    this->j_first_retrieval_position.m_jointStates[5].m_position = 2.3562;
    this->j_second_retrieval_position.m_jointStates[0].m_position = 0.45;
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
    this->i_current_segment = 0;
    this->i_current_coverage_index = 0;
    this->i_current_retrieval_index = 0;
    // Extract and store the joints profile
    this->pvvd_arm_motion_profile
        = this->p_motion_plan->getArmMotionProfile();
    this->i_initial_segment = 0;
    this->b_is_last_segment = false;
    this->b_first_retrieval_point_reached = false;
    this->b_second_retrieval_point_reached = false;
    this->d_call_period = 0.5; // TODO: MAKE THIS CONFIGURABLE!!!
    this->i_iteration_counter = 0;
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
    double gain = 2.0;
    double max_speed = 0.05;

    // Create Profile for initial operation

    for (uint i = 0; i < 6; i++)
    {
            std::cout << " Executor Goal Joint " << i << " is " << (*this->pvvd_arm_motion_profile)[this->waypoint_navigation.getCurrentSegment()][i] << std::endl;
    }

    this->updateArmPresentReadings(j_arm_present_readings_m);
    
    if (this->isArmColliding())
    {
        for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
        {
            j_next_arm_command_m.m_jointStates[i].m_position
                = j_arm_present_readings_m.m_jointStates[i].m_position;
        }
        mc_m = this->getZeroRoverCommand();
        return 6;
    }

    if ((this->armstate == INITIALIZING)&&(this->i_iteration_counter == 0))
    {
        std::cout << " WN Current Segment = " << this->waypoint_navigation.getCurrentSegment() << std::endl;
	for (uint i = 0; i < 6; i++)
        {
            std::cout << " Arm Present Reading " << i << " is " << this->vd_arm_present_readings[i] << std::endl;
        }
	if (this->p_motion_plan->computeArmDeployment(this->waypoint_navigation.getCurrentSegment(), this->vd_arm_present_readings) != 0)
        {
               return 8;
	}
	std::cout << "The arm deployment is computed" << std::endl;	
        if (this->p_motion_plan->computeArmRetrieval((*this->pvvd_arm_sweeping_profile)[(*this->pvvd_arm_sweeping_profile).size()-1],this->vd_retrieval_position) != 0)
        {
               return 8;
	}
	std::cout << "The arm retrieval is computed" << std::endl;	
	this->pvvd_init_arm_profile  
                    = this->p_motion_plan->getInitArmMotionProfile();
        this->pvd_init_time_profile  
                    = this->p_motion_plan->getInitArmTimeProfile();
        this->pvvd_retrieval_arm_profile  
                    = this->p_motion_plan->getRetrievalArmMotionProfile();
        this->pvd_retrieval_time_profile  
                    = this->p_motion_plan->getRetrievalArmTimeProfile();
    }

    // Getting Arm Command
    this->prepareNextArmCommand(j_next_arm_command_m);

    this->updateArmCommandAndPose(j_next_arm_command_m);

    std::cout << "The Rover Segment is " << this->waypoint_navigation.getCurrentSegment() << std::endl;
    std::cout << "The Current Segment is " << this->i_current_segment << " and the path size is " << this->vpw_path.size() << std::endl;
    std::cout << "The state is " << this->armstate << std::endl;
    switch (this->armstate)
    {
        case INITIALIZING:
	    double d_elapsed_init_time;
	    // Keep track of vdd_init_arm_profile
	    std::cout << " Creating initialization command" << std::endl;
            d_elapsed_init_time = (double)this->i_iteration_counter * this->d_call_period;
            if (this->i_current_segment < (*this->pvvd_init_arm_profile).size()-1)
            {
                if ((*this->pvd_init_time_profile)[this->i_current_segment]*5.0 <= d_elapsed_init_time)//TODO - ADHOC value to make this slower
                {
                    this->i_current_segment++;
		    this->updateArmCommandVectors((*this->pvvd_init_arm_profile)[this->i_current_segment]);   
		}
            }
            this->i_iteration_counter++;
	    this->assignPresentCommand(j_next_arm_command_m);
            // TODO - Check if it is following!!

            fixMotionCommand(mc_m);// This sets the maneuver as Point Turn if needed
            if (((*this->pvd_init_time_profile)[(*this->pvvd_init_arm_profile).size()-1]*5.0 < 
		 (double)this->i_iteration_counter * this->d_call_period) &&
	        (this->i_current_segment >= this->pvvd_init_arm_profile->size()-1) &&
		(mc_m.m_manoeuvreType == 0))
            {  // If the arm is ready and the rover is going to start with an ackemann
               this->i_initial_segment = this->waypoint_navigation.getCurrentSegment();
	       this->i_current_segment = this->i_initial_segment;
               this->armstate = READY;  
	    }
            if (mc_m.m_manoeuvreType != 1)
            {
                mc_m = this->getZeroRoverCommand();
            }
	    return 0;
	case READY:
            this->armstate = COUPLED_MOVING;
	    return 1;
	case COUPLED_MOVING:
            if (!isArmFollowing(j_next_arm_command_m, j_arm_present_readings_m))
	    {
                for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
                {
                    j_next_arm_command_m.m_jointStates[i].m_position
                        = j_arm_present_readings_m.m_jointStates[i].m_position;
                }

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
	    std::cout << "VAlue of b_isfinal is " << b_is_last_segment << std::endl;
	    if(this->navstate == TARGET_REACHED)
	    {
                mc_m = this->getZeroRoverCommand();
	    }
	    if ((b_is_last_segment)&&(this->navstate == TARGET_REACHED)&&(this->isArmReady(j_next_arm_command_m, j_arm_present_readings_m)))
	    { 
                mc_m = this->getZeroRoverCommand();
		this->armstate = SAMPLING_POS;
		this->i_current_coverage_index = 0;
		this->i_current_retrieval_index = 0;
		this->i_iteration_counter = 0;
		return 2;
	    }
	    return 1;
	case SAMPLING_POS:
            mc_m = this->getZeroRoverCommand();
	    return 2;
    }
}

unsigned int MobileManipExecutor::getCoverageCommand(Joints &j_next_arm_command, const Joints &j_present_joints_m)
{
    // TODO: introduce followingarm checker
    double d_elapsed_time = (double)this->i_iteration_counter * this->d_call_period;
    bool b_is_finished = false;
    
    this->prepareNextArmCommand(j_next_arm_command);
    this->updateArmPresentReadings(j_present_joints_m); 
    if (this->i_current_coverage_index < (*this->pvvd_arm_sweeping_profile).size()-1)
    {
        if ((*this->pvd_arm_sweeping_times)[this->i_current_coverage_index]*2.0 <= d_elapsed_time)//TODO - ADHOC value to make this slower
        {
            this->i_current_coverage_index++;
            this->updateArmCommandVectors((*this->pvvd_arm_sweeping_profile)[this->i_current_coverage_index]);   
	}
    }
    else if ((*this->pvd_arm_sweeping_times)[(*this->pvvd_arm_sweeping_profile).size()-1]*2.0 < d_elapsed_time)
    {
            b_is_finished = true; 
    }
    std::cout << "The Coverage Index is " << i_current_coverage_index << std::endl;
    this->assignPresentCommand(j_next_arm_command); 
    this->i_iteration_counter++;
    if (b_is_finished)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void MobileManipExecutor::resetIterator()
{
    this->i_iteration_counter = 0;
}

void MobileManipExecutor::assignPresentCommand(Joints &j_command)
{
    for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
    {
        j_command.m_jointStates[i].m_position
            = vd_arm_present_command[i];
    }
}

unsigned int MobileManipExecutor::getRetrievalCommand(const Joints &j_present_joints_m, Joints &j_next_arm_command)
{
     // TODO: introduce followingarm checker
    double d_elapsed_time = (double)this->i_iteration_counter * this->d_call_period;
    bool b_is_finished = false;
    
    this->prepareNextArmCommand(j_next_arm_command);
    this->updateArmPresentReadings(j_present_joints_m); 
    if (this->i_current_retrieval_index < (*this->pvvd_retrieval_arm_profile).size()-1)
    {
        if ((*this->pvd_retrieval_time_profile)[this->i_current_retrieval_index]*2.0 <= d_elapsed_time)//TODO - ADHOC value to make this slower
        {
            this->i_current_retrieval_index++;
            this->updateArmCommandVectors((*this->pvvd_retrieval_arm_profile)[this->i_current_retrieval_index]);   
	}
    }
    else if ((*this->pvd_retrieval_time_profile)[(*this->pvvd_retrieval_arm_profile).size()-1]*2.0 < d_elapsed_time)
    {
            b_is_finished = true; 
    }
    std::cout << "The Retrieval Index is " << i_current_retrieval_index << std::endl;
    this->assignPresentCommand(j_next_arm_command); 
    this->i_iteration_counter++;
    if (b_is_finished)
    {
        return 1;
    }
    else
    {
        return 0;
    }
  
/*    this->prepareNextArmCommand(j_next_arm_command_m);

    this->updateArmPresentReadings(j_arm_present_readings_m); 

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
   }*/
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
    double d_margin;
    for (uint i = 0; i < 6; i++)
    {
	std::cout << "In joint " << i << " the previous command is " << this->vd_arm_previous_command[i] << " and the current pos is " << j_present_joints.m_jointStates[i].m_position << std::endl; 
        d_margin = max(1.2*abs(this->vd_arm_previous_command[i] - this->vd_arm_present_command[i]), this->vd_arm_posmargin[i]);
        if (abs(this->vd_arm_previous_command[i] - j_present_joints.m_jointStates[i].m_position) > d_margin)//TODO - ADhoc threshold in radians
        {
            isMoving = false;
	    std::cout << "Error = " << abs(this->vd_arm_previous_command[i] - j_present_joints.m_jointStates[i].m_position) << " rad" << std::endl;
	}
    }
    //return isMoving;
    return true; // TODO - Remove this
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

bool MobileManipExecutor::updateArmPresentReadings(const Joints &j_present_joints_m)
{
    for (uint i = 0; i < 6; i++)
    {
        this->vd_arm_present_readings[i] = j_present_joints_m.m_jointStates[i].m_position;
    }
}

bool MobileManipExecutor::updateArmCommandVectors()
{
    for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
    {
        this->vd_arm_previous_command[i] = this->vd_arm_present_command[i];
    }
}

bool MobileManipExecutor::updateArmCommandVectors(const std::vector<double> &vd_present_command_m)
{
    for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
    {
        this->vd_arm_previous_command[i] = this->vd_arm_present_command[i];
        this->vd_arm_present_command[i] = vd_present_command_m[i];
    }
}

void MobileManipExecutor::updateArmCommandAndPose(Joints &j_next_arm_command)
{

    int i_actual_segment = this->waypoint_navigation.getCurrentSegment();

    switch (armstate)
    {
        case INITIALIZING:
	    break;
	default:
            if ((this->i_current_segment == this->i_initial_segment)||(this->i_current_segment != i_actual_segment))
            {
                this->updateArmCommandVectors(); 
		if (i_current_segment < i_actual_segment)
		{
                    i_current_segment++;// = i_actual_segment;
		}
                this->b_is_last_segment = coupled_control.selectNextManipulatorPosition(
                    i_current_segment,
                    this->pvvd_arm_motion_profile,
                    &(this->vd_arm_present_command),
                    true);
            }
	    this->assignPresentCommand(j_next_arm_command);
            break;	
    }
}

bool MobileManipExecutor::prepareNextArmCommand(Joints &j_next_arm_command)
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
}



std::vector<double>* MobileManipExecutor::getArmCurrentReadings()
{
    return &(this->vd_arm_present_readings); 
}
