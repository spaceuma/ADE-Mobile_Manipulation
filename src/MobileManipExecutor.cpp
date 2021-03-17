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


#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "mmFileManager.h"

MobileManipExecutor::MobileManipExecutor(MotionPlan *presentMotionPlan,
                                         std::string s_urdf_path_m,
					 unsigned int ui_operation_mode)
{
    // this->initializeArmVariables(j_present_readings);
    this->vd_arm_previous_command.resize(6);
    this->vd_arm_present_command.resize(6);
    this->vd_arm_present_readings.resize(6);
    this->p_motion_plan = presentMotionPlan;
    this->updateMotionPlan();
    this->armstate = INITIALIZING;
    this->p_collision_detector = new CollisionDetector(s_urdf_path_m);
    this->pvvd_arm_sweeping_profile = new std::vector<std::vector<double>>;
    this->pvd_arm_sweeping_times = new std::vector<double>;
    this->d_dist_to_sample = 10000;

    this->pvvd_turning_curvature_matrix = new std::vector<std::vector<double>>;
    this->pvvd_turning_angle_matrix = new std::vector<std::vector<double>>;
    this->pvd_lsc_x0 = new std::vector<double>;
    this->pvd_lsc_y0 = new std::vector<double>;

    readMatrixFile(s_urdf_path_m + "/LastSectionControl/LSC_TurningCurvature.txt",
                   (*this->pvvd_turning_curvature_matrix));
    readMatrixFile(s_urdf_path_m + "/LastSectionControl/LSC_TurningAngle.txt",
                   (*this->pvvd_turning_angle_matrix));
    readVectorFile(s_urdf_path_m + "/LastSectionControl/LSC_X0.txt",
                   (*this->pvd_lsc_x0));
    readVectorFile(s_urdf_path_m + "/LastSectionControl/LSC_Y0.txt",
                   (*this->pvd_lsc_y0));
    switch(ui_operation_mode)
    {
        case 0: //PICK
            readMatrixFile(s_urdf_path_m + "/arm_pick_profile.txt",
                   (*this->pvvd_arm_sweeping_profile));
            readVectorFile(s_urdf_path_m + "/arm_pick_times.txt",
                   (*this->pvd_arm_sweeping_times));
	    break;
        case 1: //DROP
            readMatrixFile(s_urdf_path_m + "/arm_drop_profile.txt",
                   (*this->pvvd_arm_sweeping_profile));
            readVectorFile(s_urdf_path_m + "/arm_drop_times.txt",
                   (*this->pvd_arm_sweeping_times));
	    break;
        default: //SWEEPING
            readMatrixFile(s_urdf_path_m + "/arm_sweeping_profile.txt",
                   (*this->pvvd_arm_sweeping_profile));
            readVectorFile(s_urdf_path_m + "/arm_sweeping_times.txt",
                   (*this->pvd_arm_sweeping_times));
	    break;
     
    } 

    this->i_current_init_index = 0;
    this->i_current_retrieval_index = 0;
}


void MobileManipExecutor::setOperationMode(unsigned int ui_operation_mode, std::string s_urdf_path_m)
{
    switch(ui_operation_mode)
    {
        case 0: //PICK
            readMatrixFile(s_urdf_path_m + "/arm_pick_profile.txt",
                   (*this->pvvd_arm_sweeping_profile));
            readVectorFile(s_urdf_path_m + "/arm_pick_times.txt",
                   (*this->pvd_arm_sweeping_times));
	    break;
        case 1: //DROP
            readMatrixFile(s_urdf_path_m + "/arm_drop_profile.txt",
                   (*this->pvvd_arm_sweeping_profile));
            readVectorFile(s_urdf_path_m + "/arm_drop_times.txt",
                   (*this->pvd_arm_sweeping_times));
	    break;
        default: //SWEEPING
            readMatrixFile(s_urdf_path_m + "/arm_sweeping_profile.txt",
                   (*this->pvvd_arm_sweeping_profile));
            readVectorFile(s_urdf_path_m + "/arm_sweeping_times.txt",
                   (*this->pvd_arm_sweeping_times));
	    break;     
    } 
}


void MobileManipExecutor::initializeArmVariables(
    const proxy_library::Joints &j_present_readings)
{
    double d_val;
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
    std::vector<base::Waypoint> *rover_path
        = this->p_motion_plan->getRoverPath();

    // Set the path into the Waypoint Navigation class
    this->vpw_path.resize(rover_path->size());
    for (size_t i = 0; i < rover_path->size(); i++)
    {
        rover_path->at(i).tol_position = 0.1;
        this->vpw_path.at(i) = (&rover_path->at(i));
    }
    this->waypoint_navigation.configureTol(
        0.1, 5.0 / 180.0 * 3.1416); // tolpos,tolheading
    this->waypoint_navigation.configureAlignment(
                    5.0 / 180.0 * M_PI, //deadband
                    20.0 / 180.0 * M_PI, //saturation
                    10.0 / 180.0 * M_PI); //rotationalVelocity 
    
    this->waypoint_navigation.setTrajectory(this->vpw_path);
    this->i_current_segment = 0;
    this->i_current_coverage_index = 0;
    this->i_current_init_index = 0;
    this->i_current_retrieval_index = 0;
    // Extract and store the joints profile
    this->pvvd_arm_motion_profile = this->p_motion_plan->getCoupledArmMotionProfile();
    this->i_initial_segment = 0;
    this->b_is_last_segment = false;
    this->d_operational_time = 0.0;
    this->updateDeployment();
    this->ui_current_timestamp = 0;
    this->ui_past_timestamp = 0;
    this->updateRetrieval();
}

void MobileManipExecutor::updateRetrieval()
{
    this->i_current_retrieval_index = 0;
    this->pvvd_retrieval_arm_profile
        = this->p_motion_plan->getRetrievalArmMotionProfile();
    this->pvd_retrieval_time_profile
        = this->p_motion_plan->getRetrievalArmTimeProfile();
}

void MobileManipExecutor::updateDeployment()
{
    this->i_current_init_index = 0;
    this->pvvd_init_arm_profile
        = this->p_motion_plan->getInitArmMotionProfile();
    this->pvd_init_time_profile = this->p_motion_plan->getInitArmTimeProfile();
}

double MobileManipExecutor::computeBilinearInterpolation(double x, double y, 
		double x1, double x2, double y1, double y2, double Q11, 
		double Q12, double Q21, double Q22)
{
    double A = ( (x2 - x)*Q11 + (x - x1)*Q21 ) * ( y2 - y );
    double B = ( (x2 - x)*Q12 + (x - x1)*Q22 ) * ( y - y1 );
    double C = ( x2 - x1 ) * ( y2 - y1 );
    return ( A + B ) / C ; 
}

bool MobileManipExecutor::isAligned(base::Pose &rover_pose)
{
    double dx,dy,dist;
    dx = rover_pose.position[0] - (*this->vpw_path.back()).position[0];  
    dy = rover_pose.position[1] - (*this->vpw_path.back()).position[1];  
    dist = sqrt(pow(dx,2)+pow(dy,2));

    if ((dist < 2.3)&&(this->d_dist_to_sample >= 2.3)) //This would be d_inner_sampling_dist from the map class
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::isAligned()]\033[0m Entering the last section, distance to sample is " << dist << " meters"  << std::endl;
       
    }
    if ((dist < 1.4)&&(this->d_dist_to_sample >= 1.4)) //This would be d_inner_sampling_dist from the map class
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::isAligned()]\033[0m Arrived, distance is " << dist << " meters"  << std::endl;
       
    }
    this->d_dist_to_sample = dist;
    if ((dist < 1.1))//&&(dacos < 0.1))
    {
        return true; 
    }
    else
    {
        return false;
    }
}


/*
 * getCoupledCommand()
 *
 */

unsigned int MobileManipExecutor::getCoupledCommand(
    Pose &rover_pose,
    const proxy_library::Joints &j_arm_present_readings_m,
    proxy_library::MotionCommand &mc_m,
    proxy_library::Joints &j_next_arm_command_m)
{


    /*
     * 1 --Initial Temptative Trajectory Tracking Command--
     */

    // Rover Command from C-Pursuit Waypoint Navigation
    waypoint_navigation.setPose(rover_pose);
    waypoint_navigation.update(mc_m);
    this->navstate = waypoint_navigation.getNavigationState();
    if ((this->navstate != DRIVING) && (this->navstate != ALIGNING)
        && (this->navstate != TARGET_REACHED))
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

    // mc_m is modified if rover is within Last Section
    this->getLastSectionCommand(rover_pose, mc_m); 


    /*
     * 2 --Update and check Arm status--
     */

    // Current arm joints positions are stored
    this->updateArmPresentReadings(j_arm_present_readings_m);

    // Check if there may be any (near)collision
    if (this->isArmColliding())
    {
       
        // Arm must stop 
        for (uint i = 0; i < this->ui_num_joints; i++)
        {
            j_next_arm_command_m.m_jointStates[i].m_position
                = j_arm_present_readings_m.m_jointStates[i].m_position;
        }
        
	// Rover must stop
	mc_m = this->getZeroRoverCommand();
        return 6;
    }


    unsigned int ui_status = 0;
    int i_actual_segment;
    double d_targetHeading, d_headingError;
    
    switch (this->armstate)
    {

        case INITIALIZING:
            
            // This sets the maneuver as Point Turn if needed
            fixMotionCommand(mc_m); 
            if (mc_m.m_manoeuvreType != 1)
            {
                mc_m = this->getZeroRoverCommand();
            }
	    else
	    {
                //It is a point turn
		d_targetHeading = waypoint_navigation.getTargetHeading(); 
		d_headingError = waypoint_navigation.getHeadingError();
                std::cout << "[MM] \033[35m[----------]"
                " [MobileManipExecutor::getCoupledCommand()]\033[0m Point Turn Data: Rover Heading = " << rover_pose.getYaw() << " rad, " <<
                "Target Heading = " << d_targetHeading << " rad, Heading Error = " << d_headingError << " rad" << std::endl;
               
	    }
        
            ui_status = this->getAtomicCommand(j_arm_present_readings_m, 
			                       j_next_arm_command_m,0);
            
	    if(ui_status == 1)
	    {

                this->i_initial_segment
                    = this->waypoint_navigation.getCurrentSegment();
                this->i_current_segment = min(0, 
                    (int)(*this->pvvd_arm_motion_profile).size() - 1);
                if (this->vpw_path.size() == 1)
	        {
                    mc_m = this->getZeroRoverCommand();
                    this->armstate = SAMPLING_POS;
                    this->i_current_coverage_index = 0;
                    this->i_current_retrieval_index = 0;
                    return 2;
	        }
	        else
	        {
                    mc_m = this->getZeroRoverCommand();
                    this->armstate = COUPLED_MOVING;
		    return 1;
	        }

            }
	    else if (ui_status > 1)
	    {
                // Error state 
                mc_m = this->getZeroRoverCommand();
                if (ui_status == 4)
		{
                    return 6; 
                }
		else if (ui_status == 5)
		{
                    return 7;
		}
		else
		{
                    //std::cout << "ui_status = " << ui_status << std::endl;
                    return 4;
                }
	    }
            return 0;

	case COUPLED_MOVING:
	    //To avoid the dummy turn waypoint
	    i_actual_segment = min(this->waypoint_navigation.getCurrentSegment(), (int)(*this->pvvd_arm_motion_profile).size() - 1);
	    //std::cout << "COUPLED_MOVING: initial segment is " << i_initial_segment << std::endl;
            if ((this->i_current_segment == this->i_initial_segment)
                || (this->i_current_segment != i_actual_segment))
            {
                if (i_actual_segment
                    < (*this->pvvd_arm_motion_profile).size() - 7)
                {
                    i_actual_segment = max(0, i_actual_segment - 6);
                }
                this->updateArmCommandVectors(); //TODO: could be changed to unify the updateArm... function?
                if (i_current_segment < i_actual_segment)
                {
                    i_current_segment = min(i_actual_segment, i_current_segment + 1);// + (i_actual_segment - i_current_segment)/30);
                    //i_current_segment++; // = i_actual_segment;
                }
            }
	    //std::cout << "COUPLED_MOVING: current segment is " << i_current_segment << std::endl;
            double d_angle, d_na;
            for (unsigned int i = 0; i < this->vd_arm_present_command.size(); i++)
            {
                d_angle = (*this->pvvd_arm_motion_profile)[i_current_segment][i];
                d_na = atan2(sin(d_angle), cos(d_angle));
                if (d_na < -3.1416)
		{
		    d_na = d_na + 2 * 3.1416;
		}
                if (d_na > 3.1416)
		{
		    d_na = d_na - 2 * 3.1416;
		}
		this->vd_arm_present_command[i] = d_na;
            }
            // Returns true if it is the last position
            this->b_is_last_segment = (i_current_segment == (*this->pvvd_arm_motion_profile).size() - 1);

	    
            this->assignPresentCommand(j_next_arm_command_m);
            if ((!isArmFollowing(j_next_arm_command_m, j_arm_present_readings_m))&&
		(!isArmMoving(j_arm_present_readings_m)))
            {
                for (uint i = 0; i < this->ui_num_joints; i++)
                {
                    j_next_arm_command_m.m_jointStates[i].m_position
                        = j_arm_present_readings_m.m_jointStates[i].m_position;
                }

                mc_m = this->getZeroRoverCommand();
                return 7;
            }
	     
            fixMotionCommand(mc_m); // This sets the maneuver as Point Turn if needed
            //std::cout << "\033[32m[----------]\033[0m [INFO] Final Rover Motion Command is (translation speed = " << mc_m.m_speed_ms
          //<< " m/s, rotation speed = " << mc_m.m_turnRate_rads << " rad/s)" << "and the maneuvre type is "<< mc_m.m_manoeuvreType << std::endl;
        //std::cout << "VAlue of b_isfinal is " << b_is_last_segment << std::endl;
            //if (this->navstate == TARGET_REACHED)
	    //if (i_actual_segment == (int)this->vpw_path.size() - 2)
	    if(isAligned(rover_pose))
            {
                mc_m = this->getZeroRoverCommand();
            }
            if ((b_is_last_segment) && 
			   (isAligned(rover_pose))
                && (this->isArmReady(j_next_arm_command_m,
                                     j_arm_present_readings_m)))
            {
                mc_m = this->getZeroRoverCommand();
                this->armstate = SAMPLING_POS;
                this->i_current_coverage_index = 0;
                this->i_current_retrieval_index = 0;
                return 2;
            }
	    else if((b_is_last_segment) && 
			   (isAligned(rover_pose)) )
	    {
                //std::cout << "For some reason the arm is not yet ready" << std::endl;
	    }
            return 1;
        case SAMPLING_POS:
            mc_m = this->getZeroRoverCommand();
            return 2;
    }
}

void MobileManipExecutor::resetOperationTime()
{
    this->d_operational_time = 0.0;
    this->ui_current_timestamp = 0;
    this->ui_past_timestamp = 0;
}

bool MobileManipExecutor::assignPresentCommand(proxy_library::Joints &j_command)
{
    if (j_command.m_jointNames.empty())
    {
        j_command.m_jointNames.resize(6);
        j_command.m_jointNames[0] = "arm_joint_1";
        j_command.m_jointNames[1] = "arm_joint_2";
        j_command.m_jointNames[2] = "arm_joint_3";
        j_command.m_jointNames[3] = "arm_joint_4";
        j_command.m_jointNames[4] = "arm_joint_5";
        j_command.m_jointNames[5] = "arm_joint_6";
    }
    if (j_command.m_jointStates.empty())
    {
        j_command.m_jointStates.resize(6);
    }
    if ((vd_arm_present_command.empty())
        || (vd_arm_present_command.size() != this->ui_num_joints))
    {
        return false;
    }
    for (uint i = 0; i < this->ui_num_joints;
         i++)
    {
        j_command.m_jointStates[i].m_position = vd_arm_present_command[i];
    }
    return true;
}

void MobileManipExecutor::printExecutionStatus()
{
    std::cout << "\r \033[33m[----------]\033[0m [printExecutionStatus()] EXECUTION INFORMATION" << std::endl; 
    std::cout << "\r \033[33m[----------]\033[0m [printExecutionStatus()] - Current Timestamp: " << this->ui_current_timestamp << std::endl; 
    std::cout << "\r \033[33m[----------]\033[0m [printExecutionStatus()] - Step Time (ms): " << (this->ui_current_timestamp - this->ui_past_timestamp) / 1000 << std::endl; 
}

unsigned int MobileManipExecutor::getAtomicCommand(
    const proxy_library::Joints &j_present_joints_m,
    proxy_library::Joints &j_next_arm_command,
    unsigned int ui_mode)
{  

    if (!this->updateArmPresentReadings(j_present_joints_m))
    {
        return 2; // j_present_joints_m has wrong size
    }

    if (this->isArmColliding())
    {
        for (uint i = 0; i < this->ui_num_joints; i++)
        {
            j_next_arm_command.m_jointStates[i].m_position
                = j_present_joints_m.m_jointStates[i].m_position;
        }
        return 4;
    }

    switch (ui_mode)
    {
        case 0: // Deployment
            if (((*this->pvvd_init_arm_profile).empty())
               || ((*this->pvd_init_time_profile).empty()))
            {
                // TODO - It may happen it is empty because the arm is already initialized...
		// return 3; // There is no profile available
		return 1;
            }
	    break;
        case 1: // Retrieval
            if (((*this->pvvd_retrieval_arm_profile).empty())
               || ((*this->pvd_retrieval_time_profile).empty()))
            {
                return 3; // There is no profile available
            }
	    break;
        case 2:  // Coverage
            if (((*this->pvvd_arm_sweeping_profile).empty())
               || ((*this->pvd_arm_sweeping_times).empty()))
            {
                return 3; // There is no profile available
            }
	    break;
    }
    

    // TODO: introduce followingarm checker
    this->ui_past_timestamp = this->ui_current_timestamp;
    this->ui_current_timestamp = j_present_joints_m.m_time; 
    double d_step_time = (double)this->ui_current_timestamp - (double)this->ui_past_timestamp;  
    d_step_time = std::max(0.0,d_step_time / 1000000);
    this->d_operational_time += std::min(2.0, d_step_time); 

    //std::cout << "Step Time is " << d_step_time << " seconds" << std::endl;
    //std::cout << "Operational Time is " << this->d_operational_time << " seconds" << std::endl;

    double d_elapsed_time
	= this->d_operational_time;
    bool b_is_finished = false;
    bool b_is_ready = false;
    double d_current_timelimit, d_ratio_bef, d_ratio_aft;
    
    switch (ui_mode)
    {
        case 0: // Deployment
            //std::cout << "Current init index " << this->i_current_init_index << std::endl;
            if (this->i_current_init_index < (*this->pvvd_init_arm_profile).size() - 1)
            {
                d_current_timelimit = (*this->pvd_init_time_profile)[this->i_current_init_index] * 1.0;
	        //std::cout << "The current time limit is " << d_current_timelimit << std::endl;
                if ((*this->pvd_init_time_profile)[this->i_current_init_index] * 1.0
                    <= d_elapsed_time) // TODO - ADHOC value to make this slower
                {
                    //this->i_current_init_index = min((int)(*this->pvvd_init_arm_profile).size() - 1,this->i_current_init_index+1);
                    d_ratio_bef = (double)this->i_current_init_index / (double)((*this->pvvd_init_arm_profile).size() - 1);
                    d_ratio_aft = (double)(this->i_current_init_index + 1) / (double)((*this->pvvd_init_arm_profile).size() - 1);
                    if ((d_ratio_bef < 0.25)&&(d_ratio_aft >= 0.25)) 
                    {
                        std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Deployment: 25 percent" << std::endl;
		    }
                    if ((d_ratio_bef < 0.5)&&(d_ratio_aft >= 0.5)) 
                    {
                        std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Deployment: 50 percent" << std::endl;
		    }
                    if ((d_ratio_bef < 0.75)&&(d_ratio_aft >= 0.75)) 
                    {
                        std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Deployment: 75 percent" << std::endl;
		    }
                    if ((d_ratio_bef < 0.9)&&(d_ratio_aft >= 0.9)) 
                    {
                        std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Deployment: 90 percent" << std::endl;
		    }
		    this->i_current_init_index++;
		    this->updateArmCommandVectors(
                        (*this->pvvd_init_arm_profile)[this->i_current_init_index]);
                }
            }
            else
            {
	        this->updateArmCommandVectors(
                        (*this->pvvd_init_arm_profile)[this->i_current_init_index]); 
                this->assignPresentCommand(j_next_arm_command);
		if ((this->isArmReady(j_next_arm_command, j_present_joints_m)))
		{
                    b_is_ready = true;
		}
		if (((*this->pvd_init_time_profile)
                             [(*this->pvvd_init_arm_profile).size() - 1]
                         * 1.0 < d_elapsed_time)&&((this->isArmReady(j_next_arm_command, j_present_joints_m))))

                {
                    b_is_finished = true;
                    std::cout << "[MM] \033[1;35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Deployment is finished" << std::endl;
                }
	    }
	    break;

        case 1: // Retrieval
	    if (this->i_current_retrieval_index < (*this->pvvd_retrieval_arm_profile).size() - 1)
            {
                d_current_timelimit = (*this->pvd_retrieval_time_profile)[this->i_current_retrieval_index] * 1.0;
	        //std::cout << "The current time limit is " << d_current_timelimit << std::endl;
                if ((*this->pvd_retrieval_time_profile)[this->i_current_retrieval_index]
                    * 1.0 <= d_elapsed_time) // TODO - ADHOC value to make this slower
                {
                    this->i_current_retrieval_index++;
                    this->updateArmCommandVectors(
                        (*this->pvvd_retrieval_arm_profile)
                        [this->i_current_retrieval_index]);
                }
            }
            else
	    {	
		this->updateArmCommandVectors(
                        (*this->pvvd_retrieval_arm_profile)
                        [this->i_current_retrieval_index]);
                this->assignPresentCommand(j_next_arm_command);
       		if ((this->isArmReady(j_next_arm_command, j_present_joints_m)))
		{
                    b_is_ready = true;
		}
                if (((*this->pvd_retrieval_time_profile)
                     [(*this->pvvd_retrieval_arm_profile).size() - 1]
                 * 1.0
                < d_elapsed_time)&&((this->isArmReady(j_next_arm_command, j_present_joints_m))))
                {
                    b_is_finished = true;
                }
	    }
            break;
        case 2:  // Coverage
	    if (this->i_current_coverage_index < (*this->pvvd_arm_sweeping_profile).size() - 1)
            {
                d_current_timelimit = (*this->pvd_arm_sweeping_times)[this->i_current_coverage_index]
                    * 1.5 + 5.0;
	        //std::cout << "The current time limit is " << d_current_timelimit << std::endl;
                if ( d_current_timelimit <= d_elapsed_time) // TODO - ADHOC value to make this slower
                {
                    this->i_current_coverage_index++;
                    std::cout << "[MM] \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Going to Coverage Point " << this->i_current_coverage_index << "/" << ((*this->pvvd_arm_sweeping_profile).size() - 1) << std::endl;
//                    std::cout << " \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Elapsed time: " << d_elapsed_time << "sec" << std::endl;
//                    std::cout << " \033[35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Time limit: " << (*this->pvd_arm_sweeping_times)[this->i_current_coverage_index] *1.5 + 5.0 << " sec" << std::endl;
                    this->updateArmCommandVectors((*this->pvvd_arm_sweeping_profile)
                                              [this->i_current_coverage_index]);
                }
            }
            else if (((*this->pvd_arm_sweeping_times)
                     [(*this->pvvd_arm_sweeping_profile).size() - 1]
                 * 1.5 + 5.0
             < d_elapsed_time)&&((this->isArmReady(j_next_arm_command, j_present_joints_m))))

            {
                b_is_finished = true;
                std::cout << "[MM] \033[1;35m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Coverage is finished" << std::endl;
            }
	    else
	    {
                //std::cout << " \033[33m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m Coverage is still not finished" << std::endl;
                //std::cout << " \033[33m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m  - Elapsed time: " << d_elapsed_time << " seconds" << std::endl;
                /*std::cout << " \033[33m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m  - Time to reach: " << (*this->pvd_arm_sweeping_times)
                     [(*this->pvvd_arm_sweeping_profile).size() - 1]
                 * 1.5 + 5.0 <<  " seconds " << std::endl;*/
                //std::cout << " \033[33m[----------] [MobileManipExecutor::getAtomicCommand()]\033[0m  - isArmReady() is returning " << ((this->isArmReady(j_next_arm_command, j_present_joints_m))) << std::endl;
	    }
            break;
    }

    if ((!b_is_ready)&&(!this->isArmMoving(j_present_joints_m))&&(this->d_operational_time>5.0)&&(ui_mode != 2)) //ADHOC time
    {
	std::vector<double> vd_present_joints;
	vd_present_joints.resize(6);

	for (uint i = 0; i < this->ui_num_joints; i++) // TODO: adhoc number of joints = 6
        {
           vd_present_joints[i] 
                = j_present_joints_m.m_jointStates[i].m_position;
        }
	this->updateArmCommandVectors(vd_present_joints);
        this->assignPresentCommand(j_next_arm_command);
        
        return 5;
    }

    // std::cout << "The Retrieval Index is " << i_current_init_index <<
    // std::endl;
    // std::cout << "Creating the proxy library Joints command" << std::endl;
    this->assignPresentCommand(j_next_arm_command);
    if (b_is_finished)
    {
        return 1;
    }
    else
    {
        return 0;
    }
   
}

void MobileManipExecutor::fixMotionCommand(proxy_library::MotionCommand &mc_m)
{
    if ((abs(mc_m.m_speed_ms) < 0.0000001)
        && (abs(mc_m.m_turnRate_rads) > 0.0000001))
    {
        mc_m.m_manoeuvreType = 1;
    }
    else
    {
        if ((abs(mc_m.m_speed_ms) > 0.0000001)
            && (abs(mc_m.m_turnRate_rads) > 0.0000001))
        {
            mc_m.m_curvature_radm = mc_m.m_turnRate_rads / mc_m.m_speed_ms;
            mc_m.m_manoeuvreType = 0;
        }
        else
        {
            if ((abs(mc_m.m_speed_ms) > 0.0000001)
                && (abs(mc_m.m_turnRate_rads) < 0.0000001))

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
    return this->p_collision_detector->isColliding(
        this->vd_arm_present_readings);
}

bool MobileManipExecutor::isArmReady(
    proxy_library::Joints &j_next_command,
    const proxy_library::Joints &j_present_joints)
{
    //std::cout << "Executing isArmReady" << std::endl;
    if (j_next_command.m_jointStates.size() != 6)
    {
        std::cout << " \033[1;33m[----------] [MobileManipExecutor::isArmReady()]\033[0m WARNING: The input j_next_command is empty" << std::endl;
        if (!this->assignPresentCommand(j_next_command))
	{
	    return false;
	}
    }
    //std::cout << "Checked" << std::endl;
    for (uint i = 0; i < 6; i++)
    {
        if (abs(j_next_command.m_jointStates[i].m_position
                - j_present_joints.m_jointStates[i].m_position)
            > 0.05)
        {
            return false;
        }
    }

    return true;
}

bool MobileManipExecutor::isArmMoving(
    const proxy_library::Joints &j_present_joints)
{
    bool isMoving = false;
    for (uint i = 0; i < 6; i++)
    {
        if (abs(j_present_joints.m_jointStates[i].m_speed) > 0.0001)
	{
            isMoving = true;
	}
    }
    return isMoving;
}

bool MobileManipExecutor::isCoupledMoving()
{
    return this->armstate == COUPLED_MOVING;
}

bool MobileManipExecutor::isArmFollowing(
    const proxy_library::Joints &j_next_command,
    const proxy_library::Joints &j_present_joints)
{
    double d_deg2rad = 3.1416 / 180.0;
    bool isMoving = true;
    double d_margin;
    for (uint i = 0; i < 6; i++)
    {
        // std::cout << "In joint " << i << " the previous command is " <<
        // this->vd_arm_previous_command[i] << " and the current pos is " <<
        // j_present_joints.m_jointStates[i].m_position << std::endl;
        d_margin = max(1.2
                           * abs(this->vd_arm_previous_command[i]
                                 - this->vd_arm_present_command[i]),
                       this->vd_arm_posmargin[i]);
        if (abs(this->vd_arm_previous_command[i]
                - j_present_joints.m_jointStates[i].m_position)
            > d_margin) // TODO - ADhoc threshold in radians
        {
            isMoving = false;
            // std::cout << "Error = " << abs(this->vd_arm_previous_command[i] -
            // j_present_joints.m_jointStates[i].m_position) << " rad" <<
            // std::endl;
        }
    }
    return isMoving;
}

proxy_library::MotionCommand MobileManipExecutor::getZeroRoverCommand()
{
    proxy_library::MotionCommand mc_zero;
    mc_zero.m_manoeuvreType = 0;    // 0: Ackermann, 1: PointTurn
    mc_zero.m_curvature_radm = 0.0; // in radians/meter
    mc_zero.m_speed_ms = 0.0;       // in meters/seconds
    mc_zero.m_turnRate_rads = 0.0;  // in radians/seconds
    return mc_zero;
}

bool MobileManipExecutor::getLastSectionCommand(base::Pose &rover_pose, proxy_library::MotionCommand &mc)
{
    // Returns whether the rover is in last section or not
    double dx,dy,dist,dyaw,dtargetheading, x0,y0,dTransformAngle;
    dyaw = rover_pose.getYaw();
    dx = (*this->vpw_path.back()).position[0] - rover_pose.position[0];  
    dy = (*this->vpw_path.back()).position[1] - rover_pose.position[1];  
    dTransformAngle = dyaw - 0.5*3.1416;
    x0 = cos(dTransformAngle)*dx + sin(dTransformAngle)*dy;
    y0 = - sin(dTransformAngle)*dx + cos(dTransformAngle)*dy;
    /*
	std::cout << " - dyaw = " << dyaw << std::endl; 
	std::cout << " - dtransform = " << dTransformAngle << std::endl; 
	std::cout << " - dx = " << dx << std::endl; 
	std::cout << " - dy = " << dy << std::endl; 
	std::cout << " - x0 = " << x0 << std::endl; 
	std::cout << " - y0 = " << y0 << std::endl; 
*/
    dist = sqrt(pow(dx,2)+pow(dy,2));
    dtargetheading =  (*this->vpw_path.back()).heading;

    unsigned int ui_counter, ui_x0_min, ui_x0_max, ui_y0_min, ui_y0_max;
    double d_K, d_x, d_y, d_x1, d_x2, d_y1, d_y2, d_R11, d_R12, d_R21, d_R22;
    //Finding x0 indexes 
    if ((x0 <=(*this->pvd_lsc_x0)[0])||(x0 >= (*this->pvd_lsc_x0).back()))
    {
        //std::cout << " It is not in Last Section" << std::endl;
	return false;
    }
    else if ((y0 <=(*this->pvd_lsc_y0)[0])||(y0 >= (*this->pvd_lsc_y0).back()))
    { 
        //std::cout << " It is not in Last Section" << std::endl;
	return false;
    }
    else
    {
	ui_counter = 1;
        while(ui_counter < this->pvd_lsc_x0->size())
	{
            if ((*this->pvd_lsc_x0)[ui_counter] > x0)
	    {
                ui_x0_min = ui_counter-1;
	        ui_x0_max = ui_counter;
	        break;	
	    }
	    ui_counter++;
	}       
        ui_counter = 1;	
	while(ui_counter < this->pvd_lsc_y0->size())
	{
            if ((*this->pvd_lsc_y0)[ui_counter] > y0)
	    {
                ui_y0_min = ui_counter-1;
	        ui_y0_max = ui_counter;
	        break;	
	    }
	    ui_counter++;
	}
        d_x = x0; 
	d_y = y0;
        d_x1 = (*this->pvd_lsc_x0)[ui_x0_min];
        d_x2 = (*this->pvd_lsc_x0)[ui_x0_max];
        d_y1 = (*this->pvd_lsc_y0)[ui_y0_min];
        d_y2 = (*this->pvd_lsc_y0)[ui_y0_max];
        d_R11 = (*this->pvvd_turning_curvature_matrix)[ui_y0_min][ui_x0_min];
        d_R12 = (*this->pvvd_turning_curvature_matrix)[ui_y0_max][ui_x0_min];
        d_R21 = (*this->pvvd_turning_curvature_matrix)[ui_y0_min][ui_x0_max];
        d_R22 = (*this->pvvd_turning_curvature_matrix)[ui_y0_max][ui_x0_max];
        d_K = - computeBilinearInterpolation(d_x, d_y, d_x1, d_x2, d_y1, d_y2, d_R11, d_R12, d_R21, d_R22);  // It must be negative, it is turning right always
        /*
	std::cout << " In Last Section" << std::endl;
        std::cout << "  - x0 indexes: " << ui_x0_min << " and " << ui_x0_max << std::endl;
        std::cout << "  - y0 indexes: " << ui_y0_min << " and " << ui_y0_max << std::endl;
        std::cout << " Size of Curvature matrix is " << (*this->pvvd_turning_curvature_matrix).size() << " x " << (*this->pvvd_turning_curvature_matrix)[0].size() << std::endl; 
	std::cout << " - dyaw = " << dyaw << std::endl; 
	std::cout << " - dtransform = " << dTransformAngle << std::endl; 
	std::cout << " - x0 = " << d_x << std::endl; 
	std::cout << " - y0 = " << d_y << std::endl; 
	std::cout << " - R11 = " << d_R11 << std::endl; 
	std::cout << " - R12 = " << d_R12 << std::endl; 
	std::cout << " - R21 = " << d_R21 << std::endl;  
	std::cout << " - R22 = " << d_R22 << std::endl; 
	std::cout << " - K = " << d_K << std::endl; 
	std::cout << " - Radius = " << 1.0/d_K << std::endl; 
        std::cout << " Size of Omegamatrix is " << (*this->pvvd_turning_angle_matrix).size() << " x " << (*this->pvvd_turning_angle_matrix)[0].size() << std::endl; 
        std::cout << " - manoeuvre is " << mc.m_manoeuvreType << std::endl;
        std::cout << " - curvature is " << mc.m_curvature_radm << std::endl;
        std::cout << " - speed is " << mc.m_speed_ms << std::endl;
        std::cout << " - turnrate is " << mc.m_turnRate_rads << std::endl;
	*/
        mc.m_manoeuvreType = 0;
	mc.m_curvature_radm = d_K;
	mc.m_speed_ms = 0.1;
	mc.m_turnRate_rads = 0.1*d_K;
	return true;
    }
}

bool MobileManipExecutor::updateArmPresentReadings(
    const proxy_library::Joints &j_present_joints_m)
{
    if ((j_present_joints_m.m_jointStates.empty())
        || (j_present_joints_m.m_jointStates.size() != this->ui_num_joints))
    {
        return false;
    }
    for (uint i = 0; i < this->ui_num_joints; i++)
    {
        this->vd_arm_present_readings[i]
            = j_present_joints_m.m_jointStates[i].m_position;
    }
    return true;
}

bool MobileManipExecutor::updateArmCommandVectors()
{
    for (uint i = 0; i < this->ui_num_joints; i++)
    {
        this->vd_arm_previous_command[i] = this->vd_arm_present_command[i];
    }
}

bool MobileManipExecutor::updateArmCommandVectors(
    const std::vector<double> &vd_present_command_m)
{
    for (uint i = 0; i < this->ui_num_joints; i++)
    {
        this->vd_arm_previous_command[i] = this->vd_arm_present_command[i];
        this->vd_arm_present_command[i] = vd_present_command_m[i];
    }
}

std::vector<double> *MobileManipExecutor::getArmCurrentReadings()
{
    return &(this->vd_arm_present_readings);
}

std::vector<double> *MobileManipExecutor::getFirstCoverageProfile()
{
    return &((*this->pvvd_arm_sweeping_profile).front());
}

std::vector<double> *MobileManipExecutor::getLastCoverageProfile()
{
    return &((*this->pvvd_arm_sweeping_profile).back());
}
