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


#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;


/*
 * MOBILEMANIPMOTIONPLANNER CONSTRUCTOR
 */

MobileManipMotionPlanner::MobileManipMotionPlanner(
    const RoverGuidance_Dem &navCamDEM,
    string s_configfile_path_m,
    unsigned int ui_operation)
{   

    /*
     * 1 --Initialization--
     */

    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m" 
	         "Creating Mobile Manipulation Component" << std::endl;
    
    // Software always starts in IDLE status and error code is NO_ERROR
    this->status = IDLE;
    this->error = NO_ERROR;
    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m"
	         "Initialized status to IDLE" << std::endl;

    // Storing the path to the configuration files 
    this->s_configfile_path = s_configfile_path_m;  
    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m" 
	         "Path to Configuration File is " << s_configfile_path_m << 
		 std::endl;


    /*
     * 2 --Creating Mobile Manipulation Map Class--
     */   

    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m" 
	         "Creating MMMap Class"<< std::endl;

    // New instance of MobileManipMap() is created
    this->p_mmmap = new MobileManipMap();

    // Reading configuration values for the map 
    if (!this->readConfigFile())
    {
        std::cout << 
		 "[MM] \033[31m[-WARNING--] [MobileManipMotionPlanner()]\033[0m"
                 "Config file could not be read, using default values instead" 
		  << std::endl;
    }

    // Storing and processing the input Navigation DEM
    if (!this->updateNavCamDEM(navCamDEM))
    {
        std::cout << 
	       "[MM] \033[1;31m[--ERROR!--] [MobileManipMotionPlanner()]\033[0m"
               " Map Class could not be created" << std::endl;
        this->printErrorCode();
    }
    else
    {
        std::cout << 
	       "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m"
	       " Map Class Successfully created" << std::endl;
    }


    /* 
     * 3 --Creating Motion Plan Class--
     */
   
    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m"
	         " Creating MotionPlan Class"<< std::endl;
    
    // New instance of MotionPlan is created
    this->p_motionplan = new MotionPlan(this->p_mmmap, s_configfile_path_m); 
    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m"
	         " Motion Plan Class Successfully created" << std::endl;
   

    /* 
     * 4 --Creating Mobile Manipulation Executor Class--
     */  

    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m"
	         " Creating MMExecutor Class"<< std::endl;

    // New instance of Executor is created
    this->p_mmexecutor = new MobileManipExecutor(this->p_motionplan, 
		                                 s_configfile_path_m);
    
    // The type of arm-sample operation is set
    this->setArmTargetOperation(ui_operation);
    std::cout << "[MM] \033[32m[----------] [MobileManipMotionPlanner()]\033[0m"
	    " Executor Class Successfully created" << std::endl;
    
    std::cout << 
	       "[MM] \033[1;32m[--DONE!---] [MobileManipMotionPlanner()]\033[0m"
	       " MMMP Class Successfully created" << std::endl;


}


/*
 * readConfigFile()
 *  - It reads the configuration values from a file to later build the cost map
 */

bool MobileManipMotionPlanner::readConfigFile()
{

    // Reading a external file may trigger an exception
    // We use the try() statement to handle this

    try
    {
        
	// Complete the path with the name of the configuration file
	std::string s_config_path = this->s_configfile_path + 
		                    "/path_planner_config.txt"; 
        
	// Creating an ifstream variable linked to the conf. path
	std::ifstream e_file(s_config_path.c_str(), std::ios::in);
        
        // Check if the file exists	
	if (e_file.is_open())
        {
            // The string to store the characters read 
	    std::string cell;
           
	    // Reading the slope threshold 
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_slope_threshold = stof(cell);
	    
	    // Reading the spherical deviation (roughness) threshold 
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_sd_threshold = stof(cell);
            
	    // Reading the Valid/Total Pixels Ratio Threshold 
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_valid_ratio_threshold = stof(cell);
            
	    // Reading the Contour/Valid Pixels Ratio Threshold
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_contour_ratio_threshold = stof(cell);
            
	    // Reading the number of CLOSE operation iterations 
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    int i_close_iter = (int)stof(cell);

	    // Reading the Avoidance distance
            std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_avoid_dist = stof(cell);
            
	    // Reading the Occupancy Radius
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_occ_radius = stof(cell);

            // Reading the Minimal Reachable Distance
            std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_min_reach = stof(cell);
            
	    // Reading the Maximum Reachable Distance 
	    std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_max_reach = stof(cell);

            // Reading the Obstacle Dilation Distance
            std::getline(e_file, cell); std::getline(e_file, cell); 
	    double d_dilation = stof(cell);

            // Reading the Underneath Cost Clearing Option
            std::getline(e_file, cell); std::getline(e_file, cell); 
	    bool b_clear_underneath = (bool)stof(cell);

	    std::cout << "[MM] \033[32m[----------] [readConfigFile()]\033[0m"
                     " Temptative threshold values are " << d_slope_threshold << 
		    ", " << d_sd_threshold << ", " << d_valid_ratio_threshold <<
		    ", " << d_contour_ratio_threshold << std::endl;
            
	    // The Threshold Values for the Cost Map are set
	    this->p_mmmap->setThresholdValues(d_slope_threshold, 
			                      d_sd_threshold, 
					      d_valid_ratio_threshold, 
					      d_contour_ratio_threshold);
	    
	    // The Configuration Variables for the Cost Map are set
	    this->p_mmmap->setConfigValues(i_close_iter, d_avoid_dist, d_occ_radius,
			                   d_min_reach, d_max_reach, d_dilation, 
					   b_clear_underneath);
	    
	    // Correct Configuration File Reading
	    return true;

        }
        else
        {
            
	    // File is not accesible
	    throw std::exception();

        }   
    }
    catch (std::exception &e)
    {
        
        // Something happened that prevented from successfully reading
        return false;

    }

}


/*
 * setArmTargetOperation()
 *  - Function to set the operation the arm will perform with the sample
 */

bool MobileManipMotionPlanner::setArmTargetOperation(unsigned int ui_operation)
{

    std::cout << "[MM] \033[32m[----------] [setOperationMode()]\033[0m"
	         " Setting new Arm Operation" << std::endl;
    
    // Check if the software is in the initial (IDLE) state 
    if (this->status == IDLE)
    {
   
        this->p_mmexecutor->setOperationMode(ui_operation, 
			                     this->s_configfile_path);
        std::cout << "[MM] \033[1;32m[----------] [setOperationMode()]\033[0m"
		     " Target Arm Operation is ";
	
	// An operation is chosen
	switch (ui_operation)
	{
	
            // PICK operation: the arm deploys and takes something
            case 0:
		    std::cout << " PICK" << std::endl;
		    break;
            
	    // DROP operation: the arm deploys and drops something
	    case 1:
		    std::cout << " DROP" << std::endl;
		    break;
            
	    // The arm makes a coverage movement with the arm
	    default:
		    std::cout << " SWEEPING" << std::endl;
		    break;
	}
	return true;

    }
    else
    {

        std::cout << "[MM] \033[1;31m[--ERROR---] [setOperationMode()]\033[0m"
		     " MM not in IDLE Status";
        setError(IMPROPER_CALL);
        return false;

    }

}


/*
 * initArmReset()
 *  - Initialize an Arm Reset Operation
 *  - This operation makes the arm return to its home position
 */

bool MobileManipMotionPlanner::initArmReset(
    const proxy_library::Joints &j_present_readings)
{
    
    // Check if the software is in the initial (IDLE) state 
    if (this->status == IDLE)
    {
        
        std::cout << "[MM] \033[32m[----------] [initArmReset()]\033[0m"
		     " Planning Arm Reset Motion" << std::endl;
        
	// TODO: (remove this?) Executor resets its internal operation clock
	this->p_mmexecutor->resetOperationTime();
       
        // Getting the values of position from each joint	
	std::vector<double> vd_arm_readings;
        vd_arm_readings.resize(6); // TODO: adhoc number of joints = 6
	for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
        {
            vd_arm_readings[i] = j_present_readings.m_jointStates[i].m_position;
        }

        // MotionPlan generates a profile for the arm retrieval
	std::cout << "[MM] \033[32m[----------] [initArmReset()]\033[0m"
		     " Computing arm retrieval motion plan" << std::endl;
        if (this->p_motionplan->computeArmRetrieval(vd_arm_readings) != 0)
        {
            return false;
        } 
	std::cout << "[MM] \033[32m[----------] [initArmReset()]\033[0m"
		     " Arm retrieval motion plan computed with " << 
		     this->p_motionplan->getNumberRetrievalSamples() << 
		     " samples" << std::endl;
        
        // The executor is updated with the new plan for the retrieval	
	this->p_mmexecutor->updateRetrieval();
	std::cout << "[MM] \033[32m[----------] [initArmReset()]\033[0m"
		     " Executor is updated with new motion plans" << std::endl;

        // The arm is considered already deployed
        this->b_is_atomic_deployed = true;
	
	// Executor resets its internal operation clock
        this->p_mmexecutor->resetOperationTime();
       
        // Going directly to execution status	
        setStatus(EXECUTING_ATOMIC_OPERATION);
        
	std::cout << "[MM] \033[1;32m[----------] [initArmReset()]\033[0m"
		     " Arm Reset Plan Computed, current status is "
		     "EXECUTING_ATOMIC_OPERATION" << std::endl; 
        
	return true;

    }
    else
    {
        
        setError(IMPROPER_CALL);
        return false;

    }
}


/*
 * initAtomicOperation()
 *  - This operation makes the arm move while the rover stands still.
 */

bool MobileManipMotionPlanner::initAtomicOperation(
    const proxy_library::Joints &j_present_readings,
    const base::Waypoint &w_goal,
    double d_roll,
    double d_pitch,
    double d_yaw)
{
    if (this->status == IDLE)
    {
        std::cout << "[MM] \033[32m[----------] [initAtomicOperation()]\033[0m Planning Atomic Operation Motion" << std::endl;
        this->p_mmexecutor->resetOperationTime();
        std::vector<double> vd_arm_readings;
        vd_arm_readings.resize(6);
        for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
        {
            vd_arm_readings[i] = j_present_readings.m_jointStates[i].m_position;
        }
        std::vector<double> vd_orientation_goal;
        vd_orientation_goal.resize(3);
        vd_orientation_goal[0] = d_roll;
        vd_orientation_goal[1] = d_pitch;
        vd_orientation_goal[2] = d_yaw;
        std::cout << "[MM] \033[32m[----------] [initAtomicOperation()]\033[0m Computing arm deployment motion plan" << std::endl;
        
        unsigned int ui_code = this->p_motionplan->computeArmDeployment(
                w_goal, vd_orientation_goal, vd_arm_readings);
	if (ui_code != 0)
        {
            
            std::cout << "[MM] \033[1;31m[----------] [initAtomicOperation()]\033[0m Could not compute arm deployment" << std::endl;
	    setError(UNREACH_GOAL);
	    return false;//TODO - It must set the corresponding error
        }
        std::cout << "[MM] \033[32m[----------] [initAtomicOperation()]\033[0m Arm deployment motion plan computed" << std::endl;
        std::vector<double> *pvd_arm_goal;
	if (this->p_motionplan->isInitArmMotionProfileEmpty())
	{
            std::cout << " \033[32m[----------] [initAtomicOperation()]\033[0m Arm deployment motion plan is empty" << std::endl;
            pvd_arm_goal = &(vd_arm_readings);
	}
	else
	{
            pvd_arm_goal =this->p_motionplan->getBackInitArmMotionProfile(); 
		   /*for (uint i = 0; i<pvd_arm_goal->size(); i++)
	    {
                std::cout << "Init joint " << i << " is " << (*pvd_arm_goal)[i] << std::endl;
	    }*/
	}
        std::cout << "[MM] \033[32m[----------] [initAtomicOperation()]\033[0m Arm deployment motion plan computed with " << this->p_motionplan->getNumberDeploymentSamples() << " samples" << std::endl;
        std::cout << "[MM] \033[32m[----------] [initAtomicOperation()]\033[0m Computing arm retrieval motion plan" << std::endl;
        if (this->p_motionplan->computeArmRetrieval((*pvd_arm_goal)) != 0)
        {
            return false;
        }
        std::cout << "[MM] \033[32m[----------]\033[0m Arm retrieval motion plan computed with " << this->p_motionplan->getNumberRetrievalSamples() << " samples" << std::endl;
        this->p_mmexecutor->updateDeployment();
        this->p_mmexecutor->updateRetrieval();
        this->b_is_atomic_deployed = false;
        this->p_mmexecutor->resetOperationTime();
        std::cout << "[MM] \033[32m[----------]\033[0m Executor is updated with new motion plans" << std::endl;
        setStatus(EXECUTING_ATOMIC_OPERATION);
        std::cout << "[MM] \033[1;32m[----------] [initAtomicOperation()]\033[0m Atomic Operation Plan Computed, current status is EXECUTING_ATOMIC_OPERATION" << std::endl; 
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}

unsigned int MobileManipMotionPlanner::updateAtomicOperation(
    proxy_library::Joints &arm_command,
    proxy_library::Joints arm_joints,
    bool b_display_status)
{
    unsigned int ui_error_code = 0;
    if (this->status == EXECUTING_ATOMIC_OPERATION)
    {
        if (!this->b_is_atomic_deployed)
        {
            if (b_display_status)
            {
                std::cout
                    << " \033[32m[----------]\033[0m [updateAtomicOperation()] "
                       "Getting Deployment Command"
                    << std::endl;
            }
            ui_error_code = this->p_mmexecutor->getAtomicCommand(
                arm_joints, arm_command, 0); // 0 -> Deployment
            if (b_display_status)
            {
                std::cout << " \033[32m[----------]\033[0m "
                             "[updateAtomicOperation()]  Timestamp = "
                          << arm_joints.m_time << std::endl;
                for (uint i = 0; i < 6; i++)
                {
                    std::cout
                        << "             J" << i << ": " << std::fixed
                        << std::setprecision(2)
                        << 180.0 / 3.1416
                               * arm_joints.m_jointStates[i].m_position
                        << "/"
                        << 180.0 / 3.1416
                               * arm_command.m_jointStates[i].m_position
                        << " deg (Error = "
                        << 180.0 / 3.1416
                               * (arm_joints.m_jointStates[i].m_position
                                  - arm_command.m_jointStates[i].m_position)
                        << " deg)" << std::endl;
                }
                std::cout << std::endl;
            }
            switch (ui_error_code)
            {
                case 0:
                    return 0;
                case 1:
                    std::cout << " \033[32m[----------] [updateAtomicOperation()]\033[0m Arm is completely deployed" << std::endl;
                    this->b_is_atomic_deployed = true;
                    this->p_mmexecutor->resetOperationTime();
                    return 1;
                case 2:
                    setError(INCOMPLETE_INPUT);
                    return 3;
                case 3:
                    setError(DEGEN_PATH);
                    return 3;
		case 4:
		    setError(COLLIDING_ARM);
		    return 3;
       		case 5:
		    setError(NON_RESP_ARM);
		    return 3;
                default:
                    return 3;
            }
        }
        else
        {
            if (b_display_status)
            {
                std::cout
                    << " \033[32m[----------]\033[0m [updateAtomicOperation()] "
                       "Getting Retrieval Command"
                    << std::endl;
            }
            ui_error_code = this->p_mmexecutor->getAtomicCommand(
                arm_joints, arm_command, 1); // 1 -> Retrieval
            if (b_display_status)
            {
                std::cout << " \033[32m[----------]\033[0m "
                             "[updateAtomicOperation()] Timestamp = "
                          << arm_joints.m_time << std::endl;
                for (uint i = 0; i < 6; i++)
                {
                    std::cout
                        << "             J" << i << ": " << std::fixed
                        << std::setprecision(2)
                        << 180.0 / 3.1416
                               * arm_joints.m_jointStates[i].m_position
                        << "/"
                        << 180.0 / 3.1416
                               * arm_command.m_jointStates[i].m_position
                        << " deg (Error = "
                        << 180.0 / 3.1416
                               * (arm_joints.m_jointStates[i].m_position
                                  - arm_command.m_jointStates[i].m_position)
                        << " deg)" << std::endl;
                }
                std::cout << std::endl;
            }
            switch (ui_error_code)
            {
                case 0:
                    return 1;
                case 1:
                    std::cout << " \033[1;32m[----------] [updateAtomicOperation()]\033[0m Atomic Operation is finished, current status is IDLE" << std::endl;
                    this->b_is_atomic_deployed = false;
                    setStatus(IDLE);
                    return 2;
                case 2:
                    setError(INCOMPLETE_INPUT);
                    return 3;
                case 3:
                    setError(DEGEN_PATH);
                    return 3;
		case 4:
		    setError(COLLIDING_ARM);
		    return 3;
       		case 5:
		    setError(NON_RESP_ARM);
		    return 3;
                default:
                    return 3;
            }
        }
    }
    else
    {
        if (b_display_status)
        {
            std::cout << " \033[32m[----------]\033[0m "
                         "[updateAtomicOperation()] ERROR: this function can "
                         "only be used in EXECUTING_ATOMIC_OPERATION status"
                      << std::endl;
        }
        setError(IMPROPER_CALL);
        return 3;
    }
}

bool MobileManipMotionPlanner::updateNavCamDEM(
    const RoverGuidance_Dem &navCamDEM)
{
    unsigned int ui_error_code = 0;
    if (getStatus() == IDLE)
    {
        ui_error_code = this->p_mmmap->loadDEM(navCamDEM);
    }
    else
    {
        std::cout << "[MM] \033[31m[--ERROR!--]\033[0m [updateNavCamDEM()] Function called in wrong status";
        setError(IMPROPER_CALL);
	return false;
    }
    
    switch (ui_error_code)
    {
        case 0:
            std::cout << "[MM] \033[32m[----------] [updateNavCamDEM()]\033[0m NavCam DEM is successfully loaded" << std::endl;
            return true;
        case 1:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m DEM resolution is zero or less" << std::endl;
            setError(POOR_DEM);
            return false;
        case 2:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m DEM rows are less than 5" << std::endl;
            setError(POOR_DEM);
            return false;
        case 3:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m DEM columns are less than 5" << std::endl;
            setError(POOR_DEM);
            return false;
        case 4:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m Cannot read the DEM offset" << std::endl;
            setError(POOR_DEM);
            return false;
        case 5:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m DEM info could not be allocated in memory" << std::endl;
            setError(BAD_DEM_ALLOC);
            return false;
        case 6:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m Not enough valid pixels in input NavCamDEM" << std::endl;
            setError(POOR_DEM);
            return false;
        case 7:
            std::cout << "[MM] \033[31m[--ERROR!--] [updateNavCamDEM()]\033[0m Too many holes within NavCamDEM valid area" << std::endl;
            setError(POOR_DEM);
            return false;
    }
}

//-- Generate Motion Plan
bool MobileManipMotionPlanner::generateMotionPlan(
    proxy_library::Pose plpose_m,
    const proxy_library::Joints &j_present_readings,
    double d_sample_pos_x,
    double d_sample_pos_y)
{
    std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Starting to compute the Motion Plan" << std::endl;
    base::Pose basepose_dummy;
    basepose_dummy.orientation = Eigen::Quaterniond(plpose_m.m_orientation.m_w,
                                                    plpose_m.m_orientation.m_x,
                                                    plpose_m.m_orientation.m_y,
                                                    plpose_m.m_orientation.m_z);

    std::vector<double> vd_offset = this->p_mmmap->getOffset();
    base::Waypoint w_sample_globalposition;

    // Offset is substracted to set position in local map frame
    this->w_current_rover_position.position[0]
        = plpose_m.m_position.m_x - vd_offset[0];
    this->w_current_rover_position.position[1]
        = plpose_m.m_position.m_y - vd_offset[1];
    this->w_current_rover_position.position[2] = plpose_m.m_position.m_z;
    this->w_current_rover_position.heading = basepose_dummy.getYaw();

    std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Global Rover Position = ( "  << plpose_m.m_position.m_x << ", " << plpose_m.m_position.m_y << ") m" << std::endl;
    std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Local Rover Position = ( "  << this->w_current_rover_position.position[0] << ", " << this->w_current_rover_position.position[1] << ") m" << std::endl;
    std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Rover Heading = ( "  << this->w_current_rover_position.heading << " rad" << std::endl;
    std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Offset = ( "  << vd_offset[0] << ", " << vd_offset[1] << ") m" << std::endl;

    w_sample_globalposition.position[0] = d_sample_pos_x;
    w_sample_globalposition.position[1] = d_sample_pos_y;

    // Can only be called in IDLE or REPLANNING states
    if ((getStatus() == IDLE)||(getStatus() == REPLANNING))
    {
        unsigned int ui_code = 0;
        // TODO - Since for now there is no computation, the state will go to
        // READY_TO_MOVE
        //setStatus(GENERATING_MOTION_PLAN);
        this->p_mmexecutor->initializeArmVariables(j_present_readings);
        // The cost map must be computed based on FACE method
        ui_code = this->p_mmmap->computeFACE(w_sample_globalposition, this->w_current_rover_position);
	switch (ui_code)
        {
            case 0:
                break;
            case 1:
                setError(POOR_DEM);
                return false;
            case 2:
                setError(OOB_GOAL_POS);
                std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m ERROR: Goal " << w_sample_globalposition.position[0] << ", " << w_sample_globalposition.position[1] << " is outside the DEM " << std::endl;
                //std::cout << "[MM] \033[31m[----------]\033[0m [generateMotionPlan()] DEM info:" ;
		//this->p_mmmap->printDEMinfo();
                return false;
            case 3:
                std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m ERROR: Goal " << w_sample_globalposition.position[0] << ", " << w_sample_globalposition.position[1] << " is placed within obstacle area" << std::endl;
		//TODO: Indicate whether the goal is in dilated obstacle or not...
                setError(OBS_GOAL_POS);
                return false;
	    case 4:
	        std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m Goal is too close, cannot fetch" << std::endl;
                setError(GOAL_TOO_CLOSE);
		return false;
        }

        std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Cost Map edited using FACE method" << std::endl;
        std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Starting to calculate the rover path" << std::endl;	
        
	// To compute the path for the rover base
        ui_code = this->p_motionplan->computeRoverBasePathPlanning(
            this->w_current_rover_position);
        
	switch (ui_code)
        {
            case 0:
                break;
            case 1:
		std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m Rover position is out of DEM boundaries" << std::endl;
                setError(OOB_ROVER_POS);
                return false;
            case 2:
		std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m Rover position is within obstacle area" << std::endl;
                setError(OBS_ROVER_POS); 
                return false;
            case 3:
		std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m There is no sample target" << std::endl;
                setError(PLAN_WO_SAMPLE); // TODO: can it reach here anyways?
                return false;
            case 4:
		std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m Goal is unreachable" << std::endl;
                setError(UNREACH_GOAL);
                return false;
            case 5:
		std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m No smooth path could be obtained" << std::endl;
                setError(DEGEN_PATH);
                return false;
        }
        
	std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Raw Rover Base Path is successfully computed with " << this->p_motionplan->getNumberWaypoints() << " waypoints" << std::endl;
        
	std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Resulting path is going to be shortened" << std::endl; 
	// The rover base path is shortened to stop near the sample
        if (!(this->p_motionplan->shortenPathForFetching()))
        {
	    std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m Goal is too close, cannot fetch" << std::endl;
            
	    double d_dist = sqrt(pow(w_sample_globalposition.position[0]
                 - plpose_m.m_position.m_x,2) + pow(w_sample_globalposition.position[1]
                 - plpose_m.m_position.m_y,2));
	    
            std::cout << "[MM] \033[1;31m[----------] [generateMotionPlan()]\033[0m Distance Rover-Goal is " << d_dist << " meters" << std::endl;
            //this->printRoverPathInfo();
            setError(GOAL_TOO_CLOSE);
            return false;
        } 
	std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Rover Base Path is successfully shortened, with now " << this->p_motionplan->getNumberWaypoints() << " waypoints" << std::endl;

        std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Starting to calculate Arm Motion Profile" << std::endl;
	// The arm positions profile is to be computed
        ui_code = this->p_motionplan->computeArmProfilePlanning();
	switch (ui_code)
        {
            case 0:
                break;
            case 1:
                setError(COLLIDING_PROF);
                return false;
            case 2:
                setError(DEVIATED_PROF); //TODO: Maybe substitute by UNFEASIBLE...?
                return false;
            case 3:
                setError(PLAN_WO_SAMPLE);
                return false;
        }
        std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Arm Profile for Coupled Control is successfully computed" << std::endl;
       
        if (this->getStatus() == IDLE)
	{	
	    // Deployment Computation
	    std::vector<double> vd_arm_readings;
            vd_arm_readings.resize(6);
            for (uint i = 0; i < 6; i++) // TODO: adhoc number of joints = 6
            {
                vd_arm_readings[i] = j_present_readings.m_jointStates[i].m_position;
            }
            if (this->p_motionplan->computeArmDeployment(0, vd_arm_readings) != 0)
            {
                setError(COLLIDING_PROF);
                return false;
            }
            std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Arm Deployment Profile is successfully computed with " << this->p_motionplan->getNumberDeploymentSamples() << " samples"  << std::endl;
        
	    // Retrieval Computation
	    std::vector<double> *pvd_last_profile
                = this->p_mmexecutor->getLastCoverageProfile();
            if (this->p_motionplan->computeArmRetrieval((*pvd_last_profile),1) != 0)
            {
                return false;
            }
            std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Arm Retrieval Profile is successfully computed with " << this->p_motionplan->getNumberRetrievalSamples() << " samples" << std::endl;
	}
	// this->p_motionplan->computeArmDeployment(0,);
	// Adds a dummy waypoint at the end to smoothly turn the rover at the end	
	this->p_motionplan->addSampleWaypointToPath();
        
	std::cout << "[MM] \033[32m[----------] [generateMotionPlan()]\033[0m Added final Turning Waypoint, now path has " << this->p_motionplan->getNumberWaypoints() << " waypoints" << std::endl;
        this->p_mmexecutor->updateMotionPlan();
        std::cout << "[MM] \033[1;32m[--DONE!---] [generateMotionPlan()]\033[0m Executor is updated with new plan" << std::endl;
        if (this->getStatus() == IDLE)
	{
	    setStatus(READY_TO_MOVE);
	}
	else //REPLANNING
	{
            setStatus(EXECUTING_MOTION_PLAN);
	}
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}

bool MobileManipMotionPlanner::start()
{
    if (getStatus() == READY_TO_MOVE)
    {
        setStatus(EXECUTING_MOTION_PLAN);
        std::cout << "[MM] \033[1;32m[----------] [start()]\033[0m Status is EXECUTING_MOTION_PLAN" << std::endl;
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}

bool MobileManipMotionPlanner::abort()
{
    // TODO - Add abort for atomic arm operation
    std::vector<double> *pvd_current_readings;
    switch (getStatus())
    {
        case READY_TO_MOVE:
            setStatus(IDLE);
            return true;
        case EXECUTING_MOTION_PLAN:
            // TODO - complete this
            // this->p_motionplan->computeArmRetrieval(j_current_readings);
            this->p_mmexecutor->resetOperationTime();
            pvd_current_readings = this->p_mmexecutor->getArmCurrentReadings();
            if (this->p_motionplan->computeArmRetrieval((*pvd_current_readings))
                != 0)
            {
                return false;
            }
            this->p_mmexecutor->updateRetrieval();
            setStatus(RETRIEVING_ARM);
            return true;
        case EXECUTING_ARM_OPERATION:
            this->p_mmexecutor->resetOperationTime();
            pvd_current_readings = this->p_mmexecutor->getArmCurrentReadings();
            if (this->p_motionplan->computeArmRetrieval((*pvd_current_readings))
                != 0)
            {
                return false;
            }
            this->p_mmexecutor->updateRetrieval();
            setStatus(RETRIEVING_ARM);
            return true;
        case PAUSE:
            this->p_mmexecutor->resetOperationTime();
            pvd_current_readings = this->p_mmexecutor->getArmCurrentReadings();
            if (this->p_motionplan->computeArmRetrieval((*pvd_current_readings))
                != 0)
            {
                return false;
            }
            this->p_mmexecutor->updateRetrieval();
            setStatus(RETRIEVING_ARM);
            return true;
        default:
            setError(IMPROPER_CALL);
            return false;
    }
}

bool MobileManipMotionPlanner::pause(
    proxy_library::MotionCommand &rover_command)
{
    if ((getStatus() == EXECUTING_MOTION_PLAN)
        || (getStatus() == EXECUTING_ARM_OPERATION)
        || (getStatus() == RETRIEVING_ARM))
    {
        setStatus(PAUSE);
        rover_command.m_speed_ms = 0.0;
        rover_command.m_turnRate_rads = 0.0;
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}

bool MobileManipMotionPlanner::resumeOperation()
{
    if (getStatus() == PAUSE)
    {
        setStatus(this->priorStatus);
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}


/*
 * updateLocCamDEM()
 *  - Receives a Local DEM and checks if a replanning is needed
 */

bool MobileManipMotionPlanner::updateLocCamDEM(
    RoverGuidance_Dem locCamDEM,
    proxy_library::Joints &arm_command,
    proxy_library::MotionCommand &rover_command,
    proxy_library::Joints arm_joints)
{

    // Variable to store the error code received each time a function is called
    unsigned int ui_error_code = 0;

    // Check if the robot is executing the motion plan (moving)
    if (getStatus() == EXECUTING_MOTION_PLAN)
    {
        
        // Checks if the rover is not moving (e.g. stopped near the sample)	    
        if (!this->p_mmexecutor->isCoupledMoving())
	{
           
            // As the rover is not going to move, replanning is not needed	
            std::cout << " \033[32m[----------] [updateNavCamDEM()]\033[0m"
		         " Input LocCam DEM discarded, system not in coupled"
			 " movement" << std::endl;
            return true;

	}

	// The DEM is loaded, the boolean true indicates it is a LocDEM
        ui_error_code = this->p_mmmap->loadDEM(locCamDEM, true); 
	switch (ui_error_code)
        {
           
            // LocDEM is successfully loaded, internal DEM is updated
            case 0:

                std::cout << "[MM] \033[32m[----------]"
			" [updateNavCamDEM()]\033[0m LocCam DEM is "
			"successfully loaded, checking obstacles" << std::endl;
                
	        // Check if current path collides with obstacles	
		if(this->p_motionplan->isPathColliding())
                {

                    std::cout << "[MM] \033[32m[----------]"
			    " [updateNavCamDEM()]\033[0m Detected obstacle"
			    " close to current path, going to REPLANNING status"
			    << std::endl;
                    
		    // Create a command to stop the rover
		    rover_command = this->p_mmexecutor->getZeroRoverCommand();
                    
		    // Create a command to stop the arm
		    for (uint i = 0; i < 6; i++) // TODO: adhoc joints = 6
                    {
                        arm_command.m_jointStates[i].m_position
                            = arm_joints.m_jointStates[i].m_position;
                    }

		    // The status is set to REPLANNING
	    	    setStatus(REPLANNING);
	    	    return false;

	        }
	        else
	        {

                    // Path is still safe to follow, no need to replan
                    std::cout << "[MM] \033[32m[----------]"
			    " [updateNavCamDEM()]\033[0m No obstacles on"
			    " current path" << std::endl;
                    return true;

	        }

            // Error in locDEM resolution
            case 1:
                std::cout << "[MM] \033[1;31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m DEM resolution is zero or"
			" less" << std::endl;
                setError(POOR_DEM);
                return false;

            // Error in number of rows
            case 2:
                std::cout << "[MM] \033[31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m DEM rows are less than 5"
		       	<< std::endl;
                setError(POOR_DEM);
                return false;

            // Error in number of columns
            case 3:
                std::cout << "[MM] \033[31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m DEM columns are less than"
			" 5" << std::endl;
                setError(POOR_DEM);
                return false;

            // Error in offset value
	    case 4:
                std::cout << "[MM] \033[31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m Cannot read the DEM offset"
		       	<< std::endl;
                setError(POOR_DEM);
                return false;

            // Error in memory allocation
	    case 5:
                std::cout << "[MM] \033[31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m DEM info could not be"
			" allocated in memory" << std::endl;
                setError(BAD_DEM_ALLOC);
                return false;

            // Error in amount of valid pixels
	    case 6:
                std::cout << "[MM] \033[31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m Not enough valid pixels"
			" in input NavCamDEM" << std::endl;
                setError(POOR_DEM);
                return false;

            // Error in the quality of the DEM (holes inside)
	    case 7:
                std::cout <<"[MM] \033[31m[--ERROR!--]"
			" [updateNavCamDEM()]\033[0m Too many holes within"
			" NavCamDEM valid area" << std::endl;
                setError(POOR_DEM);
                return false;

	}
        
	return true;

    }
    else if ((this->getStatus() == RETRIEVING_ARM) || 
	     (this->getStatus() == EXECUTING_ARM_OPERATION))
    {

        // The rover is stopped, there is no need to replan
        std::cout << " \033[1;32m[----------] [updateLocCamDEM()]\033[0m"
		" InputLocCAM discarded, replanning not needed" << std::endl;
        return true;

    }
    else
    {

        setError(IMPROPER_CALL);
	return false;

    }

}



bool MobileManipMotionPlanner::updateRoverArmPos(
    proxy_library::Joints &arm_command,
    proxy_library::MotionCommand &rover_command,
    proxy_library::Pose plpose_m,
    proxy_library::Joints arm_joints)
{
    unsigned int ui_error_code = 0;
    switch (getStatus())
    {

        case EXECUTING_MOTION_PLAN:
        {

            std::vector<double> vd_offset = this->p_mmmap->getOffset();


            std::cout << "[MM] \033[32m[----------] " << 
                         "[updateRoverArmPos()]\033[0m Proxy Rover Position" <<
			 " (in global coord.): ( "  << plpose_m.m_position.m_x << ", " << plpose_m.m_position.m_y << ", " << plpose_m.m_position.m_z << ") m" << std::endl;
            std::cout << "[MM] \033[32m[----------] " << 
                         "[updateRoverArmPos()]\033[0m Proxy Rover Orientation" <<
			 ": quaternion = ( x: "  << plpose_m.m_orientation.m_x
			 << ", y: " << plpose_m.m_orientation.m_y << ", z: " << plpose_m.m_orientation.m_z << ", w: " << plpose_m.m_orientation.m_w << ")" << std::endl;


            base::Pose basepose;
            // TODO - Path is in local coordinates, the rover position may be in
            // global!
            basepose.position[0] = plpose_m.m_position.m_x - vd_offset[0];
            basepose.position[1] = plpose_m.m_position.m_y - vd_offset[1];
            basepose.position[2] = plpose_m.m_position.m_z;



            basepose.orientation
                = Eigen::Quaterniond(plpose_m.m_orientation.m_w,
                                     plpose_m.m_orientation.m_x,
                                     plpose_m.m_orientation.m_y,
                                     plpose_m.m_orientation.m_z);
            
	    this->w_current_rover_position.position[0]
                = plpose_m.m_position.m_x;
            this->w_current_rover_position.position[1]
                = plpose_m.m_position.m_y;
            this->w_current_rover_position.position[2]
                = plpose_m.m_position.m_z;
            this->w_current_rover_position.heading = basepose.getYaw();
            // TODO: use w_current_rover_position instead of basepose


            std::cout << "[MM] \033[32m[----------] " << 
                         "[updateRoverArmPos()]\033[0m Input Rover Position to" <<
			 " Executor (in local coord.): ( "  << basepose.position[0] << ", " << basepose.position[1] << ", " << basepose.position[2] << ") m" << std::endl;
            std::cout << "[MM] \033[32m[----------] " << 
                         "[updateRoverArmPos()]\033[0m Input Rover Orientation to" <<
			 " Executor: quaternion = ( x: "  << basepose.orientation.x()
			 << ", y: " << basepose.orientation.y() << ", z: " << basepose.orientation.z() << ", w: " << basepose.orientation.w() << "), heading = " << basepose.getYaw() << std::endl;

	    ui_error_code = this->p_mmexecutor->getCoupledCommand(
                basepose, arm_joints, rover_command, arm_command);


	    switch (ui_error_code)
            {
                case 0: // Deploying arm to initial position
                    return true;
                case 1: // Either driving or aligning
                    return true;
                case 2: // (Rover) Target reached
                    this->p_mmexecutor->resetOperationTime();
                    std::cout << "[MM] \033[32m[----------] [updateRoverArmPos()]\033[0m Finished Coupled Motion" << std::endl;
                    std::cout << "[MM] \033[32m[----------] [updateRoverArmPos()]\033[0m Starting Arm Operation" << std::endl;
                    setStatus(EXECUTING_ARM_OPERATION);
                    return true;
                case 3: // Out of boundaries
                    setError(EXCESSIVE_DRIFT);
                    return false;
                case 4: // Either no trajectory or no pose
                    // TODO - Is this situation even possible to reach??
                    std::cout << "[MM] \033[1;31m[--ERROR!--] [updateRoverArmPos()]\033[0m An strange error occurred, there is no pose??"
                              << std::endl;
                    setError(INCOMPLETE_INPUT);
                    return false;
                case 5:
                    setError(FORB_ARM_POS);
                    return false;
                case 6:
                    setError(COLLIDING_ARM);
                    return false;
                case 7:
                    std::cout << "[MM] \033[1;31m[--ERROR!--] [updateRoverArmPos()]\033[0m ERROR: Arm is not responding" << std::endl;
                    setError(NON_RESP_ARM); // TODO - Shouldnt be better
                                            // NON_FOLLOWING_ARM?
                    return false;
                case 8:// TODO - CHECK THIS
                    setError(UNFEASIBLE_INIT);
                    return false;
		// TODO - There should be a case when a point turn takes too long or the rover does not move at all
		//
            }
            return false;
            break;
        }
        case RETRIEVING_ARM:
            ui_error_code = this->p_mmexecutor->getAtomicCommand(
                arm_joints, arm_command, 1); // 1 -> Retrieval
            rover_command = this->p_mmexecutor->getZeroRoverCommand();
            if (ui_error_code == 1)
            {
                    std::cout << "[MM] \033[32m[----------] [updateRoverArmPos()]\033[0m Finished retrieving the arm" << std::endl;
                    std::cout << "[MM] \033[1;32m[--DONE!---] [updateRoverArmPos()]\033[0m Mobile Manipulation Task is successfully finished" << std::endl;
		setStatus(FINISHED);
                return false;
            }
            if (ui_error_code == 4)
	    {
                    std::cout << "[MM] \033[31m[--ERROR!--] [updateRoverArmPos()]\033[0m Arm is in collidable configuration" << std::endl;
                setError(COLLIDING_ARM);
		return false;
	    }
            if (ui_error_code == 5)
	    {
                std::cout << "[MM] \033[31m[--ERROR!--] [updateRoverArmPos()]\033[0m Arm is not responding" << std::endl;
                setError(NON_RESP_ARM);
		return false;
	    }
            return true;
            break;
        case EXECUTING_ARM_OPERATION:
            //std::cout << "Status is Executing Arm Operation" << std::endl;
            rover_command = this->p_mmexecutor->getZeroRoverCommand();
            ui_error_code = this->p_mmexecutor->getAtomicCommand(arm_joints,
                                                                 arm_command, 2); // 2 -> Coverage
            if (ui_error_code == 1)
            {
                this->p_mmexecutor->resetOperationTime();
                    std::cout << "[MM] \033[32m[----------] [updateRoverArmPos()]\033[0m Finished Arm Operation" << std::endl;
                    std::cout << "[MM] \033[32m[----------] [updateRoverArmPos()]\033[0m Retrieving arm" << std::endl;
                setStatus(RETRIEVING_ARM);
            }
            if (ui_error_code == 4)
	    {
                std::cout << "[MM] \033[31m[--ERROR!--] [updateRoverArmPos()]\033[0m Arm is in collidable configuration" << std::endl;
                setError(COLLIDING_ARM);
		return false;
	    }
            if (ui_error_code == 5)
	    {
                std::cout << "[MM] \033[31m[--ERROR!--] [updateRoverArmPos()]\033[0m Arm is not responding" << std::endl;
                setError(NON_RESP_ARM);
		return false;
	    }
            return true;
	case REPLANNING:
            rover_command = this->p_mmexecutor->getZeroRoverCommand();
            std::cout << "[MM] \033[1;32m[----------] [updateRoverArmPos()]\033[0m It is in Replanning status" << std::endl;
	    return false;
        case ERROR:
            return false;
        default:
            setError(IMPROPER_CALL);
            return false;
    }
}

void MobileManipMotionPlanner::updateSamplePos(proxy_library::Pose sample)
{
    throw "Not yet implemented";
}

bool MobileManipMotionPlanner::ack()
{
    if (getStatus() == FINISHED)
    {
        setStatus(IDLE);
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}

void MobileManipMotionPlanner::resumeError()
{
    switch (this->error)
    {
        case NO_ERROR:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m It is not in ERROR state" << std::endl;
            break;
        case POOR_DEM:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
           break;
        case POOR_CONFIG:
            break;
        case OOB_ROVER_POS:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
            break;
        case OOB_GOAL_POS:
             std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
           break;
        case OBS_ROVER_POS:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
            break;
        case OBS_GOAL_POS:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
            break;
        case PLAN_WO_SAMPLE:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
            break;
        case UNREACH_GOAL:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
            break;
        case DEGEN_PATH: // Internal error
            break;
        case COLLIDING_PROF: // Internal error
            break;
        case DEVIATED_PROF: // Internal error
            break;
        case FORB_ARM_POS:
            break;
        case INCOMPLETE_INPUT:
            break;
        case NON_RESP_ARM: // Replan?
            break;
        case COLLIDING_ARM: // Fatal error
            break;
        case NON_RESP_ROVER: // Replan?
            break;
        case EXCESSIVE_DRIFT: // Replan
            break;
        case GOAL_TOO_CLOSE:
            std::cout << "[MM] \033[32m[----------] [resumeError()]\033[0m Going to previous state" << std::endl;
            this->setStatus(priorStatus);
            this->setError(NO_ERROR);
            this->printStatus();
            break;
        case BAD_DEM_ALLOC:
            break;
        case IMPROPER_CALL:
            setStatus(priorStatus);
            setError(NO_ERROR);
            break;
    }
}

bool MobileManipMotionPlanner::isStatusError()
{
    return this->status == ERROR;
}

MMError MobileManipMotionPlanner::getErrorCode()
{
    return this->error;
}

MMStatus MobileManipMotionPlanner::getStatus()
{
    return this->status;
}

void MobileManipMotionPlanner::printRoverPathInfo()
{
    std::cout << "[MM] \033[32m[----------]\033[0m [printRoverPathInfo()] Rover Path has "
              << this->p_motionplan->getNumberWaypoints() << " waypoints"
              << std::endl;
}

void MobileManipMotionPlanner::printExecutionInfo()
{
    this->p_mmexecutor->printExecutionStatus();
}

void MobileManipMotionPlanner::printStatus()
{
    std::cout << "[MM] \033[1;32m[----------] [printStatus()]\033[0m Current Status is: ";
    switch (this->status)
    {
        case IDLE:
            std::cout << "IDLE";
            break;
        case EXECUTING_ATOMIC_OPERATION:
            std::cout << "EXECUTING_ATOMIC_OPERATION";
            break;
        case GENERATING_MOTION_PLAN:
            std::cout << "GENERATING_MOTION_PLAN";
            break;
        case READY_TO_MOVE:
            std::cout << "READY_TO_MOVE";
            break;
        case EXECUTING_MOTION_PLAN:
            std::cout << "EXECUTING_MOTION_PLAN";
            break;
        case EXECUTING_ARM_OPERATION:
            std::cout << "EXECUTING_ARM_OPERATION";
            break;
        case RETRIEVING_ARM:
            std::cout << "RETRIEVING_ARM";
            break;
        case FINISHED:
            std::cout << "FINISHED";
            break;
        case ERROR:
            std::cout << "ERROR";
            break;
        case REPLANNING:
            std::cout << "REPLANNING";
            break;
        case PAUSE:
            std::cout << "PAUSE";
            break;
    }
    std::cout << std::endl;
}

void MobileManipMotionPlanner::printErrorCode()
{
    std::cout << "[MM] \033[1;32m[----------] [printErrorCode()]\033[0m Current Error "
                 "Code: ";
    switch (this->error)
    {
        case NO_ERROR:
            std::cout << "NO_ERROR";
            break;
        case POOR_DEM:
            std::cout << "POOR_DEM";
            break;
        case POOR_CONFIG:
            std::cout << "POOR_CONFIG";
            break;
        case OOB_ROVER_POS:
            std::cout << "OOB_ROVER_POS";
            break;
        case OOB_GOAL_POS:
            std::cout << "OOB_GOAL_POS";
            break;
        case OBS_ROVER_POS:
            std::cout << "OBS_ROVER_POS";
            break;
        case PLAN_WO_SAMPLE:
            std::cout << "PLAN_WO_SAMPLE";
            break;
        case OBS_GOAL_POS:
            std::cout << "OBS_GOAL_POS";
            break;
        case UNREACH_GOAL:
            std::cout << "UNREACH_GOAL";
            break;
        case UNCERT_GOAL:
            std::cout << "UNCERT_GOAL";
            break;
        case DEGEN_PATH:
            std::cout << "DEGEN_PATH";
            break;
        case COLLIDING_PROF:
            std::cout << "COLLIDING_PROF";
            break;
        case DEVIATED_PROF:
            std::cout << "DEVIATED_PROF";
            break;
        case FORB_ARM_POS:
            std::cout << "FORB_ARM_POS";
            break;
        case INCOMPLETE_INPUT:
            std::cout << "INCOMPLETE_INPUT";
            break;
        case NON_RESP_ARM:
            std::cout << "NON_RESP_ARM";
            break;
        case COLLIDING_ARM:
            std::cout << "COLLIDING_ARM";
            break;
        case NON_RESP_ROVER:
            std::cout << "NON_RESP_ROVER";
            break;
        case EXCESSIVE_DRIFT:
            std::cout << "EXCESSIVE_DRIFT";
            break;
        case GOAL_TOO_CLOSE:
            std::cout << "GOAL_TOO_CLOSE";
            break;
        case BAD_DEM_ALLOC:
            std::cout << "BAD_DEM_ALLOC";
            break;
        case IMPROPER_CALL:
            std::cout << "IMPROPER_CALL";
            break;
    }
    std::cout << std::endl;
}

void MobileManipMotionPlanner::setError(MMError error_m)
{
    if (error_m != NO_ERROR)
    {
        setStatus(ERROR);
    }
    this->error = error_m;
}

void MobileManipMotionPlanner::setStatus(MMStatus status_m)
{
    this->priorStatus = getStatus();
    this->status = status_m;
}

double MobileManipMotionPlanner::getCurrentRoverYaw()
{
    return this->w_current_rover_position.heading;
}

std::vector<std::vector<double>> *MobileManipMotionPlanner::getWristPath()
{
    // Workaround due to how the wrist path is programmed
    std::cout << "[MM] \033[32m[----------] [getWristPath()]\033[0m Getting pointer to Wrist Path" << std::endl;
    if ((this->status == IDLE)||(this->status == ERROR)||(this->status == REPLANNING))
    {
        std::cout << "[MM] \033[1;32m[----------] [get3DCostMap()]\033[0m Empty Wrist Path" << std::endl;
        return &(this->p_motionplan->vvd_wristpath);
    }
    else
    {
        std::cout << "[MM] \033[1;32m[----------] [get3DCostMap()]\033[0m Wrist Path seems already calculated" << std::endl;
        return this->p_motionplan->getWristPath();
    }
}

std::vector<base::Waypoint> *MobileManipMotionPlanner::getRoverPath()
{
    return this->p_motionplan->getRoverPath();
}

std::vector<double> *MobileManipMotionPlanner::getOffset()
{
    return this->p_mmmap->getPointer2Offset();
}

std::vector<std::vector<double>> *MobileManipMotionPlanner::getCostMap()
{
    return this->p_mmmap->getCostMap();
}


bool MobileManipMotionPlanner::getMorphMaps(
    std::vector<std::vector<double>> &vvd_elevation_map_m,
    std::vector<std::vector<double>> &vvd_slope_map_m,
    std::vector<std::vector<double>> &vvd_sd_map_m,
    std::vector<std::vector<int8_t>> &vvi_validity_map_m)
{
    std::cout << "[MM] \033[32m[----------] [getMorphMaps()]\033[0m Getting Elevation Map" << std::endl;
    if (!this->p_mmmap->getElevationMap(vvd_elevation_map_m))
    {
        std::cout << "[MM] \033[1;33m[--ERROR---] [getMorphMaps()]\033[0m DEM is not loaded" << std::endl;
        return false;    
    }
    std::cout << "[MM] \033[32m[----------] [getMorphMaps()]\033[0m Getting Slope Map" << std::endl;
    this->p_mmmap->getSlopeMap(vvd_slope_map_m);
    std::cout << "[MM] \033[32m[----------] [getMorphMaps()]\033[0m Getting Roughness Map" << std::endl;
    this->p_mmmap->getSDMap(vvd_sd_map_m);
    std::cout << "[MM] \033[32m[----------] [getMorphMaps()]\033[0m Getting Validity Map" << std::endl;
    this->p_mmmap->getValidityMap(vvi_validity_map_m);
    std::cout << "[MM] \033[1;32m[----------] [getMorphMaps()]\033[0m All morph maps are obtained" << std::endl;
    return true;
}

bool MobileManipMotionPlanner::getNavigationMaps(
    std::vector<std::vector<int>> &vvi_traversability_map_m,
    std::vector<std::vector<double>> &vvd_cost_map_m)
{
    std::cout << "[MM] \033[32m[----------] [getNavigationMaps()]\033[0m Getting Traversability Map" << std::endl;
    this->p_mmmap->getTraversabilityMap(vvi_traversability_map_m);
    std::cout << "[MM] \033[32m[----------] [getNavigationMaps()]\033[0m Getting Cost Map" << std::endl;
    this->p_mmmap->getCostMap(vvd_cost_map_m);
    std::cout << "[MM] \033[1;32m[----------] [getNavigationMaps()]\033[0m All navigation maps are obtained" << std::endl;
    return true;
}

std::vector<std::vector<double>>
    *MobileManipMotionPlanner::getArmMotionProfile()
{
    return this->p_motionplan->getCoupledArmMotionProfile();
}

std::vector<std::vector<std::vector<double>>>
    *MobileManipMotionPlanner::get3DCostMap()
{
    // Workaround due to how the 3dcostmap is programmed
    std::cout << "[MM] \033[32m[----------] [get3DCostMap()]\033[0m Getting pointer to 3d Cost Map" << std::endl;
    if ((this->status == IDLE)||(this->status == ERROR)||(this->status == REPLANNING))
    {
        std::cout << "[MM] \033[1;32m[----------] [get3DCostMap()]\033[0m Empty 3d Cost Map" << std::endl;
        return &(this->p_motionplan->vvvd_3d_costmap);
    }
    else
    {
        std::cout << "[MM] \033[1;32m[----------] [get3DCostMap()]\033[0m 3d Cost Map seems already calculated" << std::endl;
        return this->p_motionplan->get3DCostMap();
    }
}
