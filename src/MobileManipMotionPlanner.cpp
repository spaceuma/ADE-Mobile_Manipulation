#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;

//--CONSTRUCTOR
MobileManipMotionPlanner::MobileManipMotionPlanner(
    const RoverGuidance_Dem &navCamDEM,
    const Joints &j_present_readings,
    string s_configfile_path_m)
{
    double d_zres_m = 0.08; // TODO: this must come from an external config file
    this->status = IDLE;
    this->error = NO_ERROR;
    // DEM is introduced into the map class
    this->p_mmmap = new MobileManipMap(navCamDEM);
    // Each class contains a pointer to the previous one
    this->p_motionplan
        = new MotionPlan(this->p_mmmap, d_zres_m, s_configfile_path_m);
    this->p_mmexecutor = new MobileManipExecutor(
        this->p_motionplan, j_present_readings, s_configfile_path_m);
}

//-- Generate Motion Plan
bool MobileManipMotionPlanner::generateMotionPlan(proxy_library::Pose plpose_m,
                                                  double d_sample_pos_x,
                                                  double d_sample_pos_y)
{
    base::Pose basepose_dummy;
    basepose_dummy.orientation = Eigen::Quaterniond(plpose_m.m_orientation.m_w,
                                                    plpose_m.m_orientation.m_x,
                                                    plpose_m.m_orientation.m_y,
                                                    plpose_m.m_orientation.m_z);

    base::Waypoint sample_position;
    this->w_current_rover_position.position[0] = plpose_m.m_position.m_x;
    this->w_current_rover_position.position[1] = plpose_m.m_position.m_y;
    this->w_current_rover_position.position[2] = plpose_m.m_position.m_z;
    this->w_current_rover_position.heading = basepose_dummy.getYaw();

    sample_position.position[0] = d_sample_pos_x;
    sample_position.position[1] = d_sample_pos_y;

    // Can only be called in IDLE state
    if (getStatus() == IDLE)
    {
        unsigned int ui_code = 0;
        // TODO - Since for now there is no computation, the state will go to
        // READY_TO_MOVE
        setStatus(GENERATING_MOTION_PLAN);
	// The cost map must be computed based on FACE method
        ui_code = this->p_mmmap->computeFACE(sample_position);
	switch (ui_code)
	{
            case 0:
		break;
	    case 1:
                setError(POOR_DEM);
		return false;
	    case 2:
		setError(OOB_GOAL_POS);
		return false;
	    case 3:
		setError(OBS_GOAL_POS);
		return false;
	}
	// To compute the path for the rover base
        ui_code = this->p_motionplan->computeRoverBasePathPlanning(
            this->w_current_rover_position);
        switch (ui_code)
        {
            case 0:
                break;
            case 1:
                setError(OOB_ROVER_POS);
                return false;
            case 2:
                setError(OBS_ROVER_POS);
                return false;
            case 3:
                setError(PLAN_WO_SAMPLE);
                return false;
            case 4:
                setError(GOAL_TOO_CLOSE);
                return false;
            case 5:
                setError(DEGEN_PATH);
                return false;
        }
	// The rover base path is shortened to stop near the sample
        if (!(this->p_motionplan->shortenPathForFetching()))
        {
            setError(GOAL_TOO_CLOSE);
            return false;
        }
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
                setError(DEVIATED_PROF);
                return false;
	    case 3:
		setError(PLAN_WO_SAMPLE);
		return false;
        }
        this->p_mmexecutor->updateMotionPlan();
        setStatus(READY_TO_MOVE);
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
    switch (getStatus())
    {
        case READY_TO_MOVE:
            setStatus(IDLE);
            return true;
        case EXECUTING_MOTION_PLAN:
            setStatus(RETRIEVING_ARM);
            return true;
        case EXECUTING_ARM_OPERATION:
            setStatus(RETRIEVING_ARM);
            return true;
        case PAUSE:
            setStatus(RETRIEVING_ARM);
            return true;
        default:
            setError(IMPROPER_CALL);
            return false;
    }
}

bool MobileManipMotionPlanner::pause(MotionCommand &rover_command)
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

void MobileManipMotionPlanner::updateLocCamDEM(
    RoverGuidance_Dem locCamDEM,
    proxy_library::Pose rover_position,
    Joints arm_joints)
{
    throw "Not yet implemented";
}

bool MobileManipMotionPlanner::updateRoverArmPos(Joints &arm_command,
                                                 MotionCommand &rover_command,
                                                 proxy_library::Pose plpose_m,
                                                 Joints arm_joints)
{

    unsigned int ui_retrieval_code;
    switch (getStatus())
    {
        case EXECUTING_MOTION_PLAN:
        {
            base::Pose basepose;
	    // TODO - Path is in local coordinates, the rover position may be in global!
            basepose.position[0] = plpose_m.m_position.m_x;
            basepose.position[1] = plpose_m.m_position.m_y;
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
            unsigned int ui_error_code = this->p_mmexecutor->getCoupledCommand(
                basepose, arm_joints, rover_command, arm_command);
            switch (ui_error_code)
            {
                case 0: // Deploying arm to initial position
                    return true;
                case 1: // Either driving or aligning
                    return true;
                case 2: // (Rover) Target reached
                    setStatus(EXECUTING_ARM_OPERATION);
                    return true;
                case 3: // Out of boundaries
                    setError(EXCESSIVE_DRIFT);
                    return false;
                case 4: // Either no trajectory or no pose
                    // TODO - Is this situation even possible to reach??
                    std::cout << "An strange error occurred, there is no pose??"
                              << std::endl;
                    setError(INCOMPLETE_INPUT);
                    return false;
                case 5:
                    setError(FORB_ARM_POS);
                    return false;
                case 6:
                    setError(COLLIDING_ARM);
                    return false;
            }
            return false;
            break;
        }
        case RETRIEVING_ARM:
            ui_retrieval_code = this->p_mmexecutor->getRetrievalCommand(
                arm_joints, arm_command);
            if (ui_retrieval_code == 2)
            {
                return false;
            }
            return true;
            break;
        case EXECUTING_ARM_OPERATION:
            std::cout << "Status is Executing Arm Operation" << std::endl;
            setStatus(RETRIEVING_ARM);
            return true;
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
            break;
        case POOR_DEM:
            break;
        case POOR_CONFIG:
            break;
        case OOB_ROVER_POS:
            break;
        case OOB_GOAL_POS:
            break;
        case OBS_ROVER_POS:
            break;
        case OBS_GOAL_POS:
            break;
	case PLAN_WO_SAMPLE:
	    break;
        case UNREACH_GOAL:
            break;
        case UNCERT_GOAL:
            break;
        case DEGEN_PATH:
            break;
        case COLLIDING_PROF:
            break;
        case DEVIATED_PROF:
            break;
        case FORB_ARM_POS:
            break;
        case INCOMPLETE_INPUT:
            break;
        case NON_RESP_ARM:
            break;
        case COLLIDING_ARM:
            break;
        case NON_RESP_ROVER:
            break;
        case EXCESSIVE_DRIFT:
            break;
        case UNCERT_HEADING:
            break;
        case GOAL_TOO_CLOSE:
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
    std::cout << " The Rover Path has "
              << this->p_motionplan->getNumberWaypoints() << " waypoints"
              << std::endl;
}

void MobileManipMotionPlanner::printStatus()
{
    std::cout << "Current Status: ";
    switch (this->status)
    {
        case IDLE:
            std::cout << "IDLE";
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
    std::cout << "Current Error Code: ";
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
        case UNCERT_HEADING:
            std::cout << "UNCERT_HEADING";
            break;
        case GOAL_TOO_CLOSE:
            std::cout << "GOAL_TOO_CLOSE";
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
    return this->p_motionplan->getWristPath();
}

std::vector<base::Waypoint> *MobileManipMotionPlanner::getRoverPath()
{
    return this->p_motionplan->getRoverPath();
}

std::vector<std::vector<double>>
    *MobileManipMotionPlanner::getArmMotionProfile()
{
    return this->p_motionplan->getArmMotionProfile();
}

std::vector<std::vector<std::vector<double>>>
    *MobileManipMotionPlanner::get3DCostMap()
{
    return this->p_motionplan->get3DCostMap();
}

void MobileManipMotionPlanner::executeAtomicOperation()
{
    // TODO - implement MobileManipMotionPlanner::executeAtomicOperation
    throw "Not yet implemented";
}

