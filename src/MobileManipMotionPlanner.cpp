#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;

MobileManipMotionPlanner::MobileManipMotionPlanner(
    const RoverGuidance_Dem &navCamDEM,
    const Joints &j_present_readings,
    double d_zres_m,
    string s_urdf_path_m)
{
    cout << "MMPLANNER: Creating MMMP" << endl;
    this->status = IDLE;
    this->error = NO_ERROR;
    // DEM is introduced into the map class
    this->p_mmmap = new MobileManipMap(navCamDEM);
    // Each class contains a pointer to the previous one
    this->p_motionplan = new MotionPlan(this->p_mmmap, d_zres_m, s_urdf_path_m);
    this->p_mmexecutor
        = new MobileManipExecutor(this->p_motionplan, j_present_readings);
}

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

    if (getStatus() == IDLE)
    {
        unsigned int ui_code = 0;
        // TODO - Since for now there is no computation, the state will go to
        // READY_TO_MOVE
        setStatus(GENERATING_MOTION_PLAN);
        this->p_mmmap->computeFACE(sample_position);
        ui_code = this->p_motionplan->executeRoverBasePathPlanning(
            this->w_current_rover_position, sample_position);
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
                setError(OOB_GOAL_POS);
                return false;
            case 4:
                setError(OBS_GOAL_POS);
                return false;
            case 5:
                setError(GOAL_TOO_CLOSE);
                return false;
            case 6:
		setError(DEGEN_PATH);
		return false;
        }
        if (!(this->p_motionplan->shortenPathForFetching()))
        {
            setError(GOAL_TOO_CLOSE);
            return false;
        }
	ui_code = this->p_motionplan->executeEndEffectorPlanning();
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

void MobileManipMotionPlanner::executeAtomicOperation()
{
    // TODO - implement MobileManipMotionPlanner::executeAtomicOperation
    throw "Not yet implemented";
}

void MobileManipMotionPlanner::start()
{
    if (getStatus() == READY_TO_MOVE)
    {
        setStatus(EXECUTING_MOTION_PLAN);
    }
    else
    {
        setError(IMPROPER_CALL);
    }
}

void MobileManipMotionPlanner::abort()
{
    switch (getStatus())
    {
        case READY_TO_MOVE:
            setStatus(IDLE);
            break;
        case EXECUTING_MOTION_PLAN:
            setStatus(RETRIEVING_ARM);
            break;
        case EXECUTING_ARM_OPERATION:
            setStatus(RETRIEVING_ARM);
            break;
        case PAUSE:
            setStatus(RETRIEVING_ARM);
            break;
        default:
            setError(IMPROPER_CALL);
            break;
    }
}

void MobileManipMotionPlanner::pause(Joints &arm_command,
                                     MotionCommand &rover_command)
{
    if ((getStatus() == EXECUTING_MOTION_PLAN)
        || (getStatus() == EXECUTING_ARM_OPERATION)
        || (getStatus() == RETRIEVING_ARM))
    {
        setStatus(PAUSE);
        for (uint i = 0; i < 6; i++)
        {
            arm_command.m_jointStates[i].m_position = 0.0;
            arm_command.m_jointStates[i].m_speed = 0.0;
        }
        rover_command.m_speed_ms = 0.0;
        rover_command.m_turnRate_rads = 0.0;
    }
    else
    {
        setError(IMPROPER_CALL);
    }
}

void MobileManipMotionPlanner::resumeOperation()
{
    if (getStatus() == PAUSE)
    {
        setStatus(this->priorStatus);
    }
    else
    {
        setError(IMPROPER_CALL);
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
    switch (getStatus())
    {
        case EXECUTING_MOTION_PLAN:
        {
            base::Pose basepose;
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
        case EXECUTING_ARM_OPERATION:
            std::cout << "Status is Executing Arm Operation" << std::endl;
            return false;
        case RETRIEVING_ARM:
            throw "not finished";
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

std::vector<std::vector<double>> *MobileManipMotionPlanner::getEndEffectorPath()
{
    return this->p_motionplan->getEndEffectorPath();
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
