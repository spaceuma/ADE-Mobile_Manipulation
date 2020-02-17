#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;

MobileManipMotionPlanner::MobileManipMotionPlanner(
    const RoverGuidance_Dem &navCamDEM,
    const Joints &j_present_readings,
    double d_zres_m)
{
    cout << "MMPLANNER: Creating MMMP" << endl;
    this->status = IDLE;
    this->error = NO_ERROR;
    // DEM is introduced into the map class
    this->p_mmmap = new MobileManipMap(navCamDEM);
    // Each class contains a pointer to the previous one
    this->p_motionplan = new MotionPlan(this->p_mmmap, d_zres_m);
    this->p_mmexecutor = new MobileManipExecutor(this->p_motionplan, j_present_readings);
}

bool MobileManipMotionPlanner::generateMotionPlan(
    const base::Waypoint &rover_position,
    const base::Waypoint &sample_position)
{
    if (getStatus() == IDLE)
    {
        unsigned int ui_code = 0;
        // TODO - Since for now there is no computation, the state will go to
        // READY_TO_MOVE
        setStatus(GENERATING_MOTION_PLAN);
        this->p_mmmap->computeFACE(sample_position);
        ui_code = this->p_motionplan->executeRoverBasePathPlanning(
            rover_position, sample_position);
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
        }
        if (!(this->p_motionplan->shortenPathForFetching()))
        {
            setError(GOAL_TOO_CLOSE);
            return false;
        }
        printRoverPathInfo();
        // TODO - Deal with EndEffectorPlanning errors
        this->p_motionplan->executeEndEffectorPlanning();
        setStatus(READY_TO_MOVE);
        return true;
    }
    else
    {
        setError(IMPROPER_CALL);
        return false;
    }
}

void MobileManipMotionPlanner::executeAtomicOperation(
    ArmOperation arm_operation)
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

void MobileManipMotionPlanner::updateLocCamDEM(RoverGuidance_Dem locCamDEM,
                                               Pose rover_position,
                                               Joints arm_joints)
{
    throw "Not yet implemented";
}

bool MobileManipMotionPlanner::updateRoverArmPos(Joints &arm_command,
                                                 MotionCommand &rover_command,
                                                 Pose rover_position,
                                                 Joints arm_joints)
{
    switch (getStatus())
    {
        case EXECUTING_MOTION_PLAN:
            if (this->p_mmexecutor->isRoverFinished())
            {
                setStatus(EXECUTING_ARM_OPERATION);
                return true;
            }
            else
            {
                unsigned int ui_error_code
                    = this->p_mmexecutor->getCoupledCommand(rover_position,
				                            arm_joints,
                                                          rover_command,
							  arm_command);
                switch (ui_error_code)
                {
                    case 0: // Either driving or aligning
                        return true;
                    case 1: // (Rover) Target reached
                        return true;
                    case 2: // Out of boundaries
                        setError(EXCESSIVE_DRIFT);
                        return false;
                    case 3: // Either no trajectory or no pose
                        // TODO - Is this situation even possible to reach??
                        setError(IMPROPER_CALL);
                        return false;
                }
            }
        case EXECUTING_ARM_OPERATION:
            throw "not finished";
        case RETRIEVING_ARM:
            throw "not finished";
        default:
            setError(IMPROPER_CALL);
            return false;
    }
}

void MobileManipMotionPlanner::updateSamplePos(Pose sample)
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
    std::cout << " Current Error Code: ";
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
