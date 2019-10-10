

// THis is the enumerated with the states

enum mobility_status = { IDLE, GENERATING_MOTION_PLAN, READy_TO_MOVE, EXECUTING_MOTION_PLAN, SAMPLE_HANDLING, RETRIEVING_ARM, FINISHED };

mobility_status mob_status; /* variable */
mob_interface  mobility
Position  RoverPosition;
DEM       navcamDEM;
DEM       loccamDEM;

/**
 * Description: function to move the rover and arm to the pick or drop a given sample
 * We assume that we have a pointer to the Rover (that is not part of mobility) and a pointer to mobility
 * (a singleton class that has your functions)
 */
void harness_main_operation( Pose Sample_position )
{
// TODO #1: Change 1 in your interface. UpdateNavCanDEM signature. We assume the DEM as argument */
mobility->updateNavCamDEM(navcamDEM);
RoverPosition = rover->getRoverPosition();
if (mobilility->getStatus()==IDLE)
	{
	/** This will trigger a transition to GENERATING_MOTION_PLAN */
	/** TODO #2: generateMotionPLan signature, we expect RoverPosition and sample position, arm_position ?) */
	mobility->generateMotionPlan(RoverCurrentPosition, sample_position, arm_position );
	}

/** Waiting till the plan is generated */
while ( (mobilility->getStatus()==GENERATING_MOTION_PLAN) && (!timeout) )

/** If we are in ready to move, we shall start */
if (mobilility->getStatus()==READY_TO_MOVE)
	{
	previousPosition = roverPosition; /** This is to compute the distance travelled, see below */
	mobility->start();  /* This will trigger a transition to EXECUTING_MOTION_PLAN - hopefully */
	}
else
	{
	if (timeout)
		message ("timeout while planning in c mobility");
	else
		{
		message ("unexpected mov. status %d while planning", mobility->getStatus());
		}
	}

}

/**
 * Description: 10 Hz task to handle nominal transitions
 */
void harness_cmobility_10HzTask()
{
mobility_status mob_stat = 	mobility->getStatus();
switch (mob_stat)
	{
	case  EXECUTING_MOTION_PLAN:
		//
		// TODO #3: Signature for UpdateRoverArmPos. note that we need to return the arm and rover commands, since
		// it will be the harness the responsible for sending them to the SherpaTT API
		// We assume you need rover and arm positions when being called. Let us know otherwise
		//
		mobility->UpdateRoverArmPos(&arm_commands,&rover_commands,rover_position,arm_joints);
		rover->SendRoverCommandsViaSherpaAPI(rover_commands);
		rover->SendArmCommandsViaSherpaAPI(arm_commands);
		// UPdate the status
		mob_stat = mobility->getStatus();
		break;
	case  SAMPLE_HANDLING:
		// Same TODO #3
		mobility->UpdateRoverArmPos(&arm_commands,&rover_commands,rover_position,arm_joints);
		rover->SendArmCommandsViaSherpaAPI(arm_commands);
		// UPdate the status
		mob_stat = mobility->getStatus();
	case PAUSE:
		// Just get the status
		mob_stat = mobility->getStatus();
		// Do nothing just make sure the status has not changed
		break;
	case RETRIEVING_ARM;
		// Same as before TODO #3. This time only arm commands are sent
		mobility->UpdateRoverArmPos(&arm_commands,&rover_commands,rover_position,arm_joints);
		rover->SendArmCommandsViaSherpaAPI(arm_commands);
		mob_stat = mobility->getStatus();
	case ERROR:
		// We get into this state if something goes wrong;
		// TODO: Change #4: Better to have a method to get the error msg
		message("mobility error:%s", mobility->getErrorCode());
		// QUESTION #1: Resume error does not need any extra argument, we assume
		mobility->resumeError();
		// get status again
		mob_stat  = mobility->getStatus();
		break;
	case FINISHED:
		// QUESTION #2 ONce we reached finished, we can immediately call ack in the next time slice
		mobility->ack();
		mob_stat  = mobility->getStatus();
		break;
	}

}

int traversedDistance;
RoverPosition previousPosition;
static int DEM_THRESHOLD = 0.5;

/**
 * Description: routine being called at 10Hz for checking whether it is needed a DEM handling. Returns true
 * if the rover has been asked to stop
 */
bool harness_dem_handling_10Hz()
{
	bool buildingDEM = false;
	mob_stat = mobility->getStatus();

	switch (mob_stat)
		{
		case EXECUTING_MOTION_PLAN:
			RoverPosition = this->getRoverPosition();
			if (distance(previousPosition,RoverPosition)>DEM_THRESHOLD)
				{
				// Stop mobility
				// Pause needs to know the state from which it was called
				mobility->pause(EXECUTING_MOTION_PLAN);
				rover->StopAndGenerateDEM(); // Tell the rover to stop and generate the DEM again. The stop may or not
		   	   	   	   	   	   	   	   // be needed depending on whether we need to PAN or not....(TBD)
				previousPosition = RoverPosition; // Reset rover position
				buildingDEM = true;
				}
			break;
		case PAUSE: /** Wait until DEM available. When it is, resumeOperation */
			if (rover->DEMAvailable())
				{
				rover->getLocDem(&loccanDEM);
				/** TODO #5: We think the easiest way is to update the local DEM before we are switching from
				 * PAUSE to EXECUTING MoTION PLAN, because it is the *only* time in which it is needed. Note that we
				 * use PAUSE as a state in which we get the new DEM, and this could take time.
				 */
				mobility->updateLocCamDEM(locDEM);
				/**
				 * TODO #6: ResumeOperation needs to know the status from which it came in order to go back
				 * to the same state
				 */
				mobility->resumeOperation(EXECUTING_MOTION_PLAN)
				}
			else
				{
				buildingDEM = true;
				}
			break;
		default:
			break;
		}

	// If we are stopping the rover, that means we shoud wait until rover->DEMAvailable is provided in a next
	// time slice
return (buildingDEM);
}

/**
 * Description: the DEM
 * Strategy:
 *
 * harness_cmobility_10HzTask is the nominal behaviour, but when we have to build the DEM the status of mobility
 * is sent to PAUSE
 * harness_dem_handling_10Hz is aimed to detect if we have traversed more than x meters, and so we need to PAUSE
 * to build a DEM
 *
 * We first call harness_dem_handling_10Hz to see whether a new DEM has to be generated.
 *
 * If so, we don't call harness_cmobility_10HzTask for this time slice of 100 msec, in which we are in PAUSE
 * after some 100 millisecs slices, we will get the DEM, and the harness_cmobility_10HzTask will continue.
 *
 *
 */
void mobility_harness_10hzTask()
{
	bool buildingDEM = harness_dem_handling_10Hz();
	if (!(buildingDEM))
		{
		harness_cmobility_10HzTask();
		}
	return;
}



