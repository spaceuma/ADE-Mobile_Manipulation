enum MM_status
{
  IDLE,
  GENERATING_MOTION_PLAN,
  READY_TO_MOVE,
  EXECUTING_MOTION_PLAN,
  EXECUTING_ARM_OPERATION,
  RETRIEVING_ARM,
  FINISHED,
  ERROR,
  REPLANNING,
  PAUSE
};
