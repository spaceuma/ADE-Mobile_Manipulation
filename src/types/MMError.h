#ifndef __MMERROR__
#define __MMERROR__

enum MMError
{
  NO_ERROR = 0,
  POOR_DEM,
  POOR_CONFIG,
  OOB_ROVER_POS,
  OOB_GOAL_POS,
  OBS_ROVER_POS,
  OBS_GOAL_POS,
  PLAN_WO_SAMPLE,
  UNREACH_GOAL,
  UNCERT_GOAL,
  DEGEN_PATH,
  COLLIDING_PROF,
  DEVIATED_PROF,
  FORB_ARM_POS,
  INCOMPLETE_INPUT,
  NON_RESP_ARM,
  COLLIDING_ARM,
  NON_RESP_ROVER,
  EXCESSIVE_DRIFT,
  UNCERT_HEADING,
  GOAL_TOO_CLOSE, // The goal is too close to the rover
  BAD_DEM_ALLOC,
  UNFEASIBLE_INIT, //Initial Operation cannot be computed
  IMPROPER_CALL // A function called in an improper time
};

#endif
