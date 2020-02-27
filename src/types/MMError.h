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
  UNREACH_GOAL,
  UNCERT_GOAL,
  DEGEN_PATH,
  UNFEASIBLE_PROFILE, //TODO: distinguish between collision and sample out of tunnel
  INCOMPLETE_INPUT,
  NON_RESP_ARM,
  COLLIDING_ARM,
  NON_RESP_ROVER,
  EXCESSIVE_DRIFT,
  UNCERT_HEADING,
  GOAL_TOO_CLOSE, // The goal is too close to the rover
  IMPROPER_CALL // A function called in an improper time
};

#endif
