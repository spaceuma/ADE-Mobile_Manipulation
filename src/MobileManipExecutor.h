#include "MotionCommand.h"
#include "MotionPlan.h"
#include "WaypointNavigation.hpp"
#include "coupledControl.hpp"

using namespace waypoint_navigation_lib;
using namespace coupled_control;

class MobileManipExecutor
{

private:
    MotionPlan *p_motion_plan;
    double corridorWidth;
    WaypointNavigation waypoint_navigation;
    std::vector<base::Waypoint *> vpw_path;
    MotionCommand motion_command;
    coupledControl coupled_control;
    // Correspondence between joint reference samples and base waypoints
    // i.e. 0 indicates that joint reference corresponds to the first base
    // waypoint
    int i_current_segment;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    std::vector<double> vd_next_arm_config;

public:
    MobileManipExecutor();
    MobileManipExecutor(MotionPlan &currentMotionPlan);

    void updateMotionPlan(MotionPlan &newMotionPlan);

    bool isRoverWithinCorridor(Pose rover_pose);

    bool isArmColliding();

    bool isFinished();

    MotionCommand getRoverCommand(Pose rover_pose);

    void getArmCommand(Joints &j_next_arm_command);
};
