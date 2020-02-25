/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Library for pure-pursuit based path following
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Dec 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#pragma once

#include <Eigen/Geometry>
#include "Waypoint.hpp"
#include "MotionCommand.h"
#include "BasePose.hpp"
#include "Time.hpp"
#include <vector>

using namespace base;
using namespace proxy_library;

namespace waypoint_navigation_lib
{

enum NavigationState
{
    DRIVING = 0,        // 0
    ALIGNING,           // 1
    TARGET_REACHED,     // 2
    OUT_OF_BOUNDARIES,  // 3
    NO_TRAJECTORY,      // 4
    NO_POSE             // 5
};

class WaypointNavigation
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    WaypointNavigation();

    // Set positon and orientation, where to drive to
    void setLookaheadPoint(Waypoint& pose);

    // Set current orientation and position,
    // with a validity check (to avoid NaNs from Vicon)
    // If invalid pose is received, it is discarded and false is returned
    bool setPose(Pose& pose);

    // Sets the trajecory the robot should follow
    // and calculates the distances between consecutive waypoints
    void setTrajectory(std::vector<Waypoint*>& t);

    // Returns the trajectory
    const std::vector<Waypoint*>& getTrajectory() const { return trajectory; }

    NavigationState getNavigationState();
    void setNavigationState(NavigationState state);

    const Waypoint* getLookaheadPoint();

    bool update(MotionCommand& mc);

    bool configure(double minR, double tv, double rv, double cr, double lad, bool backward);

    bool configurePD(double P, double D, double saturation);
    bool configureTol(double TolPos, double TolHeading);

    // Calculates a motion command (Ackermann or Point turn)
    // given the robot pose and DRIVING mode
    void getMovementCommand(MotionCommand& mc);

    bool getProgressOnSegment(int segmentNumber,
                              double& progress,
                              double& distAlong,
                              double& distPerpend);
    double getLookaheadDistance();
    void setCurrentSegment(int segmentNumber);  // TESTING ONLY - TODO Remove
    int getCurrentSegment();

  private:
    NavigationState mNavigationState;
    bool aligning;
    bool targetSet;
    bool poseSet;
    bool newWaypoint;
    bool finalPhase;
    bool backwardPerimtted;

    double minTurnRadius;  //  [m]
    double maxDisplacementAckermannTurn;
    double translationalVelocity;
    double rotationalVelocity;
    double corridor;  // Allowed Distance perpendicular to path segment
    double lookaheadDistance;
    double distanceToPath;
    double targetHeading;

    // Alignment tolerances
    double defaultTolHeading, defaultTolPos;

    // Alignment controller
    double alignment_deadband, alignment_saturation;
    double headingErr, alignment_P, alignment_D;
    bool pd_initialized;
    Time tprev;

    Pose curPose;
    Waypoint targetPose;

    std::vector<double>* distanceToNext;
    std::vector<Waypoint*> trajectory;
    Waypoint lookaheadPoint;
    int currentSegment;

    Vector2d w1, w2, l1, l2, xr;

    // Helper function for setting values of Vector2d with X, Y of a trajectory waypoint
    bool setSegmentWaypoint(Vector2d& waypoint, int indexSegment);

    // Helper function for finding the closes point on the path segment
    // from the current position of the robot
    Vector2d getClosestPointOnPath();

    void initilalizeCurrentSegment();

    bool isInsideBoundaries(double& distAlong, double& distPerpend);
    inline void wrapAngle(double& angle);
    inline void saturation(double& value, double limit);
};
}  // namespace waypoint_navigation_lib
