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
 *
 * Modified by J. Ricardo Sanchez Ibanez, ricardosan@uma.es
 * 
 * ADE Project (OG10)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include "WaypointNavigation.hpp"
#include <iostream>


namespace waypoint_navigation_lib
{

WaypointNavigation::WaypointNavigation()
{
    mNavigationState = NO_TRAJECTORY;
    // Booleans
    targetSet = false;
    poseSet = false;
    finalPhase = false;

    // Ackermann turn parameters
    minTurnRadius = 0.6;  // (in meters)
    // maxDisplacementAckermannTurn = 0.25; // (meters from straight line to the next point)
    maxDisplacementAckermannTurn = 0.5;  // (meters from straight line to the next point)

    // Alignment parameters
    alignment_deadband = 5.0 / 180.0 * M_PI;
    alignment_saturation = 20.0 / 180.0 * M_PI;
    rotationalVelocity = 10.0 / 180.0 * M_PI;  // [rad/s] ... cca 8.6 deg/s

    defaultTolHeading = 3.0 / 180.0 * M_PI;
    defaultTolPos = 0.075;
    headingErr = 0;
    alignment_P = rotationalVelocity / alignment_saturation;
    alignment_D = 0.75 * alignment_P;

    pd_initialized = false;

    // Velocity
    translationalVelocity = 0.05;  // [m/s]

    // Distances
    corridor = .2;
    lookaheadDistance = .6;
    distanceToNext = new std::vector<double>();
}

NavigationState WaypointNavigation::getNavigationState()
{
    return mNavigationState;
}

void WaypointNavigation::setNavigationState(NavigationState state)
{
    if (mNavigationState != state)
    {
        //LOG_DEBUG_S << "Changing nav. state from " << mNavigationState << " to " << state;
        mNavigationState = state;
        pd_initialized = false;
    }
}

double WaypointNavigation::getLookaheadDistance()
{
    return lookaheadDistance;
}

bool WaypointNavigation::setPose(Pose& pose)
{
    if (std::isnan(pose.position(0)) || std::isnan(pose.position(1)))
    {
        // Position data are not valid, pose will not be set
        //LOG_DEBUG_S << "Position is not valid!";
        return false;
    }
    else
    {
        curPose = pose;
        xr = base::Vector2d(pose.position(0), pose.position(1));
        if (!poseSet && !trajectory.empty())
        {
            // In case the robot is out of the first waypoint,
	    // the state will be OUT_OF_BOUNDARIES
            if ((trajectory.size() > 1)&&(currentSegment == 0))
	    {
                setSegmentWaypoint(w1, currentSegment);
                setSegmentWaypoint(w2, currentSegment + 1);
	    }
	    else
	    {
                w1 << curPose.position(0), curPose.position(1);
                setSegmentWaypoint(w2, currentSegment);
	    }
            setNavigationState(DRIVING);
        }
        poseSet = true;
        return true;
    }
}

void WaypointNavigation::setLookaheadPoint(base::Waypoint& waypoint)
{
    targetSet = true;
    lookaheadPoint = waypoint;
}

// Get the movement command from current pose to the current lookahead point
void WaypointNavigation::getMovementCommand(MotionCommand& mc)
{
    if (!targetSet || !poseSet)
    {
        //LOG_DEBUG_S << "No target or pose specified";
        mc.m_speed_ms = 0;
        mc.m_turnRate_rads = 0;
        return;
    }

    // Vector from current to target position
    Eigen::Vector3d driveVector = lookaheadPoint.position - curPose.position;
    driveVector.z() = 0;  // Do not care about robot's Z displacement, navigation in 2D plane
    double distToTarget = driveVector.norm();

    // Transform Lookahead point to Robot Coordinate Frame
    Eigen::Vector3d lookaheadPointRCF(0, 0, 0);
    lookaheadPointRCF = Eigen::AngleAxisd(-curPose.getYaw(), Eigen::Vector3d::UnitZ())
                        * (lookaheadPoint.position - curPose.position);

    /*if (WAYPOINT_NAVIGATION_DEBUG){
      std::cout << "Robot, WCF:    \t ("  << curPose.position.x()
      << ", " << curPose.position.y() << ") " << std::endl;
      std::cout << "Lookahead, WCF:\t ("  << lookaheadPoint.position.x()
      << ", " << lookaheadPoint.position.y() << ") " << std::endl;
      std::cout << "Lookahead, RCF:\t ("  << lookaheadPointRCF.x()
      << ", " << lookaheadPointRCF.y() << ") " << std::endl;
      }*/

    int sign = (lookaheadPointRCF.x() < 0 ? -1 : 1);
    if (fabs(lookaheadPointRCF.y()) <= 0.001)
    {  // Straight line motion if dY below [1mm]
        // std::cout << "Straight line case" << std::endl;
        mc.m_speed_ms = sign * translationalVelocity;
        mc.m_turnRate_rads = 0;
    }
    else
    {  // Turn Required
        double turn_radius = (lookaheadPointRCF.x() * lookaheadPointRCF.x()
                              + lookaheadPointRCF.y() * lookaheadPointRCF.y())
                             / (2 * lookaheadPointRCF.y());

        // Heading of the robot at the end of the turn to the lookahead point
        double theta = atan2(lookaheadPointRCF.y(), lookaheadPointRCF.x()) * 2;
        // In Robot Coordinate Frame ... -360 to +360 deg

        wrapAngle(theta);  // maps -360 to +360 deg --> 0 to 180, -180 to 180, -180 to 0
        // Angle of the turn rotation,
        // heading difference from the current heading to the lookahead direction
        targetHeading = theta / 2;  // In range of -90 to +90

        if (!backwardPerimtted && sign < 0)
        {
            // Robot not allowed to go backwards
            targetHeading += M_PI;
            wrapAngle(targetHeading);
        }
        double distFromLine = distToTarget * (1 - cos(targetHeading));

        // SELECT THE MORE APPROPRIATE MOTION
        if (fabs(turn_radius) <= minTurnRadius
            || fabs(distFromLine) >= maxDisplacementAckermannTurn)
        {
            // Point turn required
            // Author's comment: Steering command saturation could be used instead:
            mc.m_speed_ms = 0;
            mc.m_turnRate_rads = targetHeading > 0 ? rotationalVelocity : -rotationalVelocity;
        }
        else
        {  // ACKERMANN TURN CASE
            mc.m_speed_ms = sign * translationalVelocity;
            mc.m_turnRate_rads = mc.m_speed_ms / turn_radius;
        }
        // Transform target heading to WCF
        targetHeading += curPose.getYaw();
        wrapAngle(targetHeading);
    }
}

void WaypointNavigation::setTrajectory(std::vector<base::Waypoint*>& t)
{
    // Delete old trajectory
    for (std::vector<base::Waypoint*>::iterator it = trajectory.begin(); it != trajectory.end();
         it++)
    {
        delete *it;
    }
    trajectory.clear();
    trajectory = t;
    targetSet = false;

    if (!trajectory.empty())
    {
        // Add current pose at the begining
        if (poseSet)
        {
            initilalizeCurrentSegment();
        }
        else
        {
            currentSegment = 0;
        }

        // Precalculate distances
        distanceToNext->resize(trajectory.size() - 1);
        base::Vector3d wp;
        for (size_t i = 0; i < distanceToNext->size(); i++)
        {
            wp = trajectory.at(i + 1)->position;  // w2 - w1
            wp -= trajectory.at(i)->position;
            wp.z() = 0;
            distanceToNext->at(i) = wp.norm();
        }

        // Check goal tolerances
        if (trajectory.back()->tol_position <= 0 || trajectory.back()->tol_heading <= 0)
        {
            trajectory.back()->tol_position = defaultTolPos;
            trajectory.back()->tol_heading = defaultTolHeading;
        }

        // Case of 0deg target heading: use the last segment heading instead
        if (trajectory.back()->heading == 0.0)
        {
            if (trajectory.size() > 1)
            {
                trajectory.back()->heading = atan2(wp.y(), wp.x());
            }
            else
            {  // Quickfix: Final heading from single wp trajectory.
                wp = trajectory.back()->position - curPose.position;
                trajectory.back()->heading = atan2(wp.y(), wp.x());
            }
        }
        if (poseSet)
        {
            setNavigationState(DRIVING);
        }
        else
        {
            setNavigationState(NO_POSE);
        }
    }
    else
    {
        distanceToNext = new std::vector<double>();
        setNavigationState(NO_TRAJECTORY);
    }
}

// MAIN PATH FOLLOWING UPDATE FUNCTION CALLED FROM THE COMPONENT UPDATE HOOK
bool WaypointNavigation::update(MotionCommand& mc)
{
    // 1) Update the current SEGMENT
    // Select the segment such that robot is not within the immediate reach of the 2nd Waypoint
    double distToNext = (w2 - xr).norm();
    while (distToNext <= corridor)
    {
        if (currentSegment < trajectory.size() - 1)
        {
            setSegmentWaypoint(w1, currentSegment);
            setSegmentWaypoint(w2, currentSegment + 1);
            currentSegment++;
            distToNext = (w2 - xr).norm();
        }
        else
        {
            // LAST SEGMENT HANDLING, vicinity of final waypoint
            // Executing this code means the robot is within the corridor circle of final waypoint
            double headingErr, posErr;
            distToNext = (w2 - xr).norm();  // Distance to Final
            //    error = Final heading - current heading; both angles in range (-pi; pi)
            headingErr = (trajectory.back()->heading) - curPose.getYaw();
            wrapAngle(headingErr);

            // Driving/aligning based on reaching the tolerance (the final inner radius)
            if (finalPhase || distToNext <= trajectory.back()->tol_position)
            {
                // Position for alignment was reached
                finalPhase = true;
                if (fabs(headingErr) < trajectory.back()->tol_heading
                    || (mNavigationState == TARGET_REACHED
                        && fabs(headingErr) < 2 * trajectory.back()->tol_heading))
                {
                    setNavigationState(TARGET_REACHED);
                    // std::cout << "Go to target reached" << std::endl;
                }
                else
                {
                    setNavigationState(ALIGNING);  // Align to target heading
                    targetHeading = trajectory.back()->heading;
                    wrapAngle(targetHeading);
                    // std::cout << "Go to alignment with " << targetHeading*180.0/M_PI <<
                    // std::endl;
                }
            }
            else
            {
                // finalPhase = false;
                Eigen::Vector3d xf;
                xf = trajectory.back()->position;
                targetHeading = atan2(xf.y() - xr.y(), xf.x() - xr.x());  // Heading to target
                posErr = distToNext * sin(targetHeading - curPose.getYaw());
                if (fabs(posErr) > trajectory.back()->tol_position)
                {
                    // This assumes straight line motion, not Ackermann, but approx valid
                    setNavigationState(ALIGNING);  // with targetHeading = heading to target
                }
                else
                {
                    setNavigationState(DRIVING);
                }
            }
            /*LOG_DEBUG_S << "Final phase:" << (finalPhase ? "T" : "F") << std::endl
                        << "\t Dist remng.   \t" << distToNext << " m" << std::endl
                        << "\t Heading error \t" << headingErr * 180 / M_PI << "/"
                        << trajectory.back()->tol_heading * 180 / M_PI << " deg" << std::endl
                        << "\t Pos. err. est.\t" << posErr << "/" << trajectory.back()->tol_position
                        << "m ";*/
            break;
        }
    }
    finalPhase &= (currentSegment == trajectory.size() - 1);

    // 2) Get intersection point with the Path (should also return distance from the segment)
    base::Vector2d xi = getClosestPointOnPath();

    // 3) Calculate the distance from the nominal trajectory
    distanceToPath = (xr - xi).norm();
    NavigationState currentState = getNavigationState();

    /* -------------------------------------------
     * STATEMACHINE FOR EXECUTION OF DIFFERENT PATH FOLLOWING MODES
     ------------------------------------------- */
    switch (currentState)
    {
        case (NavigationState)DRIVING:
        {
            // 0) OUT OF BOUNDARIES CHECK
            if (distanceToPath >= corridor)
            {
                setNavigationState(OUT_OF_BOUNDARIES);
                mc.m_speed_ms = 0;
                mc.m_turnRate_rads = 0;
                return false;
            }
            double distance;  // Available distance in the current segment.
            distance = distanceToPath + distToNext;

            double k_epsilon = 1.0;  // Is not changed in this implementation.

            // Adaptive lookahead distance calculation
            double lookaheadDistanceAdaptive;
            lookaheadDistanceAdaptive = lookaheadDistance - k_epsilon * distanceToPath;

            // i) Get the look ahead point segment
            base::Vector2d lineVector, lookaheadPoint2D;

            //LOG_DEBUG_S << "Lookahead Distance: " << lookaheadDistanceAdaptive << "/" << distance;
            if (distance > lookaheadDistanceAdaptive)  // Lookahead within same seg.
            {
                // ii) Get the look ahead point in the current segment
                lineVector = w2 - w1;
                lineVector.normalize();
                lookaheadPoint2D = xi + lineVector * lookaheadDistanceAdaptive;
            }
            else
            {  // Find the right segment for the lookahead point
                size_t lookaheadSegment;
                lookaheadSegment = currentSegment;
                while (lookaheadSegment < distanceToNext->size()
                       && distance <= lookaheadDistanceAdaptive)
                {
                    distance += distanceToNext->at(lookaheadSegment);
                    lookaheadSegment++;
                }

                // ii) Get the look ahead point in the lookahead segment
                setSegmentWaypoint(l1, lookaheadSegment - 1);
                setSegmentWaypoint(l2, lookaheadSegment);
                lineVector = l2 - l1;
                lineVector.normalize();
                lookaheadPoint2D = l2 - lineVector * (distance - lookaheadDistance);
            }
            // Set lookahead point: Vector2d -> Waypoint
            lookaheadPoint.position << lookaheadPoint2D(0), lookaheadPoint2D(1), 0;
            lookaheadPoint.heading = atan2(lineVector(1), lineVector(0));
            lookaheadPoint.tol_position = 0.1;
            targetSet = true;

            // iii) Get motion command to the lookahead point
            getMovementCommand(mc);
            if (fabs(mc.m_speed_ms) < 1e-6)
            {
                setNavigationState(ALIGNING);
                // Use the Angle To Target as Target heading
            }  // Else drive using the ackermann command values
            break;
        }  // --- end of DRIVING ---
        case ALIGNING:
        {
            // OUT OF BOUNDARIES CHECK
            if (distanceToPath >= corridor)
            {
                setNavigationState(OUT_OF_BOUNDARIES);
                mc.m_speed_ms = 0;
                mc.m_turnRate_rads = 0;
                return false;
            }

            mc.m_speed_ms = 0;  // Ensure
            base::Time t1 = base::Time::now();

            double disalignmentTolerance, headingErrPrev, headingErrDiff, alignment_dt;
            disalignmentTolerance =
                finalPhase ? trajectory.back()->tol_heading : alignment_deadband;

            headingErrPrev = headingErr;
            headingErr = targetHeading - curPose.getYaw();

            wrapAngle(headingErr);
            saturation(headingErr, alignment_saturation);

            //LOG_DEBUG_S << "Target headin " << targetHeading * 180.0 / M_PI << "deg";
            //LOG_DEBUG_S << "Heading error " << headingErr * 180.0 / M_PI << " deg";

            headingErrDiff = headingErr - headingErrPrev;

            if (pd_initialized)
            {
                alignment_dt = (t1 - tprev).toMilliseconds() / 1000.0;
		if (alignment_dt == 0)
		{
                    headingErrDiff = 0;
		}
		else
		{
                    headingErrDiff /= alignment_dt;
		}
                saturation(headingErrDiff, 10.0 / 180.0 * M_PI);
                //LOG_DEBUG_S << "d(heading err)/dt = " << headingErrDiff * 180 / M_PI << "deg/"
                            //<< alignment_dt << "sec = " << headingErrDiff;
            }
            else
            {
                pd_initialized = true;
                headingErrDiff = 0;
                //LOG_DEBUG_S << "PD re-initialized";
            }

            if (fabs(headingErr) < disalignmentTolerance)
            {
                mc.m_turnRate_rads = 0;
                setNavigationState(DRIVING);
            }
            else
            {
                mc.m_turnRate_rads = alignment_P * headingErr + alignment_D * headingErrDiff;
            }

            saturation(mc.m_turnRate_rads, rotationalVelocity);
            tprev = t1;

            /*LOG_DEBUG_S << "Aligning:\t " << 180.0 / M_PI * curPose.getYaw() << " to "
                        << 180.0 / M_PI * targetHeading << "+-"
                        << 180.0 / M_PI * disalignmentTolerance
                        << " deg,\nrv = " << 180.0 / M_PI * alignment_P * headingErr << " + "
                        << 180.0 / M_PI * alignment_D * headingErrDiff << " ~ "
                        << 180.0 / M_PI * mc.rotation << "deg/s ";*/

            //LOG_DEBUG_S << "e-: " << 180.0 / M_PI * headingErrPrev << "deg.";
           // LOG_DEBUG_S << "e:  " << 180.0 / M_PI * headingErr << "deg.";
            //LOG_DEBUG_S << "de: " << 180.0 / M_PI * headingErrDiff << "deg.";

            break;
        }  // --- end of ALIGNING ---
        case OUT_OF_BOUNDARIES:
        {
            mc.m_speed_ms = 0;
            mc.m_turnRate_rads = 0;
            double progress, distAlong, distPerpendicular;
            // Try the current segment first
            if (currentSegment > 0)
            {
                getProgressOnSegment(currentSegment, progress, distAlong, distPerpendicular);
                if (progress > 0 && isInsideBoundaries(distAlong, distPerpendicular))
                {
                    setNavigationState(DRIVING);
                    break;
                }
            }
            // Try the previous segment
            if (currentSegment > 1)
            {
                getProgressOnSegment(currentSegment - 1, progress, distAlong, distPerpendicular);
                if (isInsideBoundaries(distAlong, distPerpendicular))
                {
                    currentSegment--;
                    setSegmentWaypoint(w1, currentSegment - 1);
                    setSegmentWaypoint(w2, currentSegment);
                    setNavigationState(DRIVING);
                    break;
                }
            }
            // try matching the robot position with the NEXT segment
            if (currentSegment < trajectory.size() - 1)
            {
                getProgressOnSegment(currentSegment + 1, progress, distAlong, distPerpendicular);
                if (isInsideBoundaries(distAlong, distPerpendicular))
                {
                    currentSegment++;
                    setSegmentWaypoint(w1, currentSegment - 1);
                    setSegmentWaypoint(w2, currentSegment);
                    setNavigationState(DRIVING);
                    break;
                }
            }
            break;
        }
        case TARGET_REACHED: /*LOG_DEBUG_S << "Waypoint Navigation: Target Reached.";*/ break;
        case NO_TRAJECTORY: /*LOG_DEBUG_S << "Invalid trajectory.";*/ break;
        default: /*LOG_DEBUG_S << "Default case.";*/ break;
    }
    return true;
}

base::Vector2d WaypointNavigation::getClosestPointOnPath()
{
    // Solving for parameter k such that closestPoint = w1 + k*segVector;
    // 1) segVector = (w2-w1) Vector of the segment line
    // Using the equations
    //    a) w1 + segVector*k = xi
    //    b) xr + perpendicular(segVector)*j = xi
    // This gives [xr - w1] = [perpendicular(segVector), segVector][j; k]
    // 2) Calculate k using the inverse matrix
    // 3) Calculate the point of intersection using a)

    // 1)
    base::Vector2d segVector = w2 - w1;
    segVector.normalize();
    // 2)
    base::Matrix2d inverseL;
    inverseL << segVector(1), -segVector(0), segVector(0), segVector(1);
    base::Vector2d xi = inverseL * (xr - w1);
    // 3)
    xi = w1 + xi(1) * segVector;
    return xi;
}

bool WaypointNavigation::setSegmentWaypoint(base::Vector2d& waypoint, int indexSegment)
{
    if (indexSegment > ((int)trajectory.size() - 1) || indexSegment < 0) return false;
    waypoint << trajectory.at(indexSegment)->position(0), trajectory.at(indexSegment)->position(1);
    return true;
}

const base::Waypoint* WaypointNavigation::getLookaheadPoint() { return &lookaheadPoint; }

bool WaypointNavigation::configure(double minR,
                                   double tv,
                                   double rv,
                                   double cr,
                                   double lad,
                                   bool backwards)
{
    std::string reverse_allowed = backwards ? "Permitted" : "Forbidden";

    /*LOG_DEBUG_S << "------------------------------------\n"
                << "Received Path Tracker config values:\n"
                << "Min. turn radius:\t" << minR << " m.\n"
                << "Translat. vel.:\t\t" << tv << " m/s.\n"
                << "Rotation. vel:\t\t" << rv << " rad/s.\n"
                << "Clearance:\t\t" << cr << " m.\n"
                << "Lookahead dist.\t\t" << lad << " m.\n"
                << "Reverse:\t\t" << reverse_allowed << std::endl
                << "------------------------------------";*/

    // All config. parameters must be possitive
    if (minR > 0 && tv > 0 && rv > 0 && cr > 0 && lad > 0)
    {
        minTurnRadius = minR;
        translationalVelocity = tv;
        rotationalVelocity = rv;
        corridor = cr;
        lookaheadDistance = lad;
        backwardPerimtted = backwards;
        // std::cout << "Config successful, " << getLookaheadDistance() << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

bool WaypointNavigation::getProgressOnSegment(int segmentNumber,
                                              double& progress,
                                              double& distAlong,
                                              double& distPerpend)
{
    // Solving for parameter k such that closestPoint = w1 + k*segVector;
    // 1) segVector = (w2-w1) Vector of the segment line
    // Using the equations
    //    a) w1 + segVector*k = xi
    //    b) xr + perpendicular(segVector)*j = xi
    // This gives [xr - w1] = [perpendicular(segVector), segVector][j; k]
    // 2) Calculate k using the inverse matrix
    // 3) Calculate the point of intersection using a)
    // k is also the progress along that segment

    // Using wStart and wEnd instead of w1, w2
    // 1)
    base::Vector2d segVector, wStart, wEnd;
    setSegmentWaypoint(wStart, segmentNumber - 1);
    setSegmentWaypoint(wEnd, segmentNumber);
    segVector = wEnd - wStart;

    // 2)
    double determinant = segVector.dot(segVector);
    base::Matrix2d inverseL;
    inverseL << segVector(1), -segVector(0), segVector(0), segVector(1);
    inverseL /= determinant;

    base::Vector2d xi = inverseL * (xr - wStart);

    // 3)
    progress = xi(1);
    xi = wStart + progress * segVector;
    distPerpend = (xr - xi).norm();

    if (progress >= 1)
    {
        distAlong = (progress - 1) * segVector.norm();
    }
    else if (progress <= 0)
    {
        distAlong = progress * segVector.norm();
    }
    else
    {
        distAlong = 0;
    }
    /* // DEBUG OUTPUTS
       std::cout << "seg vector = (" << segVector(0) <<
       ", " << segVector(1) << ")" << std::endl;
       std::cout << "xr = (" << xr(0) <<
       ", " << xr(1) << ")" << std::endl;
       std::cout << "wStart = (" << wStart(0) <<
       ", " << wStart(1) << ")" << std::endl;
       std::cout << "Determinant: " << determinant << std::endl;
       std::cout << "solution = (" << xi(0) <<
       ", " << xi(1)? << ")" << std::endl;
       std::cout << inverseL << std::endl;
       */
    return true;
}

void WaypointNavigation::setCurrentSegment(int segmentNumber)
{
    if (segmentNumber < 0)
    {
        //LOG_DEBUG_S << "Attemp to set invalid segment number";
    }
    if (segmentNumber < 1)
    {
        w1 << curPose.position(0), curPose.position(1);
        setSegmentWaypoint(w2, segmentNumber);
    }
    else
    {
        setSegmentWaypoint(w1, segmentNumber - 1);
        setSegmentWaypoint(w2, segmentNumber);
    }
    currentSegment = segmentNumber;
}

void WaypointNavigation::initilalizeCurrentSegment()
{
    /* For each segment (except the zeroth (trivial) one):
     *   a) calculate the hypothetical progress on the segment
     *   b) store the maximum index where the robot was in corridor bounds
     *  Set the segment to the max index found.
     */
    size_t maxIndex = 0;
    double progress, distAlong, distPerpend;
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        getProgressOnSegment(i, progress, distAlong, distPerpend);
        if (progress >= 0 && progress <= 1 && distPerpend < corridor)
        {
            maxIndex = i;
        }
        /*LOG_DEBUG_S << "Segment: " << i << ", progress: " << progress << ", dist from nominal "
                    << distPerpend;*/
    }
    //LOG_DEBUG_S << "Segment set to: " << maxIndex;
    setCurrentSegment(maxIndex);
}

bool WaypointNavigation::isInsideBoundaries(double& distAlong, double& distPerpend)
{
    return fabs(distAlong) < corridor && fabs(distPerpend) < corridor;
}

inline void WaypointNavigation::wrapAngle(double& angle)
{
    if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
}

inline void WaypointNavigation::saturation(double& value, double limit)
{
    if (value > limit)
    {
        value = limit;
    }
    else if (value < -limit)
    {
        value = -limit;
    }
}

bool WaypointNavigation::configurePD(double P, double D, double saturation)
{
    if (P > 0 && D > 0 && saturation > 0)
    {
        alignment_P = P;
        alignment_D = D;
        alignment_saturation = saturation;
        // std::cout << "WaypointNavigation::configurePD: P=" << alignment_P << ", D=" <<
        // alignment_D << ", sat=+-" << alignment_saturation << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

bool WaypointNavigation::configureTol(double tolPos, double tolHeading)
{
    if (tolPos > 0 && tolHeading > 0)
    {
        defaultTolPos = tolPos;
        defaultTolHeading = tolHeading;
        return true;
    }
    else
    {
        return false;
    }
}

int WaypointNavigation::getCurrentSegment() { return currentSegment; }

}  // namespace waypoint_navigation_lib
