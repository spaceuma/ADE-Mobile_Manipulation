#include "MobileManipExecutor.h"
#include "Waypoint.hpp"
#include <gtest/gtest.h>
#include "MotionCommand.h"
#include "Waypoint.hpp"
#include <unistd.h>
#include "readMatrixFile.h"


TEST(MMExecutorTest, trajectorycontrol)
{

    // Reading the DEM 
    double res = 0.1; // meters
    double n_row, n_col;
    std::vector<double> vector_elevationData;
    readMatrixFile("test/unit/data/ColmenarRocks_smaller_10cmDEM.csv", res, n_row, n_col, vector_elevationData);
   
    // Creating the Rover Guidance DEM 
    RoverGuidance_Dem dummyDem;
    double dummyArray[(int)n_row * (int)n_col];
    dummyDem.p_heightData_m = dummyArray;
    for (uint i = 0; i < vector_elevationData.size(); i++)
    {
        dummyDem.p_heightData_m[i] = vector_elevationData[i];
    }
    dummyDem.cols = n_col;
    dummyDem.rows = n_row;
    dummyDem.nodeSize_m = res;

    // Introducing RG-DEM into MMMap
    MobileManipMap dummyMap;
    dummyMap.setRGDem(dummyDem);
    base::Waypoint roverPos, samplePos;

    clock_t begin = clock();

    roverPos.position[0] = 6.5;
    roverPos.position[1] = 6.5;
    roverPos.heading = 0;

    samplePos.position[0] = 4.0;
    samplePos.position[1] = 2.0;
    samplePos.heading = 0;


    MotionPlan dummyPlan;
    dummyPlan.executeRoverBasePathPlanning(&dummyMap, roverPos, samplePos);
    int numWaypoints = dummyPlan.shortenPathForFetching();

    double zRes = 0.08;
    dummyPlan.executeEndEffectorPlanning(&dummyMap, zRes);


    Pose robotPose;
    Waypoint lpoint;

    robotPose.position = Eigen::Vector3d(roverPos.position[0], roverPos.position[1], 0);

    std::cout << "Test Trajectory" << std::endl;
    robotPose.orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(10.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()));


    MobileManipExecutor dummyExecutor(dummyPlan);

    std::cout << "Robot = (" << robotPose.position.x() << "," << robotPose.position.y() << ","
              << robotPose.position.z() << ")"
              << "yaw = " << robotPose.getYaw() * 180 / M_PI << "deg." << std::endl;
    MotionCommand mc;
    double dt = 0.1;
    double yaw;

    while (!dummyExecutor.isFinished())
    {

        mc = dummyExecutor.getRoverCommand(robotPose);
        yaw = robotPose.getYaw();
        Eigen::AngleAxisd toWCF, robotRot;
        toWCF = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        if (fabs(mc.m_speed_ms) < 0.000001)
        {
            // Point turn
            // std::cout << "PT of " << (mc.m_turnRate_rads*dt)*180/M_PI << "deg" << std::endl;
            robotRot = AngleAxisd(mc.m_turnRate_rads * dt, Eigen::Vector3d::UnitZ());
            robotPose.orientation = Eigen::Quaterniond(robotRot) * robotPose.orientation;
        }
        else if (fabs(mc.m_turnRate_rads) < 0.000001)
        {
            // Straight line
            // std::cout << "SL" << std::endl;
            robotPose.position += (mc.m_speed_ms * dt) * (toWCF * Eigen::Vector3d::UnitX());
        }
        else
        {
            // Ackermann
            //  std::cout << "ACK" << std::endl;
            Eigen::Vector3d turnCenter;
            turnCenter << 0.0, mc.m_speed_ms / mc.m_turnRate_rads, 0.0;
            turnCenter = toWCF * (turnCenter) + robotPose.position;
            robotRot = AngleAxisd(mc.m_turnRate_rads * dt, Eigen::Vector3d::UnitZ());
            robotPose.position = robotRot * (robotPose.position - turnCenter) + turnCenter;
            robotPose.orientation = Eigen::Quaterniond(robotRot) * robotPose.orientation;
        }

        std::cout << "Robot = (" << robotPose.position.x() << "," << robotPose.position.y() << ","
                  << robotPose.position.z() << "), "
                  << "yaw = " << robotPose.getYaw() * 180 / M_PI << " deg." << std::endl
                  << std::endl;
        std::cout << "tv = " << mc.m_speed_ms;
        std::cout << ", rv = " << mc.m_turnRate_rads << std::endl << std::endl;

        usleep(10000);
    }

   /* robotPose.position = Eigen::Vector3d(1.5, 0.0, 0);
    pathTracker.setPose(robotPose);
    pathTracker.setNavigationState(OUT_OF_BOUNDARIES);
    // Confuse the segment ;)
    pathTracker.setCurrentSegment(1);
    std::cout << "-------------------------------------" << std::endl
              << "Lost segment test (set to previous by mistake)" << std::endl
              << std::endl;
    for (int i = 0; i < 3; i++)
    {
        pathTracker.update(mc);
    }
    pathTracker.setPose(robotPose);*/
}
