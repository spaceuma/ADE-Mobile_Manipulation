#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "Waypoint.hpp"
#include "Waypoint.hpp"
#include "mmFileManager.h"
#include <gtest/gtest.h>
#include <unistd.h>

TEST(MMExecutorTest, nominal_working_test)
{

    std::vector<Waypoint> path;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    readPath("test/unit/data/input/MMExecutorTest/path.txt", path);
    readMatrixFile("test/unit/data/input/MMExecutorTest/armMotionProfile.txt",
                   vvd_arm_motion_profile);
    MotionPlan dummyPlan(path, vvd_arm_motion_profile);

    Pose robotPose;
    Waypoint lpoint;

    robotPose.position
        = Eigen::Vector3d(path[0].position[0], path[0].position[1], 0);

    robotPose.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(10.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()));

    MobileManipExecutor dummyExecutor(dummyPlan);

    std::cout << "\033[32m[----------]\033[0m Robot = ("
              << robotPose.position.x() << "," << robotPose.position.y() << ","
              << robotPose.position.z() << ")"
              << "yaw = " << robotPose.getYaw() * 180 / M_PI << "deg."
              << std::endl;
    MotionCommand mc;
    double dt = 0.1;
    double yaw;

    std::vector<JointState> vj_current_jointstates;
    vj_current_jointstates.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        vj_current_jointstates[i].m_position = 0.0;
        vj_current_jointstates[i].m_speed = 0.0;
    }

    Joints j_current_joints(0, vj_current_jointstates);
    Joints j_next_joints(0, vj_current_jointstates);

    while (!dummyExecutor.isFinished())
    {

        mc = dummyExecutor.getRoverCommand(robotPose);
        yaw = robotPose.getYaw();
        Eigen::AngleAxisd toWCF, robotRot;
        toWCF = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        dummyExecutor.getArmCommand(j_next_joints);
        for (uint i = 0; i < 6; i++)
        {
            j_current_joints.m_jointStates[i].m_position
                = j_next_joints.m_jointStates[i].m_position;
        }
        if (fabs(mc.m_speed_ms) < 0.000001)
        {
            // Point turn
            std::cout << "PT of " << (mc.m_turnRate_rads * dt) * 180 / M_PI
                      << "deg" << std::endl;
            robotRot
                = AngleAxisd(mc.m_turnRate_rads * dt, Eigen::Vector3d::UnitZ());
            robotPose.orientation
                = Eigen::Quaterniond(robotRot) * robotPose.orientation;
        }
        else if (fabs(mc.m_turnRate_rads) < 0.000001)
        {
            // Straight line
            // std::cout << "SL" << std::endl;
            robotPose.position
                += (mc.m_speed_ms * dt) * (toWCF * Eigen::Vector3d::UnitX());
        }
        else
        {
            // Ackermann
            //  std::cout << "ACK" << std::endl;
            Eigen::Vector3d turnCenter;
            turnCenter << 0.0, mc.m_speed_ms / mc.m_turnRate_rads, 0.0;
            turnCenter = toWCF * (turnCenter) + robotPose.position;
            robotRot
                = AngleAxisd(mc.m_turnRate_rads * dt, Eigen::Vector3d::UnitZ());
            robotPose.position
                = robotRot * (robotPose.position - turnCenter) + turnCenter;
            robotPose.orientation
                = Eigen::Quaterniond(robotRot) * robotPose.orientation;
        }

        std::cout << "Robot = (" << robotPose.position.x() << ","
                  << robotPose.position.y() << "," << robotPose.position.z()
                  << "), "
                  << "yaw = " << robotPose.getYaw() * 180 / M_PI << " deg."
                  << std::endl
                  << std::endl;
        std::cout << "tv = " << mc.m_speed_ms;
        std::cout << ", rv = " << mc.m_turnRate_rads << std::endl;
        for (uint i = 0; i < 6; i++)
        {
            std::cout << " Arm Joint " << i << " position is "
                      << j_current_joints.m_jointStates[i].m_position
                      << std::endl;
        }
        std::cout << std::endl;
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
