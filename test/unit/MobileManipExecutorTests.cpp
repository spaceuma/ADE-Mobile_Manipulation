#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "Waypoint.hpp"
#include "Waypoint.hpp"
#include "mmFileManager.h"
#include <gtest/gtest.h>
#include <unistd.h>

TEST(MMExecutorTest, nominal_working_test)
{

    std::vector<Waypoint> vw_path;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    readPath("test/unit/data/input/MMExecutorTest/path.txt", vw_path);
    readMatrixFile("test/unit/data/input/MMExecutorTest/armMotionProfile.txt",
                   vvd_arm_motion_profile);
    MotionPlan dummyPlan(vw_path, vvd_arm_motion_profile);

    Pose robotPose, pose_robot_sim;
    Waypoint lpoint;

    robotPose.position
        = Eigen::Vector3d(vw_path[0].position[0], vw_path[0].position[1], 0);

    robotPose.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(10.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()));

    MobileManipExecutor dummyExecutor(dummyPlan);

    MotionCommand mc;
    double dt = 0.1, yaw, d_pos_error_x = 0, d_pos_error_y = 0;
    unsigned int ui_error_code;

    std::vector<JointState> vj_current_jointstates;
    vj_current_jointstates.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        vj_current_jointstates[i].m_position = 0.0;
        vj_current_jointstates[i].m_speed = 0.0;
    }

    Joints j_current_joints(0, vj_current_jointstates);
    Joints j_next_joints(0, vj_current_jointstates);

    std::ofstream robotPoseFile, robotSimPoseFile;
    robotPoseFile.open("test/unit/data/results/MMExecutorTest/roverRealPos.txt");
    robotSimPoseFile.open("test/unit/data/results/MMExecutorTest/roverEstimatedPos.txt");

    while (!dummyExecutor.isFinished())
    {
        d_pos_error_x = (double)(rand() % 500 - 250) * 0.0001;
        d_pos_error_y = (double)(rand() % 500 - 250) * 0.0001;

        pose_robot_sim.position = robotPose.position;
	pose_robot_sim.position.x() += d_pos_error_x;
	pose_robot_sim.position.y() += d_pos_error_y;
        pose_robot_sim.orientation = robotPose.orientation;
        ui_error_code = dummyExecutor.getRoverCommand(pose_robot_sim, mc);
	ASSERT_LE(ui_error_code,1);
        dummyExecutor.getArmCommand(j_next_joints);
        
	Eigen::AngleAxisd toWCF, robotRot;
        toWCF = Eigen::AngleAxisd(robotPose.getYaw(), Eigen::Vector3d::UnitZ());
        if (fabs(mc.m_speed_ms) < 0.000001)
        {
            // Point turn
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
	robotPoseFile << robotPose.position.x() << " "
                      << robotPose.position.y() << " "
                      << robotPose.position.z() << " "
                      << robotPose.getYaw() << "\n";
	robotSimPoseFile << pose_robot_sim.position.x() << " "
                      << pose_robot_sim.position.y() << " "
                      << pose_robot_sim.position.z() << " "
                      << pose_robot_sim.getYaw() << "\n";


        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is (" << robotPose.position.x() << ", "
                  << robotPose.position.y() << ", " << robotPose.position.z()
                  << ") meters, with yaw " << robotPose.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Estimated Position (simulated) is (" << pose_robot_sim.position.x() << ", "
                  << pose_robot_sim.position.y() << ", " << pose_robot_sim.position.z()
                  << ") meters, with yaw " << pose_robot_sim.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Motion Command is (translation speed = " << mc.m_speed_ms
		  << " m/s, rotation speed = " << mc.m_turnRate_rads << " rad/s)" << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Current Joint Position and Next are:" << std::endl;
	for (uint i = 0; i < 6; i++)
        {
            std::cout << "                    Joint " << i << " current position is "
                      << j_current_joints.m_jointStates[i].m_position
                      << " degrees, and the next is " 
                      << j_next_joints.m_jointStates[i].m_position << " degrees" << std::endl;
        }
        std::cout << std::endl;
	std::cout << std::endl;
        usleep(100000);

	// Joints positions are now the ones commanded
        for (uint i = 0; i < 6; i++)
        {
            j_current_joints.m_jointStates[i].m_position
                = j_next_joints.m_jointStates[i].m_position;
        }
    }
    robotPoseFile.close();
    robotSimPoseFile.close();
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
