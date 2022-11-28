#include "MobileManipExecutor.h"
#include "MotionCommand.h"
#include "Waypoint.hpp"
#include "mmFileManager.h"
#include <gtest/gtest.h>
#include <unistd.h>

TEST(MMExecutorTest, nominal_working_test)
{
    // Reading the DEM
    std::vector<std::vector<double>> vvd_cost_map_shadowing, vvd_cost_map_no_shadowing,
        vvd_elevation_map;
    ASSERT_NO_THROW(
        readMatrixFile("test/unit/data/input/MMMotionPlanTest/RH1_Zone1_10cmDEM.csv",
                       vvd_elevation_map));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMotionPlanTest/RH1_Zone1_costMap.txt",
                                   vvd_cost_map_shadowing));
    base::Waypoint w_rover_pos, samplePos;
    ASSERT_NO_THROW(w_rover_pos =
                        getWaypoint("test/unit/data/input/MMMotionPlanTest/rover_pos_01.txt"))
        << "Input Rover Waypoint file is missing";
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMotionPlanTest/sample_pos_01.txt"))
        << "Input Sample Waypoint file is missing";

    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if(if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
        throw "Cannot open urdf model path ";
    }

    double res = 0.1;    // meters
    double zRes = 0.08;

    MobileManipMap mmmap(vvd_elevation_map, vvd_cost_map_shadowing, res, samplePos, 1.0, 0.94);

    std::vector<Waypoint> vw_path;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    readPath("test/unit/data/input/MMExecutorTest/path.txt", vw_path);
    readMatrixFile("test/unit/data/input/MMExecutorTest/armMotionProfile.txt",
                   vvd_arm_motion_profile);
    MotionPlan * dummyPlan = new MotionPlan(&mmmap, s_urdf_path, vw_path, vvd_arm_motion_profile);

    Pose robotPose, pose_robot_sim;

    robotPose.position = Eigen::Vector3d(vw_path[0].position[0], vw_path[0].position[1], 0);

    robotPose.orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(10.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()));

    proxy_library::MotionCommand mc;
    double dt = 0.5, yaw, d_pos_error_x = 0, d_pos_error_y = 0;
    unsigned int ui_error_code;

    std::vector<proxy_library::JointState> vj_current_jointstates;
    vj_current_jointstates.resize(6);
    for(uint i = 0; i < 6; i++)
    {
        vj_current_jointstates[i].m_speed = 0.0;
    }

    vj_current_jointstates[0].m_position = 1.41;
    vj_current_jointstates[1].m_position = -0.76;
    vj_current_jointstates[2].m_position = 1.22;
    vj_current_jointstates[3].m_position = -1.64;
    vj_current_jointstates[4].m_position = 1.43;
    vj_current_jointstates[5].m_position = 2.03;

    proxy_library::Joints j_current_joints(0, vj_current_jointstates);
    proxy_library::Joints j_next_joints(0, vj_current_jointstates);

    std::ofstream robotPoseFile, robotSimPoseFile;
    robotPoseFile.open("test/unit/data/results/MMExecutorTest/roverRealPos.txt");
    robotSimPoseFile.open("test/unit/data/results/MMExecutorTest/roverEstimatedPos.txt");

    MobileManipExecutor dummyExecutor(dummyPlan, s_urdf_path);

    std::cout << " The loop starts " << std::endl;
    while(ui_error_code != 2)
    {
        //	ASSERT_TRUE(dummyExecutor.isArmMoving(j_current_joints));
        d_pos_error_x = (double)(rand() % 500 - 250) * 0.0001;
        d_pos_error_y = (double)(rand() % 500 - 250) * 0.0001;

        pose_robot_sim.position = robotPose.position;
        pose_robot_sim.position.x() += d_pos_error_x;
        pose_robot_sim.position.y() += d_pos_error_y;
        pose_robot_sim.orientation = robotPose.orientation;
        ui_error_code =
            dummyExecutor.getCoupledCommand(pose_robot_sim, j_current_joints, mc, j_next_joints);
        ASSERT_LE(ui_error_code, 2);

        Eigen::AngleAxisd toWCF, robotRot;
        toWCF = Eigen::AngleAxisd(robotPose.getYaw(), Eigen::Vector3d::UnitZ());
        if(fabs(mc.m_speed_ms) < 0.000001)
        {
            // Point turn
            robotRot = AngleAxisd(mc.m_turnRate_rads * dt, Eigen::Vector3d::UnitZ());
            robotPose.orientation = Eigen::Quaterniond(robotRot) * robotPose.orientation;
        }
        else if(fabs(mc.m_turnRate_rads) < 0.000001)
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
        robotPoseFile << robotPose.position.x() << " " << robotPose.position.y() << " "
                      << robotPose.position.z() << " " << robotPose.getYaw() << "\n";
        robotSimPoseFile << pose_robot_sim.position.x() << " " << pose_robot_sim.position.y() << " "
                         << pose_robot_sim.position.z() << " " << pose_robot_sim.getYaw() << "\n";

        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is ("
                  << robotPose.position.x() << ", " << robotPose.position.y() << ", "
                  << robotPose.position.z() << ") meters, with yaw "
                  << robotPose.getYaw() * 180 / M_PI << " degrees" << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Estimated Position (simulated) is ("
                  << pose_robot_sim.position.x() << ", " << pose_robot_sim.position.y() << ", "
                  << pose_robot_sim.position.z() << ") meters, with yaw "
                  << pose_robot_sim.getYaw() * 180 / M_PI << " degrees" << std::endl;
        std::cout
            << "\033[32m[----------]\033[0m [INFO] Rover Motion Command is (translation speed = "
            << mc.m_speed_ms << " m/s, rotation speed = " << mc.m_turnRate_rads << " rad/s)"
            << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Current Joint Position and Next are:"
                  << std::endl;
        for(uint i = 0; i < 6; i++)
        {
            std::cout << "                    Joint " << i << " current position is "
                      << j_current_joints.m_jointStates[i].m_position
                      << " degrees, and the next is " << j_next_joints.m_jointStates[i].m_position
                      << " degrees" << std::endl;
        }
        std::cout << std::endl;
        std::cout << std::endl;
        //        usleep(100000);

        // proxy_library::Joints positions are now the ones commanded
        for(uint i = 0; i < 6; i++)
        {
            j_current_joints.m_jointStates[i].m_position =
                j_next_joints.m_jointStates[i].m_position;
        }
    }
    robotPoseFile.close();
    robotSimPoseFile.close();
}

/*TEST(MMExecutorTest, rover_out_of_corridor_test)
{

    std::vector<Waypoint> vw_path;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    readPath("test/unit/data/input/MMExecutorTest/path.txt", vw_path);
    readMatrixFile("test/unit/data/input/MMExecutorTest/armMotionProfile.txt",
                   vvd_arm_motion_profile);
    MotionPlan * dummyPlan = new MotionPlan(vw_path, vvd_arm_motion_profile);

    Pose robotPose;

    proxy_library::MotionCommand mc;
    unsigned int ui_error_code;
    robotPose.position
        = Eigen::Vector3d(vw_path[0].position[0], vw_path[0].position[1], 0);

    std::vector<JointState> vj_current_jointstates;
    vj_current_jointstates.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        vj_current_jointstates[i].m_speed = 0.0;
    }

    vj_current_jointstates[0].m_position = 0.39;
    vj_current_jointstates[1].m_position = -1.83;
    vj_current_jointstates[2].m_position = 2.79;
    vj_current_jointstates[3].m_position = 0.0;
    vj_current_jointstates[4].m_position = -0.5;
    vj_current_jointstates[5].m_position = 2.3562;

    proxy_library::Joints j_current_joints(0, vj_current_jointstates);
    proxy_library::Joints j_next_joints(0, vj_current_jointstates);

    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
    throw "Cannot open urdf model path ";
    }
    MobileManipExecutor dummyExecutor(dummyPlan, j_current_joints, s_urdf_path);

    // Robot out of the corridor from the start
    robotPose.position.x()+=1.0;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is (" <<
robotPose.position.x() << ", "
                  << robotPose.position.y() << ", " << robotPose.position.z()
                  << ") meters, with yaw " << robotPose.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;


    ui_error_code = dummyExecutor.getCoupledCommand(robotPose, j_current_joints, mc, j_next_joints);
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is (" <<
robotPose.position.x() << ", "
                  << robotPose.position.y() << ", " << robotPose.position.z()
                  << ") meters, with yaw " << robotPose.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;
    ASSERT_EQ(ui_error_code,3) << "\033[31m[----------]\033[0m Motion Command is ( transl = " <<
mc.m_speed_ms << ", rot = " << mc.m_turnRate_rads << ") ";


    // Robot out of the corridor after the start
    robotPose.position.x()-=1.0;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is (" <<
robotPose.position.x() << ", "
                  << robotPose.position.y() << ", " << robotPose.position.z()
                  << ") meters, with yaw " << robotPose.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;

    robotPose.position.x()+=1.0;
    ui_error_code = dummyExecutor.getCoupledCommand(robotPose, j_current_joints, mc, j_next_joints);
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is (" <<
robotPose.position.x() << ", "
                  << robotPose.position.y() << ", " << robotPose.position.z()
                  << ") meters, with yaw " << robotPose.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;
    ui_error_code = dummyExecutor.getCoupledCommand(robotPose, j_current_joints, mc, j_next_joints);
    ASSERT_EQ(ui_error_code,3) << "\033[31m[----------]\033[0m Motion Command is ( transl = " <<
mc.m_speed_ms << ", rot = " << mc.m_turnRate_rads << ") ";

}

TEST(MMExecutorTest, armnotworking_test)
{

    std::vector<Waypoint> vw_path;
    std::vector<std::vector<double>> vvd_arm_motion_profile;
    readPath("test/unit/data/input/MMExecutorTest/path.txt", vw_path);
    readMatrixFile("test/unit/data/input/MMExecutorTest/armMotionProfile.txt",
                   vvd_arm_motion_profile);
    MotionPlan * dummyPlan = new MotionPlan(vw_path, vvd_arm_motion_profile);

    Pose robotPose, pose_robot_sim;

    robotPose.position
        = Eigen::Vector3d(vw_path[0].position[0], vw_path[0].position[1], 0);

    robotPose.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(10.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()));


    proxy_library::MotionCommand mc;
    double dt = 0.1, yaw, d_pos_error_x = 0, d_pos_error_y = 0;
    unsigned int ui_error_code;

    std::vector<JointState> vj_current_jointstates;
    vj_current_jointstates.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        vj_current_jointstates[i].m_speed = 0.0;
    }

    vj_current_jointstates[0].m_position = 0.39;
    vj_current_jointstates[1].m_position = -1.83;
    vj_current_jointstates[2].m_position = 2.79;
    vj_current_jointstates[3].m_position = 0.0;
    vj_current_jointstates[4].m_position = -0.5;
    vj_current_jointstates[5].m_position = 2.3562;

    proxy_library::Joints j_current_joints(0, vj_current_jointstates);
    proxy_library::Joints j_next_joints(0, vj_current_jointstates);

    std::ofstream robotPoseFile, robotSimPoseFile;
    robotPoseFile.open("test/unit/data/results/MMExecutorTest/roverRealPos_armnotworking.txt");
    robotSimPoseFile.open("test/unit/data/results/MMExecutorTest/roverEstimatedPos_armnotworking.txt");

    std::ifstream if_urdf_path("data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
    throw "Cannot open urdf model path ";
    }
    MobileManipExecutor dummyExecutor(dummyPlan, j_current_joints, s_urdf_path);

    uint ui_loop_counter = 0;
    while (!dummyExecutor.isRoverFinished())
    {
//	ASSERT_TRUE(dummyExecutor.isArmMoving(j_current_joints));
        d_pos_error_x = (double)(rand() % 500 - 250) * 0.0001;
        d_pos_error_y = (double)(rand() % 500 - 250) * 0.0001;

        pose_robot_sim.position = robotPose.position;
    pose_robot_sim.position.x() += d_pos_error_x;
    pose_robot_sim.position.y() += d_pos_error_y;
        pose_robot_sim.orientation = robotPose.orientation;
        ui_error_code = dummyExecutor.getCoupledCommand(pose_robot_sim, j_current_joints, mc,
j_next_joints); ASSERT_FALSE(ui_error_code==5);

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


        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Position is (" <<
robotPose.position.x() << ", "
                  << robotPose.position.y() << ", " << robotPose.position.z()
                  << ") meters, with yaw " << robotPose.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Estimated Position (simulated) is ("
<< pose_robot_sim.position.x() << ", "
                  << pose_robot_sim.position.y() << ", " << pose_robot_sim.position.z()
                  << ") meters, with yaw " << pose_robot_sim.getYaw() * 180 / M_PI << " degrees"
                  << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Rover Motion Command is (translation speed
= " << mc.m_speed_ms
          << " m/s, rotation speed = " << mc.m_turnRate_rads << " rad/s)" << std::endl;
        std::cout << "\033[32m[----------]\033[0m [INFO] Current Joint Position and Next are:" <<
std::endl; for (uint i = 0; i < 6; i++)
        {
            std::cout << "                    Joint " << i << " current position is "
                      << j_current_joints.m_jointStates[i].m_position
                      << " degrees, and the next is "
                      << j_next_joints.m_jointStates[i].m_position << " degrees" << std::endl;
        }
        std::cout << std::endl;
    std::cout << std::endl;
//        usleep(100000);

    // Joints positions are now the ones commanded
        if (ui_loop_counter > 201)
    {
        ASSERT_EQ(ui_error_code,6) << "\033[31m[----------]\033[0m Motion Command is ( transl = " <<
mc.m_speed_ms << ", rot = " << mc.m_turnRate_rads << ") "; break;
    }
    if (ui_loop_counter > 200)
    {
            for (uint i = 0; i < 6; i++)
            {
                j_current_joints.m_jointStates[i].m_position
                    = j_next_joints.m_jointStates[i].m_position;
            }

            j_current_joints.m_jointStates[0].m_position = 0.0;
    }
    ui_loop_counter++;
    }
    robotPoseFile.close();
    robotSimPoseFile.close();
}*/
