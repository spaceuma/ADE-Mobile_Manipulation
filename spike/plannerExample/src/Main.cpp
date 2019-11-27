#include <map>
#include <mutex>
#include <thread>
#include <iostream>
#include <plannerExample/biFMplanner.hpp>
#include <proxy_library/ProxyLibrary.hpp>
#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>

using namespace proxy_library;

//Code extracted from Sherpa API example
Joints buildDrivingPoseArmCommand(){
    Joints j;
    j.m_jointNames.push_back("arm_joint_1");
    j.m_jointNames.push_back("arm_joint_2");
    j.m_jointNames.push_back("arm_joint_3");
    j.m_jointNames.push_back("arm_joint_4");
    j.m_jointNames.push_back("arm_joint_5");
    j.m_jointNames.push_back("arm_joint_6");
    j.m_jointStates.resize(6);

    j.m_jointStates[0].m_position = 0;
    j.m_jointStates[1].m_position = -1.833;
    j.m_jointStates[2].m_position = 2.845;
    j.m_jointStates[3].m_position = -0.033;
    j.m_jointStates[4].m_position = -0.785;
    j.m_jointStates[5].m_position = 2.356;

    return j;
}

// Checks whether the arm state is the one desired
bool checkFinishedArmCommand(Joints armCommand, Joints armReadings)
{
  for (uint i = 0; i < 6; i++)
  {
    if (fabs(armCommand.m_jointStates[i].m_position -
            armReadings.m_jointStates[i].m_position ) > 0.01)
      return false;
  }
  return true;
}

// Dummy thread to command the arm
void moveArm(Joints armCommand)
{
  // The proxy config.
  Config config;
  config.printReceivedTypeInfos = true;
  config.udpConfig.print_com_infos_ms = 1000;
  config.udpConfig.max_fragment_size = 65000;
  config.udpConfig.init_send = true;
  config.udpConfig.ip_addr = "127.0.0.1"; //"127.0.0.1"; // "192.168.1.22"
  config.udpConfig.listen_port = 20001;
  config.udpConfig.to_addrs.push_back("127.0.0.1"); //"127.0.0.1" // "192.168.1.21"
  //config.udpConfig.to_addrs.push_back("10.250.247.2");
  config.udpConfig.to_port = 20000;
  config.udpConfig.default_max_queue_size = 1;

  // The proxy.
  ProxyLibrarySherpaTT proxy(config);

  std::cout << "Proxy is created" << std::endl;
  proxy.sendManipulatorJointCommand(&armCommand);

  std::cout << "Command is sent to SherpaTT" << std::endl;

  Joints jointReadings;
  bool st = false;
  while (!st)
  {
    st  = proxy.getManipulatorJointState(jointReadings);
    std::cout << "Waiting for Reading" << std::endl;
  }
  std::cout << "Last Joint Command is " << jointReadings.m_jointStates[5].m_position << std::endl;
  std::cout << "First Reading Completed" << std::endl;
  while(!checkFinishedArmCommand(armCommand, jointReadings))
  {
    std::cout << "\n================================= " << std::endl;
    st = proxy.getManipulatorJointState(jointReadings);
    std::cout << "ManipulatorJoints: " << st << std::endl;
    usleep(100000);
  }
}


int main(int argc, char** argv)
{

  std::thread executerThread(moveArm, buildDrivingPoseArmCommand());

  executerThread.join();

  return 0;
}
