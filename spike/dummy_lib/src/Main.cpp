#include <map>
#include <mutex>
#include <thread>
#include <iostream>
#include <dummy_lib/Dummy.hpp>
#include <proxy_library/ProxyLibrary.hpp>
#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>

using namespace proxy_library;

int main(int argc, char** argv)
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

  dummy_lib::DummyClass dummyClass;
  dummyClass.welcome();

  while(true) {
      std::cout << "\n================================= " << std::endl;
      bool st;

      /*DGPS dgps;
      st = proxy.getDGPS(dgps);
      std::cout << "DGPS: " << st << std::endl;

      IMU imu;
      st = proxy.getIMUData(imu);
      std::cout << "IMU: " << st << std::endl;*/

      Joints mjoints;
      st = proxy.getManipulatorJointState(mjoints);
      std::cout << "ManipulatorJoints: " << st << std::endl;
/*
      Joints rjoints;
      st = proxy.getMobileBaseJointState(rjoints);
      std::cout << "MobileBaseJoints: " << st << std::endl;

      Pose pose;
      st = proxy.getPose(pose);
      std::cout << "Pose: " << st << std::endl;
      std::cout << "--------------------------------- " << std::endl;
*/
      usleep(100000);
  }

  return 0;
}
