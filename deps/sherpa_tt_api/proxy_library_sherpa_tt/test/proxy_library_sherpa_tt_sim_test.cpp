#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTTSim.hpp>

using namespace proxy_library;

/**
 * Generates test data for joints.
 * The test data consists of random values for joint properties.
 * 
 * These properties are: position, speed, effort, acceleration, and raw.
 */
void generateJointsTestData(Joints& joints);

int main() {
    
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.max_fragment_size = 65000;        
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20011;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); // "192.168.1.21"
    config.udpConfig.to_port = 20010;
    config.udpConfig.print_com_infos_ms = 1000;
    
    ProxyLibrarySherpaTTSim proxy(config);
    
    Joints targetJoints;
    generateJointsTestData(targetJoints);

    // Send test data
    while(true) {
        proxy.sendSimPTUJointsCommand(&targetJoints);
        usleep(100000); // sleep 1/10 of a second.
    }

    std::cin.get();
}


void generateJointsTestData(Joints& joints){
    joints.m_time = 1234;

    // Set motor joint names.
    joints.m_jointNames.push_back("mastcam_ptu_pan_motor");
    joints.m_jointNames.push_back("mastcam_ptu_tilt_motor");

    // Set target pan state.
    JointState jointStatePan;
    jointStatePan.m_position = 3.14;
    jointStatePan.m_speed = 0.5;
    joints.m_jointStates.push_back(jointStatePan);

    // Set target tilt state.
    JointState jointStateTilt;
    jointStateTilt.m_position = 1.1;
    jointStateTilt.m_speed = 0.5;
    joints.m_jointStates.push_back(jointStateTilt);
}