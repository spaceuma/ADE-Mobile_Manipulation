#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>


int main() {
    using namespace proxy_library;
    
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.max_fragment_size = 65000;        
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20001;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); // "192.168.1.21"
    config.udpConfig.to_port = 20000;
    config.udpConfig.print_com_infos_ms = 1000;
    
    ProxyLibrarySherpaTT proxy(config);
    
    // Create data types to test the client serialization.
    std::vector<uint8_t> data;
    int width = 2;
    int height = 3;
    data.resize(width*height);

    //DEM dem(1.0, 2.0, 1.0, 123, width, height, RGB_UINT8, data, "Ref-Frame", Center);

    DGPS dgps(123, 1.1, 2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9);
    Image image(123, width, height, RGB_UINT8, data);
    Vector3 vec(1.1, 2.2, 3.3);
    IMU imu(123, vec, vec, vec);

    Joints joints;
    joints.m_time = 123;
    JointState js(1.1, 2.2, 3.3, 4.4);
    for(int i=0; i<4; i++) {
        joints.m_jointStates.push_back(js);
        joints.m_jointNames.push_back("Joint" + std::to_string(i));
    }

    MotionCommand motion_cmd(1,2,3,4);
    Quaternion quat(1.1, 2.2, 3.3, 4.4);
    Pose pose(vec, quat, "RefName", "FrameName");
    
    // Test storeToFile and loadromFile.
    /*
    dem.storeToFile("dem.dat");
    DEM dem2;
    dem2.loadFromFile("dem.dat");
    std::cout << "dem1 m_metersPerPixelX: " << dem.m_metersPerPixelX << 
            ", m_pointOfReference: " << dem.m_pointOfReference << 
            ", Buffer size: " << dem.getBufferSize() << std::endl;
    std::cout << "dem2 m_metersPerPixelX: " << dem2.m_metersPerPixelX << 
            ", m_pointOfReference: " << dem2.m_pointOfReference << 
            ", Buffer size: " << dem2.getBufferSize() << std::endl;
    */

    // Send test data / motion and joint command.
    while(true) {
        //proxy.sendData("dem", &dem);
        //proxy.sendData("dgps", &dgps);
        //proxy.sendData("image", &image);
        //proxy.sendData("imu", &imu);
        //proxy.sendData("joints", &joints);
        //proxy.sendData("pose", &pose);
        //proxy.sendData("motion_out", &motion_cmd);
        
        proxy.sendMotionCommand(&motion_cmd);
        proxy.sendManipulatorJointCommand(&joints);
        
        usleep(100000); // sleep 1/10 of a second.
    }
    
    std::cin.get();
}
