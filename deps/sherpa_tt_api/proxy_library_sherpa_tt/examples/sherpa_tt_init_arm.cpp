#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>

int main() {
    using namespace proxy_library;
    
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.max_fragment_size = 65000;
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1"; //"127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20001;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); //"127.0.0.1" // "192.168.1.21"
    //config.udpConfig.to_addrs.push_back("10.250.247.2");
    config.udpConfig.to_port = 20000;
    config.udpConfig.print_com_infos_ms = 0;
    config.udpConfig.default_max_queue_size = 1;

    ProxyLibrarySherpaTT proxy(config);

    // Send test data / motion and joint command.
    Joints mjoints;
    while(true) {
        bool st = proxy.getManipulatorJointState(mjoints);
        std::cout << "ManipulatorJoints: " << st << std::endl;
        if(st){
            std::cout << "received valid sample:\n" << mjoints.toString() << std::endl;
            break;
        }

        usleep(100000);
    }
    std::cout  <<"sending"<<std::endl;
    Joints sjoints = mjoints;
    sjoints.m_jointStates[0].m_position = sjoints.m_jointStates[0].m_position;
    proxy.sendManipulatorJointCommand(&sjoints);
    std::cout  <<"waiting"<<std::endl;
    sleep(5);
    std::cout  <<"done"<<std::endl;
}
