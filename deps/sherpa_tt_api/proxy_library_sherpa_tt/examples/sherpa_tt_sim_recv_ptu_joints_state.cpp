#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTTSim.hpp>

using namespace proxy_library;

int main() {
    
    // The proxy config.
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.print_com_infos_ms = 1000;
    config.udpConfig.max_fragment_size = 65000;
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1";
    config.udpConfig.listen_port = 20011;
    config.udpConfig.to_addrs.push_back("127.0.0.1");
    config.udpConfig.to_port = 20010;
    config.udpConfig.default_max_queue_size = 1;

    // The proxy.
    ProxyLibrarySherpaTTSim proxy(config);

    // The joints to fetch.
    Joints joints;

    // Fetch status indicator.
    bool isJointsFetched;
    
    while(true) {
        std::cout << "\n================================= " << std::endl;
        
        isJointsFetched = proxy.getSimPTUJointsState(joints);
        std::cout << "PTU Joints: " << isJointsFetched << std::endl;
        std::cout << "--------------------------------- " << std::endl;

        usleep(100000);
    }
}

