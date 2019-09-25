#pragma once

#include <udp/Config.hpp>

namespace proxy_library
{  
/*
// Important: Take care that the sender and receiver use the same value for 
// max_send_bytes. Otherwise fragmentation leads to a different package size error.
sequencedCommConfig.max_fragment_size = 65000;        
sequencedCommConfig.init_send = true;
sequencedCommConfig.ip_addr = "127.0.0.1"; // "192.168.1.22"
sequencedCommConfig.listen_port = 20001;
sequencedCommConfig.to_addrs.push_back("127.0.0.1"); // "192.168.1.21"
sequencedCommConfig.to_port = 20000;
sequencedCommConfig.print_com_infos_ms = 1000;    
*/

class Config {
 public:
    Config() : udpConfig(), printReceivedTypeInfos(false){
        
    } 
    
    udp::Config udpConfig;
    // If set to true toString() of all received types will be called.
    bool printReceivedTypeInfos; 
};
    
} // end namespace proxy_library