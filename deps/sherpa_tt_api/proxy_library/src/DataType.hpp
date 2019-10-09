#pragma once

#include <mutex>

#include <udp/Enumerations.hpp>

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library
{  

/**
 * Contains the data type and additional type contents like mutexes and booleans 
 * whether new informations are available.
 */
struct DataType {
    
    DataType() : baseType(NULL), mutex(), newInformations(false), ackMode(udp::NO_ACK) {
    }
    
    DataType(BaseType* base_type, udp::ACK_MODE ack_mode=udp::NO_ACK) : 
            baseType(base_type), mutex(), newInformations(false), ackMode(ack_mode) {
    }
    
    ~DataType() {
        delete baseType;
        baseType = NULL;
    }
    
    BaseType* baseType;
    std::mutex mutex;
    bool newInformations;
    udp::ACK_MODE ackMode;
};

} // end namespace proxy_library