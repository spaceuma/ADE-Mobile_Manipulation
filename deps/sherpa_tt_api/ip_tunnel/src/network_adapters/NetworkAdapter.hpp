#ifndef NETWORKADAPTER_HPP
#define NETWORKADAPTER_HPP

#include <udp/Frame.hpp>
#include <udp/Config.hpp>

#include <string>
#include <vector>
#include <map>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sstream>
#include <sys/types.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <stdint.h>
#include <arpa/inet.h>

namespace udp {

/**
 * base for specific network communication protocol implementation.
 */
class NetworkAdapter {
protected:
    base::Time last;
    double bytesSendInPeriod;
    
    NetworkAdapter(const Config &config = Config());
    ~NetworkAdapter();
    
public:
    // contains configuration information for this library
    Config conf;
    
    /**
     * contains protocol specific implementation for data transmission.
     *
     * @param storage reference to receive data storage
     * @param sender_addr reference to strorage for senders ip address
     */
    //int sendData(std::vector< uint8_t > &data, int data_size, std::string rem_ip, bool needs_ack);
    
    /**
     * sets socket to non blocking mode
     *
     * @param sd socket descriptor
     */
    void setNonblock(int sd);
    void setTimeout(int sd, int secs);
    
    /**
     * contains protocol specific implementation for collecting network data.
     *
     * @param data send data reference
     * @param data_size size of send data
     * @param rem_ip recipients ip address
     * @return num of sent bytes
     */
    virtual int receiveData(std::string &sender_addr, std::vector< uint8_t > &buffer, int buffer_size) =0;
    
    /**
     * Blocks on the filedescriptor of the socket, 
     * until either data is received, or the timeout 
     * is reached. 
     * 
     * @return true is new data is available
     * */
    virtual bool waitForData(const base::Time &timeout) = 0;
    
    /**
     * closes send and receive socket.
     */
    virtual void closeConns() =0;
    
    virtual int sendData(const char *buffer, int send_len, std::string rem_ip) =0;
    
    virtual int getSdReceiver() =0;
    
 protected:
    struct ifaddrs *addrs;
};

}

#endif