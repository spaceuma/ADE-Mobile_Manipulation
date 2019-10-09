#ifndef UDP_HPP
#define UDP_HPP

#include <udp/network_adapters/UDPAdapter.hpp>

namespace udp
{

/**
 * implements UDP protocol for network communication.
 */
class UDP : public UDPAdapter {
public:
    UDP(const Config &config = Config());
    ~UDP();
    void initReceive(bool non_blocking, bool listen_any);
    void initSend(bool non_blocking);
    int receiveData(std::string &sender_addr, std::vector<uint8_t> &buffer, int buffer_size);

    /**
     * Blocks on the filedescriptor of the socket, 
     * until either data is received, or the timeout 
     * is reached. 
     * 
     * @return true is new data is available
     * */
    bool waitForData(const base::Time &timeout);
    
    int sendData(const char *buffer, int send_len, std::string rem_ip);
    void closeConns();
};

}

#endif