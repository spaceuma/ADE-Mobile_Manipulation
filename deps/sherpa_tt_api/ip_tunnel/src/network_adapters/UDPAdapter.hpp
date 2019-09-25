#ifndef UDPADAPTER_HPP
#define UDPADAPTER_HPP

#include <udp/network_adapters/NetworkAdapter.hpp>

namespace udp {

class UDPAdapter : public NetworkAdapter {
protected:
    UDPAdapter(const Config& config) : NetworkAdapter(config), sd_sender(-1), sd_receiver(-1)
    {
    };

    virtual ~UDPAdapter() {};
    
public: 
    int sd_sender, sd_receiver;
    
    int getSdReceiver()
    {
        return sd_receiver;
    };

    virtual void initReceive(bool non_blocking, bool listen_any) =0;
    virtual void initSend(bool non_blocking) =0;
};

}

#endif
