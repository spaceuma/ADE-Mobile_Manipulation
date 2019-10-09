#ifndef _UDP_CONFIG_HPP_
#define _UDP_CONFIG_HPP_

#include <string>
#include <stdint.h>
#include <set>
#include <vector>

#include <base/Time.hpp>

#include <udp/Enumerations.hpp>

namespace udp
{

enum Protocol {
    PROTO_UDP = 1,
    PROTO_DGR = 2,
    PROTO_TCP = 3
};

/**
 * The library just uses max_queue_size if available (both for input and output ports), 
 * default_max_queue_size otherwise.
 */
struct PortInfo {
    PortInfo() : port_name(), refresh_rate(0), max_queue_size(0), ack_mode(NO_ACK) {
    }
    
    std::string port_name;
    int refresh_rate;
    int max_queue_size;
    enum ACK_MODE ack_mode;
};

struct Config {
    int max_fragment_size;     // Larger messages are split into fragments. A larger fragment size 
                               // improves the data throughput. The parameter should be set to a 
                               // value between 1000 and 65000 prefering higher values.
    float resend_after;        // resend time in msec.
    bool init_listen;          // enables listening for incomming traffic
    bool init_send;            // enables data transmission for this module
    bool repeat_packets;       // Allows to deactivate resending of unconfirmed packages
    bool send_acks;            // Used in SequencedComm, allows to deactivate acks.
    float timeout_after;       // ttl (in ms) for incomplete and resend frames; all received fragments are discarded.
    int num_resend_tries;      // number of resend tries for a specific frame
    float rtt_limit;           // The round trip time limit in ms has to be set to a value which is bigger than the expected rtt.
    Protocol protocol;         // Defines the network protocol.
    
    std::string ip_addr;               // Ip address of the current system.
    bool listen_any;                   // Listen for all incomming data packets on listen port, so ip_addr is ignored.
    int listen_port;                   // Port to listen to. If listen_any is true, all connections to this port are regarded, 
                                       // otherwise just connections from ip_addr.
    std::vector<std::string> to_addrs; // Receiver ip addresses. A package can be sent to all these addresses.
    int to_port;                       // Destination port for transmitted data packets.
    int default_max_queue_size;        // Size of the sender and receiver buffer. If the buffer size is exceeded old elements will be discarded.
    int print_com_infos_ms;            // If > 0 prints communication status informations like bytes send, frames lost every print_com_infos_ms milliseconds.
    std::vector <PortInfo> port_infos; // Contains all port informations like name, ack, queue size and refresh rate.

    Config()
        : max_fragment_size(65000)     // 1450
        , resend_after(1000.0)
        , init_listen(true)
        , init_send(true)
        , repeat_packets(true)
        , send_acks(true)
        , timeout_after(6000.0)
        , num_resend_tries(1)
        , rtt_limit(1000.0)
        , protocol(PROTO_UDP)
        , ip_addr()
        , listen_any(false)
        , listen_port(0)
        , to_addrs()
        , to_port(0)
        , default_max_queue_size(100)
        , print_com_infos_ms(0)
    {
    }
};

}

#endif // _UDP_CONFIG_HPP_
