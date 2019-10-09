#include <udp/network_adapters/NetworkAdapter.hpp>

namespace udp
{

NetworkAdapter::NetworkAdapter(const Config &config)
    : conf(config), addrs(NULL)
{
    if (conf.max_fragment_size < 50)
        throw std::invalid_argument("Wrong param for max_fragment_size (< 50)");
    if (conf.resend_after < 50)
        throw std::invalid_argument("Wrong param for resend_after (< 50)");
    if (conf.timeout_after < 50)
        throw std::invalid_argument("Wrong param for timeout_after (< 50)");
    if (conf.init_listen && conf.listen_port < 2000)
        throw std::invalid_argument("Wrong param for listen_port (< 2000)");
    if (conf.init_send && conf.to_port < 2000)
        throw std::invalid_argument("Wrong param for to_port (< 2000)");
    if (conf.rtt_limit < 50)
        throw std::invalid_argument("Wrong param for rtt_limit (< 50)");
    if (conf.num_resend_tries < 0)
        throw std::invalid_argument("Wrong param for num_resend_tries (< 0)");
  
    struct ifaddrs *ifa;
    char ip4[INET_ADDRSTRLEN];

    if (getifaddrs(&addrs) != 0) {
        LOG_ERROR_S << "getifaddrs() error";
    }

    bool valid_host_ip = false;
    for (ifa = addrs; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL || !(ifa->ifa_flags & IFF_UP))
            continue;

        void *inAddr;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in *s = (struct sockaddr_in *)ifa->ifa_addr;
            inAddr = &s->sin_addr;
            if (!inet_ntop(ifa->ifa_addr->sa_family, inAddr, ip4, sizeof(ip4)))
                LOG_ERROR_S << "Error in ntop";
            else {
                LOG_INFO_S << "Found net dev ip addr: " << ip4 << " [" << conf.ip_addr << "]";

                if (conf.ip_addr == std::string(ip4, 0, conf.ip_addr.size())) {
                    valid_host_ip = true;
                    break;
                }
            }
        }
    }

    if (!valid_host_ip)
        throw std::invalid_argument("Host ip addr is not valid");
    
    last = base::Time::now();
    bytesSendInPeriod = 0;
}

NetworkAdapter::~NetworkAdapter()
{
    if(addrs != NULL) {
        freeifaddrs(addrs);
    }
}

void NetworkAdapter::setNonblock(int sd)
{
    int nonBlocking = 1;
    if (fcntl(sd, F_SETFL, O_NONBLOCK, nonBlocking) == -1) {
        LOG_ERROR_S << "Failed to set sockt to non blocking mode";
    }
}

void NetworkAdapter::setTimeout(int sd, int secs)
{
    struct timeval tv;
    tv.tv_sec = secs;
    tv.tv_usec = 0;

    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(struct timeval));
}

} // end namespace udp
