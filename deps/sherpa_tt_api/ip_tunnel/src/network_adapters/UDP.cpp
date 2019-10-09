#include <udp/network_adapters/UDP.hpp>

namespace udp
{

UDP::UDP(const Config &config): UDPAdapter(config)
{
    if (config.init_listen)
        initReceive(false, config.listen_any);

    if (config.init_send)
        initSend(false);
}

UDP::~UDP()
{
    closeConns();
}
    
void UDP::initReceive(bool non_blocking, bool listen_any)
{
    if (sd_receiver >= 0) {
        LOG_WARN_S << "sd_listener allready created!";
    }

    sd_receiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sd_receiver < 0) {
        throw std::runtime_error("Could not create listener socket");
    }
    
    if (non_blocking) {
        setNonblock(sd_receiver);
    } else {
        setTimeout(sd_receiver, 5);
    }
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(conf.listen_port);
    if (listen_any) {
        addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, conf.ip_addr.c_str(), &(addr.sin_addr));
    }

    int reuse = 1;
    if (setsockopt(sd_receiver, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int)) < 0) {
        LOG_ERROR_S << "setsockopt error";
    }

    //30 MB
    int a = 1024 * 1024 * 30;
    if (setsockopt(sd_receiver, SOL_SOCKET, SO_RCVBUF, &a, sizeof(int)) == -1) {
        LOG_ERROR_S << "Error setting socket opts:" << strerror(errno);
    }
    
    if (bind(sd_receiver, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sd_receiver);
        throw std::runtime_error("could not bind receiver socket!");
    }   
}

void UDP::initSend(bool non_blocking)
{
    if (sd_sender >= 0) {
        LOG_WARN_S << "sd_sender allready created" << std::endl;
    }

    sd_sender = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sd_sender < 0) {
        throw std::runtime_error("could not create sender socket!");
    }

    if (non_blocking) {
        setNonblock(sd_sender);
    } else {
        setTimeout(sd_sender, 5);
    }
        
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_aton(conf.ip_addr.c_str(), &addr.sin_addr);

    int reuse = 1;
    if (setsockopt(sd_sender, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        LOG_ERROR_S << "failed to set sockopt SO_REUSEADDR for sd_sender";
    }
    
    if (bind(sd_sender, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sd_sender);
        throw std::runtime_error("could not bind sender socket");
    }     
}

int UDP::receiveData(std::string &sender_addr, std::vector< uint8_t > &buffer, int buffer_size)
{
    if (sd_receiver < 0) {
        LOG_WARN_S << "sd_receiver not created!";
        return -1;
    }
    
    buffer.resize(buffer_size);
    int bytesRead = 0;

    struct sockaddr_in src_addr;
    socklen_t src_addr_len = sizeof(src_addr);

    errno = 0;
    bytesRead = recvfrom(sd_receiver, &buffer[0], buffer_size, 0, (struct sockaddr *)&src_addr, &src_addr_len);
    if (bytesRead == -1) {
        //std::cerr << "UDP recvfrom error: " << strerror(errno) << std::endl;
        return bytesRead;
    }
    else if (bytesRead == 0) {
        return 0;
    }
    
    buffer.resize(bytesRead);
    sender_addr = std::string(inet_ntoa(src_addr.sin_addr));
    return bytesRead;
}

bool UDP::waitForData(const base::Time& timeout)
{
    if (sd_receiver < 0) {
        LOG_WARN_S << "sd_receiver not created!";
        return false;
    }
    
    fd_set receiveSet;

    FD_ZERO(&receiveSet);
    FD_SET(sd_receiver, &receiveSet);

    struct timeval tv = timeout.toTimeval();

    int retval = select(sd_receiver + 1, &receiveSet, NULL, NULL, &tv);

    errno = 0;
    if (retval == -1)
    {
        LOG_ERROR_S << "Error select: " << strerror(errno);
        return false;
    }        
    else if (retval) {
        return true;
    }

    // No data.
    return false;
}


int UDP::sendData(const char *buffer, int send_len, std::string rem_ip)
{    
    if(sd_sender == -1) {
        LOG_ERROR_S << "Filedescriptor sd_sender is not initialized (-1)";
        return -1;
    }

    /*
	printf("Send\n");
	for(int i=0; i<send_len; i++) {
		printf("%c(%d) ", buffer[i], (int)buffer[i]);
	}
	printf("\n");
	*/
    
    struct sockaddr_in remaddr;
    bzero(&remaddr, sizeof(remaddr));
    remaddr.sin_family = AF_INET;
    remaddr.sin_addr.s_addr = inet_addr(rem_ip.c_str());
    remaddr.sin_port = htons(conf.to_port);
    socklen_t rem_ip_len = sizeof(remaddr);
    
    errno = 0;
    int ret = sendto(sd_sender, buffer, send_len, 0, (struct sockaddr *)&remaddr, rem_ip_len);
    if(ret == -1) {
        LOG_ERROR_S << "UDP::sendData sendto failed (buffer size " << send_len << 
                "): " << strerror(errno);
    }
    return ret;
}

void UDP::closeConns()
{
    close(sd_receiver);
    close(sd_sender);
    sd_receiver = -1;
    sd_sender = -1;
}
    
}
