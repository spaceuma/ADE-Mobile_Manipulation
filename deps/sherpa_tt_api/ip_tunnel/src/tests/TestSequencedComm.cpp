#include <unistd.h>
#include <stdio.h>      
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>
#include <boost/iterator/iterator_concepts.hpp>

#include <udp/network_adapters/UDP.hpp>
#include <udp/communications/SequencedComm.hpp>

namespace udp {

struct ThreadParameters {
    ThreadParameters(SequencedComm* seq_comm, Config config, std::string mode) : 
            seqComm(seq_comm), config(config), mode(mode) {
    }
    
    SequencedComm* seqComm;
    Config config;
    std::string mode;
};

struct SendData {
    SendData(DataContainer data, float send_interval_ms) : data(data), 
            sendInterval_ms(send_interval_ms) {
        for(unsigned int i=0; i<data.buffer.size(); i++) {
            data.buffer[i] = (i % 10) + 48;
        } 
    }
    
    DataContainer data;
    float sendInterval_ms;
    
    bool isRdy() {
        if(timer.getPassedTime_millisec() > sendInterval_ms) {
            timer.setStartTime();
            return true;
        }
        return false;
    }
    
    Timer timer;
};

void* thread_func(void* params) {
    using namespace udp;
    
    ThreadParameters* paras = (ThreadParameters*)params;
    
    SequencedComm* seq_comm = paras->seqComm;
    
    std::vector<SendData> send_data;
    
    // Creates for operator and robot different data packets with different ack modes
    // and sending frequency.
    if(paras->mode.compare("operator") == 0) {
        send_data.push_back(SendData(DataContainer("motion_in", 24, FRAME_ACK), 100));
    } else if(paras->mode.compare("robot") == 0) {
        
        send_data.push_back(SendData(DataContainer("pose_out", 24, NO_ACK), 10));
        send_data.push_back(SendData(DataContainer("image_out", 500, FRAME_ACK), 100));
        send_data.push_back(SendData(DataContainer("pointcloud_out", 2000000, FRAGMENT_ACK), 500));
        
    } else {
        std::cout << "Operator mode " << paras->mode << " is unknown." << std::endl;
        return NULL;
    }
    
    DataContainer received_data;
    std::vector<SendData>::iterator it;
    while(true) {
        
        for(it = send_data.begin(); it != send_data.end(); it++) {
            if(it->isRdy()) {
                seq_comm->forwardPacket(it->data);
            }
        }
        
        while(paras->seqComm->getPacket(received_data)) {
            /*
            std::cout << "Sample received " << sample1.port_name << " (id " << sample1.f_id << 
                    ") with size " << sample1.data.size() << std::endl;
            for(unsigned int i=0; i<sample1.data.size(); i++) {
                std::cout << sample1.data[i] << " ";
            }
            std::cout << std::endl;
            */
        } 
        usleep(10000); 
    }
    
    return NULL;
}

std::string getLocalIpAddress() {
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);
    std::string ip_addr("127.0.0.1");

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            std::string ip_addr_tmp(addressBuffer);
            if(ip_addr_tmp.compare("127.0.0.1") != 0) {
                ip_addr = ip_addr_tmp;
            }
        } 
    }
    if (ifAddrStruct!=NULL) 
        freeifaddrs(ifAddrStruct);
    
    return ip_addr;
}

} // end namespace udp

/**
 * Simulates a robot communicating with an operator.
 * The robots sends lot of sensor datas (some with FRAME_ACK) and a large file 
 * which use FRAGMENT_ACK.
 * The operator sends motion commands using FRAME_ACK.
 * Parameters: <mode=operator, robot> <this_ip> <this_port> <goal_ip> <goal_port>
 * operator and robot defines which type of communication should be created.
 * If no parameters are passed the communication will be started locally.
 */
int main(int argc, char** argv) {  
    using namespace udp;
    
    std::string this_ip="127.0.0.1", goal_ip="127.0.0.1", operator_mode="";
    int this_port=20000, goal_port=20001;
    bool local_communication = (argc == 1);
    
    if(argc != 1 && argc != 6) {
        std::cout << "Parameters: <mode=operator, robot> <this_ip> <this_port> <goal_ip> <goal_port>" << std::endl;
        std::cout << "Without parameters a local communication will be initiated" << std::endl;
        return 1;
    }
    
    if(argc == 6) {
        operator_mode = std::string(argv[1]);
        this_ip       = std::string(argv[2]); // getLocalIpAddress();
        this_port     = std::stoi(argv[3]);
        goal_ip       = std::string(argv[4]);
        goal_port     = std::stoi(argv[5]);
    }
    
    std::cout << "Sending from " << operator_mode << " " << this_ip << ":" << 
            this_port << " to " << goal_ip << ":" << goal_port << std::endl;
    
    Config config;
    config.timeout_after = 10000; // time-to-live for unprocessed frames
    config.ip_addr = this_ip;
    config.listen_port = this_port;
    config.to_addrs.push_back(goal_ip);
    config.to_port = goal_port;
    config.rtt_limit = 1000;
    config.resend_after = 1000;
    config.num_resend_tries = 1;
    config.max_fragment_size = 65000; // packet size.
    config.print_com_infos_ms = 1000;
    
    if(local_communication) {
        SequencedComm operator_(config);
        ThreadParameters operator_paras(&operator_, config, "operator");
        pthread_t operator_thread;
        pthread_create(&operator_thread, 0, thread_func, &operator_paras);
        
        
        config.listen_port = goal_port;
        config.to_port = this_port;
        SequencedComm robot(config);
        ThreadParameters robot_paras(&robot, config, "robot");
        pthread_t robot_thread;
        pthread_create(&robot_thread, 0, thread_func, &robot_paras);
        
        
        
        while(true) {
            /*
            std::cout << operator_paras.config.ip_addr << ":" << operator_paras.config.listen_port << std::endl;
            std::cout << operator_paras.seqComm->getComInfoCopy().toString() << std::endl;
            std::cout << robot_paras.config.ip_addr << ":" << robot_paras.config.listen_port << std::endl;
            std::cout << robot_paras.seqComm->getComInfoCopy().toString() << std::endl;
            */
            sleep(1);
        }
        
        
    } else {
        SequencedComm communicator(config);
        ThreadParameters paras(&communicator, config, operator_mode);
        pthread_t thread;
        pthread_create(&thread, 0, thread_func, &paras);
        
        while(true) {
            /*
            std::cout << paras.config.ip_addr << ":" << paras.config.listen_port << std::endl;
            std::cout << paras.seqComm->getComInfoCopy().toString() << std::endl;
            */
            sleep(1);
        }
        
        
    }
}

    
    