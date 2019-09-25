#include <udp/Communication.hpp>

//#include <sys/time.h>
//#include <time.h>

#include <udp/network_adapters/UDP.hpp>
#include <udp/Communication.hpp>

namespace udp
{

int Communication::SENDER_THREAD_USLEEP_DEFAULT = 100000;
int Communication::SENDER_THREAD_USLEEP_MINIMUM = 1000;
    
// PUBLIC
bool Communication::forwardPacket(DataContainer& data)
{       
    std::string port_name = data.getControlContent(PORT_NAME);
    if(port_name.empty()) {
        std::cerr << "Data cannot be forwarded, control content PORT_NAME is missing" << std::endl;
        return false;
    }
    
    // Adds the control header, method takes care that this only happens once.
    if(!data.createControlHeader(mConfig.max_fragment_size)) {
        LOG_ERROR_S << "Appending control header error";
        return false;
    }

    // If no receivers have been specified the default recveivers of the config file
    // are used. 
    if (data.addrs.empty()) {
        std::vector<std::string>::iterator it = mConfig.to_addrs.begin();
        for(; it != mConfig.to_addrs.end(); it++) {
            data.addrs.insert(*it);
        }
    }
    
    // Adds the data container / frame id.
    data.id = nextFrameId();
    
    senderQueuesMutex.lock();
    senderQueues[port_name].push(data); // push to the back.
    comInfoMutex.lock();
    mComInfo.senderQueuesSize++;
    comInfoMutex.unlock();

    std::map< std::string, std::queue<DataContainer > >::iterator it_queue;
    it_queue = senderQueues.find(port_name);

    // Takes care (if set) that the max queue size for each port is not exceeded.
    uint16_t max_elements = mConfig.default_max_queue_size;
    std::map< std::string, PortInfo >::iterator it_info = portInfos.find(port_name);
    if(it_info != portInfos.end() && it_info->second.max_queue_size > 0) {
        max_elements = it_info->second.max_queue_size;
    }
    
    int size_diff = it_queue->second.size() - max_elements;
    // Max queue size exceeded.
    if(size_diff > 0) {
        LOG_INFO_S << "Sender: Max queue size of port " << port_name << " exceeded, " << 
                size_diff  << " old elements will be removed";
        for(int i=0; i < size_diff; i++) {
            it_queue->second.pop(); // Remove oldest elements.
            comInfoMutex.lock();
            mComInfo.senderQueuesSize--;
            mComInfo.senderQueuesDiscarded++;
            comInfoMutex.unlock();
        }
    }
    senderQueuesMutex.unlock();
    return true;
}

bool Communication::getPacket(DataContainer& data, std::string port_name) {
    
    receivedPortNamesMutex.lock();
    if(receivedPortNames.size() == 0) { 
        //std::cerr << "Sample cannot be returned, no data available" << std::endl;
        receivedPortNamesMutex.unlock();
        return false;
    }
    
    if(port_name.empty()) {
        // Use the queue which contains the oldest received sample.
        port_name = *(receivedPortNames.begin());
        receivedPortNames.pop_front();  
    }
    receivedPortNamesMutex.unlock();
    
    receiverQueuesMutex.lock();
    std::map< std::string, std::queue< DataContainer > >::iterator it;
    it = receiverQueues.find(port_name);
    
    if(it == receiverQueues.end()) {
        std::cerr << "Packet cannot be returned, no queue '" << port_name << 
                "' available" << std::endl;
        receiverQueuesMutex.unlock();
        return false;
    }
    
    if(it->second.empty()) {
        std::cerr << "Queue " << port_name << " is empty" <<  std::endl;
        return false;
    }
    
    data = DataContainer(it->second.front()); // Creates a copy of the data packet.
    it->second.pop(); // Removes front (oldest) element from the list.
    comInfoMutex.lock();
    mComInfo.receiverQueuesSize--;
    comInfoMutex.unlock();
    receiverQueuesMutex.unlock();
    return true;
}

ComInfo Communication::getComInfoCopy() {
    struct ComInfo com_info_copy;
    comInfoMutex.lock();
    mComInfo.fillNumReceivedPackets(numReceivedPackets);
    com_info_copy = mComInfo;
    comInfoMutex.unlock();
    return com_info_copy;
}

// PROTECTED
Communication::Communication(const Config &config) : mConfig(config), 
        stopSending(false), 
        stopReceiving(false)
{   
    //forwardInputPort(VIRTUAL_ACK_PORT);

    switch (config.protocol) {
    case PROTO_UDP:
        networkAdapter = new UDP(config);
        break;
    default:
        networkAdapter = new UDP(config);
        break;
    }
    
    if (pthread_create(&senderThread, 0, dataForwarderStatic, this) == 0)
        LOG_INFO_S << "Successfully created sender thread for " << this;
    else {
        LOG_FATAL_S << "Could not create sender thread for " << this;
        throw std::runtime_error("Could not create sender thread");
    }
    
    if (pthread_create(&receiverThread, 0, dataReceiverStatic, this) == 0)
        LOG_INFO_S << "Successfully created receiver thread for " << this << std::endl;
    else {
        LOG_FATAL_S << "Could not create receiver thread for " << this;
        throw std::runtime_error("Could not create receiver thread");
    }
    
    lastFrameID = 0;
    
    rttInfo.defaultResendAfter = mConfig.resend_after;
    
    // Fill the port infos.
    portInfos.clear();
    std::vector <PortInfo>::iterator it = mConfig.port_infos.begin();
    for(; it != mConfig.port_infos.end(); it++) {
        portInfos[it->port_name] = *it;
    }
}

Communication::~Communication()
{   
    stopSending = true;
    stopReceiving = true;
    
    if (pthread_join(senderThread, 0) == 0) {
        LOG_INFO_S << "Joined sender thread";
    }
    
    if (pthread_join(receiverThread, 0) == 0) {
        LOG_INFO_S << "Joined receiver thread";
    }
    
    networkAdapter->closeConns();
}

frame_id Communication::nextFrameId()
{
    resendContainersMutex.lock();
    frame_id next_frame_id = lastFrameID;
    do {
        ++next_frame_id;
    }
    while ( resendContainers.find(next_frame_id) != resendContainers.end());
   
    lastFrameID = next_frame_id;
    resendContainersMutex.unlock();
    
    return next_frame_id;
}

// PRIVATE
void Communication::calculateRTT(frame_id frame_id_) {
     // Calculate the round trip time.
    rttInfoMutex.lock();
    if (rttInfo.frame == frame_id_) {
        // Do not use resent frames for the rtt calculations.
        if(resentFramesId.find(frame_id_) == resentFramesId.end()) {
            LOG_DEBUG_S << "RTT of frame " << frame_id_ << " is " << 
                    rttInfo.whenSent.getPassedTime_millisec() << "ms";
            rttInfo.calculateRTTs();
            // Updates new rtt informations within the communication statistics.
            comInfoMutex.lock();
            mComInfo.roundTripTime = rttInfo.roundTripTime;
            mComInfo.roundTripTimeSmoothed = rttInfo.roundTripTimeSmoothed;
            mComInfo.roundTripTimeDeviation = rttInfo.roundTripTimeDeviation;
            comInfoMutex.unlock();
        }
        rttInfo.frame = 0; // Next ack packet will be uesed for a recalculation.
    }
    rttInfoMutex.unlock();
}

void Communication::registerAckToResendContainers(frame_id frame_id_, std::string ip_addr)
{
    if(ip_addr.empty()) {
        LOG_WARN_S << "Ack for frame " << frame_id_ << " could not be registered, address is empty";
        return;
    }
    
    resendContainersMutex.lock();
    // Removes the receiver 'ip_addr' from the frame 'frame_id_' (ack received).
    // If ack have been received for all receivers the resend-container will be removed.
    std::map< uint16_t, DataContainer >::iterator it = resendContainers.find(frame_id_);
    if(it != resendContainers.end()) {
        size_t ret = it->second.addrs.erase(ip_addr);
        if(ret > 0) {
            comInfoMutex.lock();
            mComInfo.numAcksRegistered++; // for FRAME_ACKs
            comInfoMutex.unlock();
            LOG_DEBUG_S << "Register ack from address " << ip_addr << " for frame " << frame_id_;
        }
        
        if(it->second.addrs.size() == 0) {
            resendContainers.erase(it);
            resentFramesId.erase(frame_id_);
            LOG_DEBUG_S << "All ACKs have been received for frame " << frame_id_ << 
                    ", frame deleted";
        }
    } else {
        LOG_ERROR_S << "ACK for frame " << frame_id_ << " cannot be registered, " <<
                "frame not available within the resend list" << std::endl;    
        std::stringstream ss;
        ss << "Available frame IDs within the resend list: " << std::endl;
        it = resendContainers.begin();
        
        for(; it != resendContainers.end(); it++) {
            ss << it->second.id << " "; 
        }
        ss << std::endl;
        LOG_WARN_S << ss.str() << std::endl;
    }
    resendContainersMutex.unlock();
}
 
void Communication::prepareDataForResend(DataContainer const& data) {
    resendContainersMutex.lock();
    std::pair<std::map< frame_id, DataContainer >::iterator, bool> ret;
    // If the packet is already available the rtt time will be refreshed.
    ret = resendContainers.insert(std::pair <frame_id, DataContainer>(data.id, data));
    // If the round trip time is available use this for resend timing.
    rttInfoMutex.lock();
    ret.first->second.resendInfo.resend_after = rttInfo.getResendTimer();
    rttInfoMutex.unlock();
    ret.first->second.resendInfo.remaining_resend_tries = mConfig.num_resend_tries;
    resendContainersMutex.unlock();
}

void Communication::processResendList(float time_passed_ms) {

    resendContainersMutex.lock();
    std::map< frame_id, DataContainer >::iterator it = resendContainers.begin();
    for(; it != resendContainers.end();) {
               
        if(it->second.resendInfo.resend_after > time_passed_ms) { // No resend required 
            it->second.resendInfo.resend_after -= time_passed_ms;
        } else {
            if(it->second.resendInfo.remaining_resend_tries <= 0) { // Delete
                std::stringstream ss;
                ss << "Resend limit reached for frame " << 
                        it->second.id << ", frame discarded. Mssing ACKs: " << std::endl;
                std::set<std::string>::iterator it_addrs = it->second.addrs.begin();
                for(; it_addrs != it->second.addrs.end(); it_addrs++) {
                    ss << *it_addrs << std::endl;
                }
                LOG_INFO_S << ss.str();
                comInfoMutex.lock();
                mComInfo.numUnconfirmedFramesDropped += it->second.addrs.size();
                comInfoMutex.unlock();
                resendContainers.erase(it++);
                resentFramesId.erase(it->second.id);
                continue;
            } else { // Resend
                it->second.resendInfo.remaining_resend_tries--;
                rttInfoMutex.lock();
                it->second.resendInfo.resend_after = rttInfo.getResendTimer();
                rttInfoMutex.unlock();
                
                udp::SendInfo send_info = sendData(it->second); 
                if(send_info.bytes_sent > 0) {
                    comInfoMutex.lock();
                    mComInfo.numFramesRepeated += it->second.addrs.size();
                    comInfoMutex.unlock();
                    LOG_INFO_S << "Frame " << it->second.id << " has been resent (containing " <<
                            it->second.addrs.size() << " receivers)";
                    resentFramesId.insert(it->second.id);
                }
            }
        }
        ++it;
    }
    comInfoMutex.lock();
    mComInfo.resendContainersSize = resendContainers.size();
    comInfoMutex.unlock();
    resendContainersMutex.unlock();
}

void* Communication::dataForwarder()
{
    Timer timer;
    timer.setStartTime();
    float max_size = 0;
    std::map< std::string, PortInfo >::iterator it_info;
    float queue_fullness = 0;
    float passed_time_total_ms = 0;
    
    while (!stopSending) { 
        
        senderQueuesMutex.lock();
        std::map< std::string, std::queue< DataContainer > >::iterator it = senderQueues.begin();
        queue_fullness = 0;
        for(; it != senderQueues.end(); it++) {
            
            // Queue is empty.
            if(it->second.empty()) {
                continue;
            }
            
            max_size = mConfig.default_max_queue_size;
            it_info = portInfos.find(it->first);
            if(it_info != portInfos.end()) {
                max_size = it_info->second.max_queue_size;
            }
            queue_fullness += it->second.size() / max_size;
            
            DataContainer data;
            data = it->second.front();
            it->second.pop();
            comInfoMutex.lock();
            mComInfo.senderQueuesSize--;
            comInfoMutex.unlock();
       
            udp::SendInfo send_info;
            
            // Adds frame to the resend list for the case no ACK will be received.
            // ACK packets has to be added to the resend list before sending to
            // handle very fast ACKs as well.
            // If the id is already available only the rtt time will be refreshed.
            if (data.ackMode == FRAME_ACK) {
                prepareDataForResend(data);
            }
            
            // Adds the control header to the buffer and send the frame to the 
            // specified receivers (addrs).
            send_info = sendData(data);
            
            if(data.addrs.size() != (uint64_t)send_info.frames_sent) {
                LOG_WARN_S << "The frame has been sent only to " << send_info.frames_sent << 
                        " of " << data.addrs.size() << " receivers";
            }
           
            if(send_info.bytes_sent > 0) {
                comInfoMutex.lock();
                mComInfo.numFramesSent += send_info.frames_sent;
                mComInfo.allBytesSend += send_info.bytes_sent;
                comInfoMutex.unlock();
                
                // Prepare round-time-trip calculation using FRAME and FRAGMENT_ACKs.
                if (data.ackMode > NO_ACK && data.ackMode < IS_FRAME_ACK) {
                    // Stores the id of the current ack packet and waits for the ack reply to calculate the rtt (round trip time).
                    // If rtt_limit milliseconds have been passed since the last ack command the current ack will be used instead.
                    rttInfoMutex.lock();
                    if (rttInfo.frame == 0 || rttInfo.whenSent.getPassedTime_millisec() >= mConfig.rtt_limit) {
                        if(rttInfo.frame != 0) {
                            LOG_DEBUG_S << "RTT limit reached (" << rttInfo.whenSent.getPassedTime_millisec() << 
                                    " ms), new frame  " << rttInfo.frame << " is used for RTT calculation now";
                        }
                        LOG_DEBUG_S << "Start rtt calculation with container " << std::endl << data.toString(); 
                        rttInfo.frame = data.id;
                        rttInfo.whenSent.setStartTime();
                    }
                    rttInfoMutex.unlock();
                    comInfoMutex.lock();
                    mComInfo.numAcksRequired += send_info.frames_sent;
                    comInfoMutex.unlock();
                }
            } else {
                // Data could not be sent, remove resend data again.
                if(data.ackMode == FRAME_ACK) {
                    resendContainersMutex.lock();
                    std::map< frame_id, DataContainer >::iterator it = resendContainers.find(data.id);
                    if(it != resendContainers.end()) {
                        resendContainers.erase(it);
                    }
                    resendContainersMutex.unlock();
                }
            }
        }
        
        if(senderQueues.size()) {
            queue_fullness /= (float)senderQueues.size(); // 0.0 to 1.0
        }
        
        senderQueuesMutex.unlock();
        
        double time_passed_ms = timer.getPassedTime_millisec();
        timer.setStartTime();
        processResendList(time_passed_ms);
        
        if(mConfig.print_com_infos_ms > 0) {
            passed_time_total_ms += time_passed_ms;
            if(passed_time_total_ms > mConfig.print_com_infos_ms) {
                comInfoMutex.lock();
                mComInfo.fillNumReceivedPackets(numReceivedPackets);
                std::cout << mConfig.ip_addr << ":" << mConfig.listen_port << std::endl;
                std::cout << mComInfo.toString() << std::endl;
                comInfoMutex.unlock();
                passed_time_total_ms = 0;
            }
        }
        
        // Adapts sender thread sleep depending on queue fullness.
        comInfoMutex.lock();
        mComInfo.senderQueueFullness = queue_fullness;
        comInfoMutex.unlock();
        int sender_thread_usleep = SENDER_THREAD_USLEEP_DEFAULT - 
                (queue_fullness + 0.8) * SENDER_THREAD_USLEEP_DEFAULT;
        if(sender_thread_usleep <= 0)
            sender_thread_usleep = SENDER_THREAD_USLEEP_MINIMUM;
       
        // Sleep value is adapted in forwardData().
        usleep(sender_thread_usleep);
    }
    return nullptr;
}

void* Communication::dataReceiver() {
    int bytes_received = 0;

    while (!stopReceiving) {
        if(getNetworkAdapter()->waitForData(base::Time::fromMilliseconds(100))) {
            // receiveSample returns false when receiving ack
            
            // receiveSample() returns true if a complete ack or data packet has been received.
            DataContainer data;
            bytes_received = 0;
            if(receiveData(data, bytes_received)) {
                std::string sender_addr;
                if(data.addrs.size() > 0) {
                    sender_addr = *(data.addrs.begin());
                } else {
                    LOG_WARN_S << "Received ack packet " << data.id << 
                            " does not contain any sender address";
                }
                
                // Received data packet acknowledged the contained frame id.
                if(data.ackMode > FRAGMENT_ACK && data.ackMode < END_ACK) {
                    comInfoMutex.lock();
                    mComInfo.numAcksReceived++;
                    mComInfo.numAcksRequired--;
                    comInfoMutex.unlock();
                    
                    LOG_DEBUG_S << "Calculates rtt for container " << std::endl << data.toString() << std::endl;
                    // Calculates the round trip time using the received ack.
                    calculateRTT(data.id);
                    
                    if(data.ackMode == IS_FRAME_ACK) {
                        // Removes the ip address from which the ACK has been received.
                        // If all receiver send their ACK the resend container will be removed.
                        registerAckToResendContainers(data.id, sender_addr);
                    }
                } 
                else // Data / port packet has been received.
                {
                    // Fills the receiver queue and discard old elements which exceed 
                    // the limit for the port. A default limit is used if no limit has been defined.
                    std::string port_name = data.getControlContent(PORT_NAME);
                    if(port_name.empty()) {
                        LOG_WARN_S << "Received data packet " << data.id << 
                                " does not contain any port name";
                    } else {
                        receiverQueuesMutex.lock();
                        receiverQueues[port_name].push(data);
                        
                        comInfoMutex.lock();
                        numReceivedPackets[port_name]++;
                        mComInfo.receiverQueuesSize++;
                        mComInfo.numFramesReceived++;
                        comInfoMutex.unlock();
                        
                        std::map< std::string, std::queue< DataContainer > >::iterator it_queue;
                        it_queue = receiverQueues.find(port_name);
                        
                        // Elements will be removed if the max queue size is exceeded.
                        int max_elements = mConfig.default_max_queue_size;
                        std::map< std::string, PortInfo >::iterator it_info;
                        it_info = portInfos.find(port_name);
                        if(it_info != portInfos.end() && it_info->second.max_queue_size > 0) {
                            max_elements = it_info->second.max_queue_size;
                        }
                        int size_diff = (int)it_queue->second.size() - max_elements;
                        if(size_diff > 0) { // Queue is full, remove old elements.
                            LOG_INFO_S << "Receiver: Max queue size of port " << port_name << 
                                    " exceeded, " << size_diff  << " old elements will be removed";
                            for(int i=0; i < size_diff; i++) {
                                it_queue->second.pop(); // Remove oldest elements.
                                comInfoMutex.lock();
                                mComInfo.receiverQueuesSize--;
                                mComInfo.receiverQueuesDiscarded++;
                                comInfoMutex.unlock();
                            }
                        } else { // Queue is not full, note that this packet has been received.
                            receivedPortNamesMutex.lock();
                            receivedPortNames.push_back(port_name);
                            receivedPortNamesMutex.unlock();
                            
                        }
                        receiverQueuesMutex.unlock();
                    }
                }
                                
                // Received frame requests an ack.
                if (data.ackMode == FRAME_ACK && !sender_addr.empty() && mConfig.send_acks) {
                    std::stringstream ss;
                    ss << data.id;
                    const std::string ack_property = ss.str();
                    
                    DataContainer data_ack;
                    data_ack.id = nextFrameId();
                    data_ack.dataInfos.insert(std::make_pair(ACKED_FRAME, ack_property));
                    data_ack.addrs.insert(sender_addr);
                    data_ack.ackMode = IS_FRAME_ACK;
                    
                    if(data_ack.createControlHeader(mConfig.max_fragment_size)) {
                        udp::SendInfo send_info = sendData(data_ack);
                        if(send_info.bytes_sent <= 0) {
                            LOG_WARN_S << "Frame " << data.id << "could not be acknowledged " << 
                                    "(ACK could not be sent)";
                        } else {
                            comInfoMutex.lock();
                            mComInfo.allBytesSend += send_info.bytes_sent;
                            mComInfo.numAcksSend++;
                            comInfoMutex.unlock();
                        }
                    } else {
                        LOG_WARN_S << "Frame " << data.id << "could not be acknowledged " << 
                            "(control header could not be created)";
                    }
                }
            }
            // Count the received bytes independently of the return of receiveData().
            comInfoMutex.lock();
            mComInfo.allBytesReceived += bytes_received;
            comInfoMutex.unlock();
        } 
    }
    return NULL;
}

}
