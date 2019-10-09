#ifndef _UDP_COM_INFOS_HPP_
#define _UDP_COM_INFOS_HPP_

#include<stdint.h>

#include <iomanip>
#include <sstream>

#include <udp/Helpers.hpp>

namespace udp {
    
struct ReceivedPackets {
    ReceivedPackets() : name(), number(0) {
    }
    
    ReceivedPackets(std::string packet_name, uint64_t packet_number, float fps) :
            name(packet_name), number(packet_number), fps(fps) {
    }
    
    std::string name;
    uint64_t number;
    float fps;
};
    
/**
 * Contains overall informations about the communication status.
 */
struct ComInfo {
    ComInfo() : allBytesSend(0),  numFramesReceived(0), 
            numFramesRepeated(0), numFramesSent(0), numAcksReceived(0), numAcksRegistered(0),
            numAcksRequired(0), senderQueuesSize(0), senderQueuesDiscarded(0), resendContainersSize(0), receiverQueuesSize(0), 
            receiverQueuesDiscarded(0), throughput(0.0),
            allBytesReceived(0), numAcksSend(0), numUnprocessedFrames(0), numIncompleteFramesDropped(0), 
            numUnconfirmedFramesDropped(0), numFragmentsSend(0), numFragmentsReceived(0), numFragmentsLost(0), numFragmentsRepeated(0), numResendFrameDropped(0),
            roundTripTime(0), roundTripTimeSmoothed(0), roundTripTimeDeviation(0), senderQueueFullness(0.0) {
                
        startTime.setStartTime();
    }
    
    ~ComInfo() {
    }
    
    uint64_t allBytesSend;
    uint64_t numFramesReceived;
    uint64_t numFramesRepeated;
    uint64_t numFramesSent;
    uint64_t numAcksReceived;          // ACKs != Frames, both are counted independently
    uint64_t numAcksRegistered;
    int64_t numAcksRequired;
    uint64_t senderQueuesSize;
    uint64_t senderQueuesDiscarded;
    uint64_t resendContainersSize;     // Size of resendContainers.
    uint64_t receiverQueuesSize;
    uint64_t receiverQueuesDiscarded;  // If the maximum queue size is reached old packets are discarded.
    
    float throughput; // bytes per second
    
    uint64_t allBytesReceived;
    uint64_t numAcksSend;
    uint64_t numUnprocessedFrames;
    uint64_t numIncompleteFramesDropped;
    uint64_t numUnconfirmedFramesDropped;
    uint64_t numFragmentsSend;
    uint64_t numFragmentsReceived; 
    uint64_t numFragmentsLost;
    uint64_t numFragmentsRepeated;
    uint64_t numResendFrameDropped;     // FRAGMENT-ACK has not been received in time.
    uint64_t resendFramesSize;          // Size of resendFrames (SequencedComm).
    
    float roundTripTime; 
    float roundTripTimeSmoothed;
    float roundTripTimeDeviation;
    
    float senderQueueFullness;
    
    std::vector<ReceivedPackets> numReceivedPackets;
    
    Timer startTime;
    
    std::string toString() {
        std::stringstream ss;
        int setw = 26;
        
        float passed_time_sec = startTime.getPassedTime_sec();
        
        ss << "Frames    send/received:       " << std::setw(setw) << numFramesSent               << " / " << std::setw(setw) << numFramesReceived    << std::endl;
        ss << "Frames    repeated:            " << std::setw(setw) << numFramesRepeated           << std::endl;
        ss << "Frames    unprocessed:         " << std::setw(setw) << numUnprocessedFrames        << std::endl;
        ss << "Frames    incomplete dropped:  " << std::setw(setw) << numIncompleteFramesDropped  << std::endl;
        ss << "Frames    unconfirmed dropped: " << std::setw(setw) << numUnconfirmedFramesDropped << std::endl;
        ss << "Fragments send/received:       " << std::setw(setw) << numFragmentsSend            << " / " << std::setw(setw) << numFragmentsReceived << std::endl;
        ss << "Fragments repeated:            " << std::setw(setw) << numFragmentsRepeated        << std::endl;
        ss << "Fragments lost:                " << std::setw(setw) << numFragmentsLost            << std::endl;
        ss << "Fragment container dropped:    " << std::setw(setw) << numResendFrameDropped       << std::endl;
        ss << "Fragment ACK list size:        " << std::setw(setw) << resendFramesSize            << std::endl;
        ss << "Bytes     send/received:       " << std::setw(setw) << allBytesSend                << " / " << std::setw(setw) << allBytesReceived     << std::endl;
        ss << "MB/s      send/received:       " << std::setw(setw) << (allBytesSend / (1000000.0)) / passed_time_sec <<
                " / " << std::setw(setw) << (allBytesReceived / (1000000.0)) / passed_time_sec << std::endl;
        ss << "ACKs      send/received:       " << std::setw(setw) << numAcksSend                 << " / " << std::setw(setw) << numAcksReceived      << std::endl;
        ss << "ACKs      required:            " << std::setw(setw) << numAcksRequired             << std::endl;    
        ss << "ACKs      registered:          " << std::setw(setw) << numAcksRegistered           << std::endl;  
        ss << "Send      queues size:         " << std::setw(setw) << senderQueuesSize            << std::endl;
        ss << "Send      queues discarded:    " << std::setw(setw) << senderQueuesDiscarded       << std::endl;
        ss << "Send      queues fullness:     " << std::setw(setw) << senderQueueFullness         << std::endl;
        ss << "Frame ACK list size:           " << std::setw(setw) << resendContainersSize        << std::endl;
        ss << "Receiver  queues size:         " << std::setw(setw) << receiverQueuesSize          << std::endl;
        ss << "Receiver  queues discarded:    " << std::setw(setw) << receiverQueuesDiscarded     << std::endl;
        ss << "Send bytes per second:         " << std::setw(setw) << throughput                  << std::endl;
        ss << "Round Trip Time:               " << std::setw(setw) << roundTripTime               << std::endl;
        ss << "Round trip Time Smoothed:      " << std::setw(setw) << roundTripTimeSmoothed       << std::endl;
        ss << "Round Trip Time Deviation:     " << std::setw(setw) << roundTripTimeDeviation      << std::endl;
        
        ss << "Received packets (Time: " << passed_time_sec << "s):" << std::endl;
        std::vector<ReceivedPackets>::iterator it = numReceivedPackets.begin();
        for(; it != numReceivedPackets.end(); it++) {
            ss << std::setw(setw) << it->name << ": " << std::setw(setw) << it->number << 
                    " -> fps: " << std::setw(setw)<< it->number / passed_time_sec << std::endl;
        }
        return ss.str();
    }
    
    void fillNumReceivedPackets(std::map<std::string, uint64_t>& num_received_packets) {
        numReceivedPackets.clear();
        float passed_time_sec = startTime.getPassedTime_sec();
        std::map<std::string, uint64_t>::iterator it = num_received_packets.begin();
        for(; it != num_received_packets.end(); it++) {
            numReceivedPackets.push_back(ReceivedPackets(it->first, 
                                                         it->second, 
                                                         it->second / passed_time_sec));
        }
    }
};
 
}; // end namespace udp

#endif