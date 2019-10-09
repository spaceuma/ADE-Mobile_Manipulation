#ifndef CONTAINERS_HPP
#define CONTAINERS_HPP

#include <math.h>
#include <string.h>
#include <time.h>

#include <set>
#include <string>
#include <iostream>
//#include <boost/concept_check.hpp>

#include <udp/Helpers.hpp>
#include <udp/Frame.hpp>

namespace udp
{ 
    
# define SMOOTH 0.125
# define RTT_RESEND_FACTOR 50    
    
struct RttInfo {
    RttInfo() : whenSent(), frame(0), roundTripTime(0.0), roundTripTimeSmoothed(0.0), 
            roundTripTimeDeviation(0.0), initialTtlCalculated(false), defaultResendAfter(0.0) {
    }
    
    RttInfo(float default_resend_after) : whenSent(), frame(0), roundTripTime(0.0), roundTripTimeSmoothed(0.0), 
            roundTripTimeDeviation(0.0), initialTtlCalculated(false), defaultResendAfter(default_resend_after) {
    }
    
    /**
     * Stores the start time when a ACK packet has been sent. If the acknowledge
     * is received the passed time is used for the rtt calculation.
     */
    Timer whenSent;
    frame_id frame;
    
    float roundTripTime; 
    float roundTripTimeSmoothed;
    float roundTripTimeDeviation;
    
    bool initialTtlCalculated;
    
    float defaultResendAfter; // Has to be set to a reasonable value != 0.
    /**
     * Returns a timer based on the current rtt time multiplied by RTT_RESEND_FACTOR.
     * If the rtt has not been calculated the config.resend_after value is used.
     * This adapts the waiting time to the load of the connection.
     * */
    float getResendTimer() {
        return initialTtlCalculated ? 
                (roundTripTimeSmoothed + roundTripTimeDeviation) * RTT_RESEND_FACTOR : 
                defaultResendAfter;
    }
    
    void calculateRTTs() {
        if(!initialTtlCalculated) { // initial calculation
            roundTripTime = roundTripTimeSmoothed = whenSent.getPassedTime_millisec();
            initialTtlCalculated = true;
        } else {
            roundTripTime = whenSent.getPassedTime_millisec();
            roundTripTimeSmoothed = (1.0 - SMOOTH) * roundTripTimeSmoothed + SMOOTH * roundTripTime;
            roundTripTimeDeviation = (1.0 - SMOOTH) * roundTripTimeDeviation + SMOOTH * fabs(roundTripTimeSmoothed - roundTripTime);
        }
    }
};

struct SendInfo{
    SendInfo() : bytes_sent(0), frames_sent(0) {
    }
    
    int bytes_sent;
    int frames_sent;
};

struct ResendInfo {
    ResendInfo() : remaining_resend_tries(0), resend_after(0) {
    }
    
    std::string toString() {
        std::stringstream ss;
        ss << "Remaining resend tries: " << remaining_resend_tries <<
                ", Resend after: " << resend_after;
        return ss.str();
    }
    
    int remaining_resend_tries;
    float resend_after;
};

struct DataContainer {
    DataContainer() : dataInfos(), addrs(), buffer(), ackMode(NO_ACK), 
            id(0), headersAdded(false), resendInfo(){
    }
    
    /**
     * If no addresses are added the default receivers defined within the config
     * will be used.
     */
    DataContainer(std::string receiver_port_name, 
                  size_t packet_len, 
                  enum ACK_MODE ack_mode) : 
            dataInfos(), addrs(), buffer(), ackMode(ack_mode), 
            id(0), headersAdded(false), resendInfo() {
                
        setControl(PORT_NAME, receiver_port_name);
        setControl(PACKET_LEN, std::to_string(packet_len));
        buffer.resize(packet_len);
    }
    
    ~DataContainer() {
    }
    
    void addAddress(std::string address) {
        addrs.insert(address);
    }
    
    /**
     * Adds a control field to the data container (PORT_NAME, ACKED_FRAME, PAKET_LEN).
     */
    bool setControl(udp::CONTROL_IDS control_id, std::string content) {
        dataInfos.erase(control_id);
        auto ret = dataInfos.insert(std::make_pair(control_id, content));
        return ret.second;
    }
    
    /**
     * Returns the stored content for the passed control id.
     * If no content has been stored an empty string will be returned.
     */
    std::string getControlContent(udp::CONTROL_IDS control_id) {
        std::map<udp::CONTROL_IDS, std::string>::iterator it = dataInfos.find(control_id);
        if(it != dataInfos.end()) {
            return it->second;
        } else {
            return std::string();
        }
    }
    
    uint16_t getControlLength() const {
        // Both are uint16_t, so size * 4.
        uint16_t c_len = dataInfos.size() * (sizeof(control_data_id) + sizeof(control_field_len));
        std::map<udp::CONTROL_IDS, std::string>::const_iterator it = dataInfos.cbegin();
        for(; it != dataInfos.cend(); ++it) {
            c_len += it->second.size();
        }
        return c_len;
    }
    
    /**
     * Uses the control informations to add the control header to the beginning of 
     * the buffer. If the control informations should be set more than once
     * 'headersAdded' has to be set to true manually.
     */
    bool createControlHeader(int max_fragment_size) {
        if(headersAdded) { // Resend containers already contain the control header.
            return true;
        }
        
        int paket_len = 0;
        int buffer_pos = 0;
        int num_frags = 0;
        
        int64_t control_length = getControlLength();
        int64_t buffer_size = sizeof(FrameInfo) + control_length;
        char buffer_tmp[buffer_size];
        
        std::string  name("Unknown"); // TODO Unknown durch not-set ersetzen?
        std::string port_name = getControlContent(PORT_NAME);
        if(!port_name.empty()) {
            name = port_name;
        }
    
        // check paket size (2^16 sequence numbers)
        auto ctrl_f = dataInfos.find(PACKET_LEN);
        if (ctrl_f != dataInfos.end()) {
            if (max_fragment_size - sizeof(Fragment) <= 0) {
                std::cerr << "Configuration error, max_fragment_size (" << max_fragment_size << 
                        ") is smaller than the size of a fragment (" << sizeof(Fragment) << ")" << std::endl;
                return false;
            }

            try {
                paket_len = std::stoi(ctrl_f->second);
            } catch (std::invalid_argument& e) {
                LOG_WARN_S << "Invalid packet length received";
                return false;
            } catch (std::out_of_range& e) {
                LOG_WARN_S << "Received packet range is out of range";
                return false;
            }

            num_frags = static_cast<int>(ceil((paket_len + buffer_size) / 
                    static_cast<double>((max_fragment_size - sizeof(Fragment)))));
            
            if (num_frags > std::numeric_limits<seq_nr>::max()) {
                std::cerr << "Paket " << name << " is too large!" << std::endl;
                return false;
            }   
        }
        
        // Prepares and appends control header.
        // ack and control len
        FrameInfo f_inf;
        switch(ackMode) {
            case END_ACK:
            case NO_ACK: 
            case IS_FRAME_ACK: 
            case IS_FRAGMENT_ACK: f_inf.ack_required = 0; break;
            case FRAME_ACK:       f_inf.ack_required = 1; break;
            case FRAGMENT_ACK:    f_inf.ack_required = 2; break;
        }
        f_inf.c_len = control_length;
        buffer_tmp[buffer_pos++] = f_inf.ack_required;
        buffer_tmp[buffer_pos++] = f_inf.c_len & 0xff;
        buffer_tmp[buffer_pos++] = f_inf.c_len >> 8;

        // controls
        for (auto ctrl_it = dataInfos.begin(); ctrl_it != dataInfos.end(); ctrl_it++) {
            control_field_len ctrl_field_len = ctrl_it->second.size();
            buffer_tmp[buffer_pos++] = ctrl_it->first & 0xff;
            buffer_tmp[buffer_pos++] = ctrl_it->first >> 8;
            buffer_tmp[buffer_pos++] = ctrl_field_len & 0xff;
            buffer_tmp[buffer_pos++] = ctrl_field_len >> 8;
            strcpy(&buffer_tmp[buffer_pos], ctrl_it->second.c_str());
            buffer_pos += ctrl_field_len;
        }

        // Appends the control informations to the data.
        buffer.insert(buffer.begin(), buffer_tmp, buffer_tmp + buffer_size);
        headersAdded = true;
        return true;
    }
    
    void markAsResendPacket() {
        //needs_ack = false;
        ackMode = NO_ACK;
    }
    
    std::string toString() {
        std::stringstream ss;
        ss << "ID: " << id << 
            ", ACK: " << ACK_MODE_NAMES[ackMode] <<
            ", Buffer size: " << buffer.size() << ", " <<
            resendInfo.toString();
        return ss.str();
    }
    
    // Has to be set by the user.
    /**
     * Sending: 
     * Marks the container either as a data packet using PORT_NAME and PACKET_LEN
     * or as a acknowledge packet (ACKED_FRAME).
     * 
     * Receiving:
     * Just contains the port name (PORT_NAME) from which the data has been received.
     */
    std::map<udp::CONTROL_IDS, std::string> dataInfos;
    /**
     * Contains either the receiver addresses or the address from which the
     * packet has been received.
     */
    std::set<std::string> addrs;
    std::vector<uint8_t> buffer;
    enum ACK_MODE ackMode;
    
    // Parameters are set automatically.
    // Unique frame id which will be assigned to all data fragments.
    // If this is an ACK (IS_..._ACK) container the id has to be set to the acknowledged id.
    frame_id id;           
    bool headersAdded;      // Does the buffer contains all required sending headers.
    ResendInfo resendInfo;  // remaining_resend_tries, until_resent;
};

} // end namespace udp

#endif
