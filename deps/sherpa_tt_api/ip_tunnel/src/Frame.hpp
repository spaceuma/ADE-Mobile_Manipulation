#ifndef FRAME_HPP
#define FRAME_HPP

#include <queue>
#include <limits>
#include <map>
#include <vector>
#include <string>

#include <stdint.h>
#include <stdlib.h>

#include<base-logging/Logging.hpp>

#include <udp/Enumerations.hpp>

namespace udp
{

typedef uint16_t frame_id;
typedef uint16_t seq_nr;
typedef uint16_t control_len;
typedef uint16_t control_field_len;
typedef uint16_t control_data_id;

struct FrameInfo{
    FrameInfo() : ack_required(0), c_len(0) {
    }
    
    uint8_t ack_required;
    control_len c_len;
} __attribute__((packed));

struct Fragment{
    Fragment() : id(0), frag_nr(0), next_frag(0) {
    }
    
    frame_id id;							// every fragment belogs to a frame
    seq_nr frag_nr;							// 0 -> first frame
    seq_nr next_frag;						// 0 -> no next frame
};

struct SequenceInfo {
    SequenceInfo() : buffer(), seq_complete(), ttl(0.0), foundEnd(0), end(0), 
            nrReceivedFragments(0), ackMode(NO_ACK), resendFragmentsTimer(0.0),
            numResendFragmentsTries(0) {
    }
    
    std::vector< uint8_t > buffer;
    std::vector< seq_nr > seq_complete;	// maps next_frame to actual seq_nr
    // walk through map -> if you reach seq_nr=0
    // then frame is complete
    float ttl;
    bool foundEnd;
    seq_nr end;
    seq_nr nrReceivedFragments;
    
    // Required for fragment resending (FRAGMENT_ACK).
    enum ACK_MODE ackMode;
    
    std::string senderAddr; // Ip address of the sender.
    
    /**
     * If this timer is set it will be reduced in updateUnprocessedFrames() and if it
     * expired the missing fragments will be requested.
     */
    float resendFragmentsTimer; 
    
    /**
     * The missing fragments are requested mConfig.num_resend_tries times.
     */
    int numResendFragmentsTries;
    
    /*
     * Returns the missing fragments of the current frame if foundEnd is true.
     * Otherwise false will be returned.
     */
    bool getMissingFragments(std::vector<seq_nr>& missing_fragments) {
        if(!foundEnd) {
            return false;
        }
        std::vector< seq_nr >::iterator it = seq_complete.begin();
        for(seq_nr i=0; i < end && it != seq_complete.end(); it++, i++) {
            if(*it == std::numeric_limits<uint16_t>::max()) {
                missing_fragments.push_back(i);
            }
        }
        return true;
    }
};

//typedef std::map< int, SequenceInfo > seq_map;	// map of different incomplete frames

}

#endif