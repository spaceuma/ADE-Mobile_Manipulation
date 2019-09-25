#ifndef ENUMERATIONS_HPP
#define ENUMERATIONS_HPP

namespace udp 
{
    
enum CONTROL_IDS {
    PORT_NAME = 1,         // Stores the port name of the data.
    ACKED_FRAME = 2,       // Used to acknowledge a received frame, the control info containes its frame id.
    PACKET_LEN = 3,        // The length of the received data.
    MISSING_FRAGMENTS = 4  // Used to request missing fragments.
                           // The control information format is: <FRAME_ID> <MISSING_FRAGMENT_ID1> <MISSING_FRAGMENT_ID2> ...
                           // Just sending <FRAME_ID> means that all fragments have been received.
}; 

enum ACK_MODE {
    NO_ACK,          // The data packet does not require an acknowledge.
    FRAME_ACK,       // The complete packet should be acknowledged and will be resent if required.
    FRAGMENT_ACK,    // Missing fragments will be reported and resent. This should be used for large packets.
    IS_FRAME_ACK,    // The data packet acknowledges that a frame has been received.
    IS_FRAGMENT_ACK, // The data packages acknowledges that all fragments have been received.
    END_ACK          // Do not add elements after this last element.
};

static std::string ACK_MODE_NAMES[] = {
    "No Ack",
    "Requests Frame Ack",
    "Requests Fragment Ack",
    "Is Frame Ack",
    "Is Fragment Ack",
    "End Ack"
};

} // end namespace udp

#endif