#ifndef SEQUENCEDCOMM_HPP
#define SEQUENCEDCOMM_HPP

#include <vector>

#include <base/Time.hpp>

#include <udp/Communication.hpp>

namespace udp
{
  
/** 
 * Contains all single fragments of a sent frame.
 * This informations are used to be able to resend missing fragments
 * which are requested by the receiver.
 */
struct ResendFrame {
  
    ResendFrame(frame_id frame_id, std::string receiver_ip, float ttl) : 
            frameId(frame_id), receiverIp(receiver_ip), ttl(ttl) {
    }
    
    frame_id frameId;  
    std::string receiverIp;
    // fragment-id -> data
    std::map <uint32_t, std::vector<char> > fragments;
    float ttl;
    
    /**
     * If not already added a new fragment is created using the fragment_id as a key
     * and the fragment data is copied to the new buffer.
     */
    bool addFragmentData(uint32_t fragment_id, char* data, size_t length) {
        std::pair<std::map <uint32_t, std::vector<char> >::iterator, bool> ret; 
        
        // TODO Directly create and add the vector causes seg faults sometimes.
        std::pair<uint32_t, std::vector<char>> frag_pair(fragment_id, std::vector<char>(length));
        ret = fragments.insert(frag_pair);
        if(ret.second) {
            std::copy(data, data+length, ret.first->second.begin());
            return true;
        }
        return false;
    }
    
    /**
     * Returns the fragment data of the passed id or map::end().
     */
    std::map <uint32_t, std::vector<char> >::iterator getFragmentData(uint32_t fragment_id) {
         return fragments.find(fragment_id);
    }
    
 private:
    ResendFrame() : frameId(0), receiverIp(), fragments() {
    }
};

/**
 * Implements the virtual sendData() and receivedata() methods of the Communication class 
 * and adds data fragmentation and fragment resending. Using UDP or TCP the maximum packet 
 * size is 65k Bytes. Therefore this class takes care to split and to reassemble the data 
 * packets. In addition - if FRAGMENT_ACK is used - it takes care to request missing
 * fragments. This and the handling of unprocessed frames happen within an extra thread.
 */
class SequencedComm : public Communication
{
 public:
    SequencedComm(const Config &config = Config());
    virtual ~SequencedComm();
    
 protected: 
    /**
     * Appends the control header to the buffer and sends the fragmented data to
     * all receivers.
     * TODO Control header appending should be done within Communication.
     */
    SendInfo sendData(DataContainer &data);

    /**
     * Fills the data container and the bytes_received parameter if a new complete
     * sample has been received.
     */
    bool receiveData(struct DataContainer& data, int& bytes_received);
     
 private:   
    pthread_t unprocessedFramesThread; 
    bool stopUpdatingUnprocessedFrames;

    std::mutex resendFramesMutex;
    std::mutex unprocessedFramesMutex;
    std::mutex blockFrameIDMutex;
     
    /**
     * Storage for incomplete received frames, e.g frames with missing fragments 
     * using frame-id as key.
     */
    std::map< int, SequenceInfo > unprocessedFrames; 
    
    /**
     * This map stores the formerly deleted unprocessed frames. This is used to prevent
     * that resent fragments fill up the gab of an unprocessed frame (which deletes the
     * frame) and directly create a new entry within the unprocessed frame list.
     * The list maps the frame_id to a timer which prevents a recreation of the
     * unprocessed frame for a while.
     */
    std::map< int, float> blockFrameID;
    
    /**
     * If FRAGMENT_ACK is used this list stores all fragments of all sent frames for 
     * resending.
     */
    std::map < uint32_t, ResendFrame> resendFrames; 
    
    /**
     * Splits the buffer into single data packets and send the packets to 'receiver'. 
     * Each data packet is preceded with the fragment header containing the frame id, 
     * the fragment number and the next fragment number. A next fragment number
     * of 0 means that the last fragment has been found.
     * \param receiver Receiver address.
     * \param data This method adds the frame id to the data container.
     * \return Returns the overall number of sent bytes which includes the
     * fragment headers as well.
     */
    size_t fragmentAndSendData(std::string receiver, DataContainer& data);
    
    /**
     * Checks the received sample and removes the sample from unprocessedFrames.
     * If an ACK is requested it will be sent using sendData() directly.
     * \return Returns only true if a complete sample with the controls PAKET_LEN and PORT_NAME
     * has been received. ACKED_FRAMEs are processed but false is returned.
     */
    bool receiveSingleSample(size_t bytes_received, const std::string& sender_addr, 
            const std::vector<uint8_t> &buffer, DataContainer &data); 
     
    /**
     * Creates an unprocessedFrames entry and collects all fragments.
     * This method prepares the fragment resending which requires the
     * last frame.
     * Method is not thread-safe.
     * \return Only returns true if a complete frame has been received.
     */
    bool registerDataAtSample(const std::vector< uint8_t >& buffer,
                              const std::string& sender_addr,
            std::map< int, SequenceInfo >::iterator &fragIt);
    
    /**
     * Erases the passed unprocessed frame and updates the comInfo statistics.
     */   
    void deleteUnprocessedFrame(std::map< int, SequenceInfo >::iterator it);
    
    static void* updateUnprocessedFramesStatic(void* params) {
        ((SequencedComm*)params)->updateUnprocessedFrames();
        return NULL;
    }
    
    /**
     * Prepares and sends a data container which requests missing frame fragments.
     * The control information format is: 
     * <FRAME_ID> <MISSING_FRAGMENT_ID1> <MISSING_FRAGMENT_ID2> ...
     * If the passed vector is empty the received frame is acknowledged.
     */
    bool requestMissingFragments(frame_id frame_id, 
                                 std::string sender_addr, 
                                 std::vector<seq_nr>& missing_fragments);
    
    /**
     * This method resends missing fragments and deletes the entry on the resendFrames
     * list if all fragments have been received.
     * \param ack_fragments_str The string contains the frame_id followed by missing fragments IDs.
     * \param ack_frame_id This parameter is set to the acknowledged frame id which has been extracted from the past string.
     * \return On an error -1 is returned, otherwise the number of resent fragments.
     * 0 means that no fragments are missing and the frame has been received successfully.
     */
    int processMissingFragmentsString(std::string ack_fragments_str, frame_id& ack_frame_id);
    
    /**
     * Updates time to live for incomplete and resend frames (free frame if ttl <= 0).
     */
    void updateUnprocessedFrames();
};

}

#endif