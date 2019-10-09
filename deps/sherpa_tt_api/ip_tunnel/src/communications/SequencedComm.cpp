#include <udp/communications/SequencedComm.hpp>

#include <errno.h>
#include <time.h>

#include <algorithm>
#include <limits>
#include <string>

#include <boost/lexical_cast.hpp>

namespace udp
{

// PUBLIC    
SequencedComm::SequencedComm(const Config& config) : Communication(config)
{  
    stopUpdatingUnprocessedFrames = false;
    
    if (pthread_create(&unprocessedFramesThread, 0, updateUnprocessedFramesStatic, this) == 0)
        LOG_INFO_S << "Successfully created updateUnprocessedFrames thread for " << this;
    else {
        LOG_INFO_S << "Could not create updateUnprocessedFrames thread for " << this;
        throw std::runtime_error("Could not create updateUnprocessedFrames thread");
    }
    
    assert(mConfig.max_fragment_size > (int)sizeof(Fragment));
}

SequencedComm::~SequencedComm()
{
    stopUpdatingUnprocessedFrames = true;
    
    if (pthread_join(unprocessedFramesThread, 0) == 0) {
        LOG_INFO_S << "Joined updateUnprocessedFrames thread";
    }
}

// PROTECTED
SendInfo SequencedComm::sendData(DataContainer &data)
{    
    SendInfo send_info;
    send_info.bytes_sent = 0;
    
    // Send the data to all defined receivers.
    for (auto rem_addresses_it = data.addrs.begin(); rem_addresses_it != data.addrs.end(); rem_addresses_it++) {
        // TODO This requires a separate fragmentation for each receiver. Is there a better way?
        int ret = fragmentAndSendData(*rem_addresses_it, data);
        send_info.bytes_sent += ret;
        if(ret > 0) {
            send_info.frames_sent++;
        }
    }

    return send_info;
}


bool SequencedComm::receiveData(struct DataContainer& data, int& bytes_received)
{
    std::string sender_addr;
    std::vector<uint8_t> buffer;
    int bytes_received_tmp = 0;
    
    // TODO Move networkAdapter->receiveData to Communication and just call 
    // receiveSingleSample here?
    bytes_received_tmp = networkAdapter->receiveData(sender_addr, buffer, 
            networkAdapter->conf.max_fragment_size);
    
    if (bytes_received_tmp > 0 ) {
        bytes_received += bytes_received_tmp; 
        if(receiveSingleSample(bytes_received, sender_addr, buffer, data)) {
            return true;
        }
    }

    return false;
}

// PRIVATE
size_t SequencedComm::fragmentAndSendData(std::string receiver, DataContainer& data) {

    char buffer[mConfig.max_fragment_size];
    uint16_t frag_nr = 0;
    uint64_t data_size = data.buffer.size();
    
    // The received data contains the control-header + the data type.
    // Either the control-header + data type or a max packet is send.
    // mConfig.max_fragment_size > (int)sizeof(Fragment) (assert constructor)
    uint64_t max_data_size_per_fragment = mConfig.max_fragment_size - (int)sizeof(Fragment);
    int sendlen = (int)std::min(data_size, max_data_size_per_fragment);
    
    uint64_t data_pos = 0;
    int bytes_sent = 0;
    
    // If ACK_FRAGMENT is used all fragments of the current frame will be stored for resending.
    std::pair<std::map < uint32_t, ResendFrame>::iterator, bool> resend_frame;
    bool store_fragments = false;
    if(data.ackMode == FRAGMENT_ACK) {
        resendFramesMutex.lock();
        resend_frame = resendFrames.insert(std::pair<uint32_t, ResendFrame>(data.id, 
                ResendFrame(data.id, receiver, mConfig.timeout_after)));
        store_fragments = resend_frame.second;
    }

    while (data_pos < data_size) {
        char *buffer_pos = buffer;

        // data[0] & data[1] -> frame id
        *buffer_pos++ = data.id & 0xff;
        *buffer_pos++ = data.id >> 8;

        // fill in fragment nr
        uint8_t fnr[sizeof(uint16_t)] = { (uint8_t)(frag_nr & 0xff), (uint8_t)(frag_nr >> 8) };
        *buffer_pos++ = fnr[0];
        *buffer_pos++ = fnr[1];

        seq_nr next_fragment = 0;
        // next fragment
        if (((data_size - 1) - data_pos) > max_data_size_per_fragment) {
            next_fragment = frag_nr + 1;
        }

        uint8_t fnext[sizeof(uint16_t)] = { (uint8_t)(next_fragment & 0xff), (uint8_t)(next_fragment >> 8) };
        *buffer_pos++ = fnext[0];
        *buffer_pos++ = fnext[1];

        // Copy the current data segment to the buffer.
        std::copy(data.buffer.begin() + data_pos, data.buffer.begin() + data_pos + sendlen, buffer_pos);
        
        errno = 0;
        int actBytesSent = networkAdapter->sendData(buffer, sendlen + sizeof(Fragment), receiver);
        
        // TODO Why?
        usleep(100);

        bool error_send = false;
        if (actBytesSent < 0) {
            LOG_ERROR_S << "Send failed (actByteSent < 0), errno " << errno;
            error_send = true;
        }
        if (actBytesSent == 0) {
            LOG_ERROR_S << "Could not send data (actByteSent == 0)";
            error_send = true;
        }
        
        if(error_send) {
            // Sending failed, remove the resend data again if available.
            if(data.ackMode == FRAGMENT_ACK) {
                if(store_fragments) {
                    resendFrames.erase(resend_frame.first);
                }
                resendFramesMutex.unlock();
            }
            return 0;
        }
        
        // Store sent fragment bytes for resending if FRAGMENT_ACK is used.
        if(store_fragments) {
            resend_frame.first->second.addFragmentData(frag_nr, buffer, actBytesSent);
        }

        // If the fragment header is not sent completely the data packet is not valid
        // and will be sent again. Otherwise the data position is increased by the number
        // of sent bytes not regarding the fragment header.
        if (actBytesSent >= (int)sizeof(Fragment)) {
            data_pos += actBytesSent - sizeof(Fragment);
            comInfoMutex.lock();
            mComInfo.numFragmentsSend++;
            comInfoMutex.unlock();
        }

        sendlen = std::min((data_size - data_pos), max_data_size_per_fragment);
        frag_nr = next_fragment;
        bytes_sent += actBytesSent;
    }
    
    if(data.ackMode == FRAGMENT_ACK) {
        resendFramesMutex.unlock();
    }
    return bytes_sent;
    
} 

bool SequencedComm::receiveSingleSample(size_t bytes_received, 
                                        const std::string& sender_addr, 
                                        const std::vector<uint8_t> &buffer, 
                                        DataContainer &data)
{
    std::map< int, SequenceInfo >::iterator sequenceIt;
    
    unprocessedFramesMutex.lock();
    
    // Only returns true if all fragments have been found.
    if(!registerDataAtSample(buffer, sender_addr, sequenceIt)) {
        unprocessedFramesMutex.unlock();
        return false;
    }
    
    const SequenceInfo &seqInfo(sequenceIt->second);
    
    // Check buffer size.
    bool error = seqInfo.buffer.size() < sizeof(FrameInfo);
    if(error) {
        LOG_ERROR_S << "Size of received frame is smaller than the size of a fragment header.";
    }
    
    // Check frame info.
    // Copies first part of the control header (ack and control data length)
    //const FrameInfo *frame_info = reinterpret_cast<const FrameInfo *>(seqInfo.buffer.data());
    FrameInfo frame_info;
    if(!error) {
        memcpy(&frame_info, seqInfo.buffer.data(), sizeof(FrameInfo));
        error = frame_info.c_len > seqInfo.buffer.size();
        if(error) {
            LOG_ERROR_S << "Received length of the control informations (" << frame_info.c_len << 
                    ") exceeds buffer size (" << seqInfo.buffer.size() << ")";
            std::stringstream ss;
            std::vector<uint8_t>::const_iterator it = seqInfo.buffer.cbegin();
            for(; it != seqInfo.buffer.cend(); it++) {
                ss << (char)*it << "(" << (int)*it << ") ";
            }
            ss << std::endl;
            LOG_WARN_S << ss.str();
        }
    }
    
    if(error) {
        deleteUnprocessedFrame(sequenceIt);
        unprocessedFramesMutex.unlock();
        return false;
    }

    // Extract control header.
    std::vector<uint8_t> control_data_buffer;
    // assign just control data
    control_data_buffer.assign(seqInfo.buffer.begin() + sizeof(FrameInfo), seqInfo.buffer.begin() + 
            sizeof(FrameInfo) + frame_info.c_len);
    
    ExtractControlInformations ctrl_infos(control_data_buffer);
    
    // Start to fill the sample data.
    data.id = sequenceIt->first;
    data.buffer.assign(seqInfo.buffer.begin() + sizeof(FrameInfo) + frame_info.c_len, seqInfo.buffer.end());
    data.addrs.insert(sender_addr);

    switch(frame_info.ack_required) {
        case 0: data.ackMode = NO_ACK; break;
        case 1: data.ackMode = FRAME_ACK; break;
        case 2: data.ackMode = FRAGMENT_ACK; break;
        default: {
            data.ackMode = NO_ACK;
            LOG_WARN_S << "Received sample contains an invalid ack number: " << frame_info.ack_required;
        }
    }

    // Check whether this is an ack packet which confirms the receiving of an 
    // previous sent packet.
    std::string ack_str = ctrl_infos.getControlInformation(ACKED_FRAME);
    if (!ack_str.empty()) {
        bool ret = true;
        try {
            frame_id acked_frame_id = std::stoi(ack_str.c_str());
            data.ackMode = IS_FRAME_ACK;
            // Sets the data id to the id of the acknowledged frame.
            data.id = acked_frame_id;
        } catch (std::invalid_argument& e) {
            LOG_WARN_S << "Invalid packet length received";
            ret = false;
        } catch (std::out_of_range& e) {
            LOG_WARN_S << "Received packet range is out of range";
            ret = false;
        }
        deleteUnprocessedFrame(sequenceIt);
        unprocessedFramesMutex.unlock();
        return ret; // Return ACK frame.
    }
    
    // 
    std::string ack_fragments_str = ctrl_infos.getControlInformation(MISSING_FRAGMENTS);
    if (!ack_fragments_str.empty()) {
        // Either sends the missing fragments (returns > 0) or received an ACK that 
        // all fragments have been received (returns 0).
        frame_id ack_frame_id = 0;
        int ret = processMissingFragmentsString(ack_fragments_str, ack_frame_id);
        deleteUnprocessedFrame(sequenceIt);
        unprocessedFramesMutex.unlock();
        if(ret == 0) { // All fragments have been received, convert data packet to an IS_ACK packet.
            data.ackMode = IS_FRAGMENT_ACK;
            // Sets the data id to the id of the acknowledged frame.
            data.id = ack_frame_id; 
            return true; // Received an IS_FRAGMENT_ACK which should be used to calculate the rtt.
        } else {
            return false; // Message already has been handled completely (resend fragments).
        }
    }
    
    // Port name available?
    std::string port_name = ctrl_infos.getControlInformation(PORT_NAME);
    if (port_name.empty()) {
        LOG_ERROR_S << "Paket contains no port name";
        deleteUnprocessedFrame(sequenceIt);
        unprocessedFramesMutex.unlock();
        return false;
    }
    // Adds the port name to the sample data.
    data.setControl(PORT_NAME, port_name);
    
    // Sample data is completely filled, delete the unprocessed frame and unlock its mutex.
    deleteUnprocessedFrame(sequenceIt);
    unprocessedFramesMutex.unlock();

    // Check length informations.
    std::string packet_len_str = ctrl_infos.getControlInformation(PACKET_LEN);
    if (!packet_len_str.empty()) {
        uint64_t paket_len_should;
        try {
            paket_len_should = std::stoi(packet_len_str);
            
            //boost::lexical_cast<int>(packet_len_str);
        //} catch (boost::bad_lexical_cast const &) {
        } catch (std::invalid_argument& e) {
            LOG_WARN_S << "Invalid packet length received";
            return false;
        } catch (std::out_of_range& e) {
            LOG_WARN_S << "Received packet range is out of range";
            return false;
        }

        if (data.buffer.size() != paket_len_should) {
            LOG_WARN_S << "The packet should contain " << paket_len_should << " bytes, but " << 
                    data.buffer.size() << " bytes have been received.";
            LOG_WARN_S << "Buffer size: " <<  data.buffer.size() << 
                    ", control header length: " << frame_info.c_len << std::endl;
            return false;
        }
    } else {
        LOG_ERROR_S << "Paket contains no length information";
        return false;
    }
 
    return true;
}

bool SequencedComm::registerDataAtSample(const std::vector<uint8_t> &buffer, 
                                         const std::string& sender_addr,
                                         std::map< int, SequenceInfo >::iterator &fragIt)
{
    const size_t bytes_received = buffer.size();
    
    // sizeof(Fragment)+1 because we need the ack byte of the control header of
    // the first fragment as well (for fragment resending)
    if (bytes_received < 0 
        || bytes_received == 0
        || bytes_received <= sizeof(Fragment)+1) { 
        return false;
    }

    const Fragment *frag = reinterpret_cast<const Fragment *>(buffer.data());
    
    // Check fragment numbers.
    if(frag->next_frag != 0 && frag->frag_nr+1 != frag->next_frag) {
        LOG_WARN_S << "Received an erroneous fragment header (id: " << frag->id << 
                ", frag nr: " << frag->frag_nr << ", next frag nr: " << frag->next_frag;
        return false;
    }
    
    // Did we already received this frame completely? In this case these doubled
    // fragments can be ignored.
    blockFrameIDMutex.lock();
    std::map< int, float>::iterator it_blocked = blockFrameID.find(frag->id);
    if(it_blocked != blockFrameID.end()) {
        LOG_DEBUG_S << "Fragment receiving (" << frag->frag_nr << ") for frame " << 
                frag->id << " is blocked.";
        blockFrameIDMutex.unlock();
        return false;
    }
    blockFrameIDMutex.unlock();
    
    // Create new unprocessed frame to store all fragments.
    fragIt = unprocessedFrames.find(frag->id);
    if (fragIt == unprocessedFrames.end()) {
        LOG_DEBUG_S << "New unprocessed frame created for frame " << frag->id;
        SequenceInfo seq_info;
        // set ttl to initial value
        seq_info.ttl = mConfig.timeout_after;
        seq_info.end = 0;
        seq_info.nrReceivedFragments = 0;
        seq_info.foundEnd = false;    
        seq_info.senderAddr = sender_addr; // Adds sender address (required for fragment resending).
        // Creates an array with max uint16 elements and assigns max uint16 to each.
        seq_info.seq_complete.resize(std::numeric_limits<uint16_t>::max(), 
                                     std::numeric_limits<uint16_t>::max());
        fragIt = unprocessedFrames.insert(std::make_pair(frag->id, seq_info)).first;
        comInfoMutex.lock();
        mComInfo.numUnprocessedFrames++;
        comInfoMutex.unlock();
    }

    SequenceInfo& seq_info(fragIt->second);
    
    // Resend frames use the original frame id. So, receiving a doubled fragment
    // should mean that we have received a resend frame. To avoid deleting the
    // unprocessed frame in the updateUnprocessedFrames thread, its ttl is refreshed.
    if(seq_info.seq_complete[frag->frag_nr] != std::numeric_limits<uint16_t>::max())
    {
        //fragment already received
        LOG_DEBUG_S << "Received doubled fragment " << frag->frag_nr<< " for frame " << 
                frag->id << std::endl;
        seq_info.ttl = mConfig.timeout_after;
        return false;
    }
    
    // If this fragment is the first fragment store the ack byte as well 
    // (required for fragment resending).
    if(frag->frag_nr == 0) {
        uint8_t ack_byte = buffer[6];
        if(ack_byte >= (uint8_t)END_ACK) {
            LOG_WARN_S << "Frame " << frag->id << " contains an invalid ack byte: " <<
                    (int)ack_byte << std::endl;
        } else {
            seq_info.ackMode = (enum ACK_MODE)ack_byte; // frame_id(2) frag_nr(2) frag_next(2) ack(1)
            LOG_DEBUG_S << "First fragment of frame " << frag->id << 
                    "found, ACK Byte extracted: " << ack_byte;
        }
    }
   
    const uint8_t* data_ptr = buffer.data() + sizeof(Fragment);
    const size_t data_size = buffer.size() - sizeof(Fragment);
    
    // Use one vector buffer for incomplete frames.
    size_t frame_buffer_pos = (frag->frag_nr) * (mConfig.max_fragment_size - sizeof(Fragment));
    if (seq_info.buffer.size() <= frame_buffer_pos)
        seq_info.buffer.resize(frame_buffer_pos + data_size);
    
    seq_info.seq_complete[frag->frag_nr] = frag->next_frag;
    
    seq_info.nrReceivedFragments++;
    comInfoMutex.lock();
    mComInfo.numFragmentsReceived++;
    comInfoMutex.unlock();
    
    memcpy(&(seq_info.buffer[frame_buffer_pos]), data_ptr, data_size);
    
    // Last fragment. 
    // If fragment resending is requested for this frame, founding the last element
    // will activate the timer after which the missing fragments are requested
    // (see updateUnprocessedFrames()).
    if(frag->next_frag == 0) {
        LOG_DEBUG_S << "Last fragment found for frame " << frag->id;
        seq_info.foundEnd = true;
        seq_info.end = frag->frag_nr;
        if(seq_info.ackMode == FRAGMENT_ACK) {
            rttInfoMutex.lock();
            seq_info.resendFragmentsTimer = rttInfo.getResendTimer();
            rttInfoMutex.unlock();
            seq_info.numResendFragmentsTries = mConfig.num_resend_tries;
        }
    }
    
    // Check if we got all fragments.
    if(seq_info.foundEnd && (seq_info.nrReceivedFragments > seq_info.end)) {
        LOG_DEBUG_S << "Found all fragments of frame " << frag->id;
        // If a FRAGMENT_ACK is requested, the receiving of the complete frame is 
        // acknowledged by sending an empty MISSING_FRAGMENTS message.
        // This tells the sender that it can delete the fragments which it has stored 
        // for resending.
        if(seq_info.ackMode == FRAGMENT_ACK) {
            std::vector<seq_nr> empty_vec;
            requestMissingFragments(frag->id, sender_addr, empty_vec);
        }
        
        // Blocks frame id so the following doubled fragments (if any) are ignored
        // and do not create a new unprocessed-frames entry.
        blockFrameIDMutex.lock();
        blockFrameID.insert(std::pair<int, float>(frag->id, mConfig.timeout_after));
        blockFrameIDMutex.unlock();
        LOG_DEBUG_S << "Add block for frame " << frag->id;
        
        return true;
    }

    return false;    
}

void SequencedComm::deleteUnprocessedFrame(std::map< int, SequenceInfo >::iterator it) {
    unprocessedFrames.erase(it);
    comInfoMutex.lock();
    mComInfo.numUnprocessedFrames--;
    comInfoMutex.unlock();
}

bool SequencedComm::requestMissingFragments(frame_id frame_id, std::string sender_addr, 
        std::vector<seq_nr>& missing_fragments) {
    
    if(!mConfig.send_acks) {
        return false;
    }
    
    // Build up missing fragments string containg <frame_id> <frag_id_1> <frag_id_2> ...
    std::stringstream ss;
    ss << frame_id << " ";
    std::vector<seq_nr>::iterator it = missing_fragments.begin();
    for(; it != missing_fragments.end(); it++) {
        ss << *it << " ";
    }
  
    DataContainer data;
    data.id = nextFrameId();
    data.setControl(MISSING_FRAGMENTS, ss.str());
    data.addrs.insert(sender_addr);
    
    if(!data.createControlHeader(mConfig.max_fragment_size)) {
        LOG_WARN_S << "Request missing fragments: control header could not be created" << std::endl;
        return false;
    }
       
    udp::SendInfo send_info = sendData(data);
    if(send_info.bytes_sent <= 0) {
        LOG_WARN_S << "Missing fragments for frame " << frame_id << " could not be requested";
        return false;
    } else {
        if(missing_fragments.size() == 0) {
            LOG_DEBUG_S << "Received a fragment ACK for frame " << frame_id;
            comInfoMutex.lock();
            mComInfo.numAcksSend++;
            comInfoMutex.unlock();
        } else {
            LOG_DEBUG_S << "Request " << missing_fragments.size() << 
                    " missing fragments for frame " << frame_id << std::endl;
        }
    }
    //sendDataMutex.unlock();
    comInfoMutex.lock();
    mComInfo.allBytesSend += send_info.bytes_sent;
    comInfoMutex.unlock();
    
    return true;
}

int SequencedComm::processMissingFragmentsString(std::string ack_fragments_str, 
                                                 frame_id& ack_frame_id) {
    if(ack_fragments_str.empty()) {
        return false;
    }
    
    // Split string into <frame_id> <missing_frag_id1> <missing_frag_id2> .. 
    std::vector<std::string> strings = split(ack_fragments_str, ' ');
    int counter_resend_fragments = 0;
    
    try {
        // Extract frame id.
        std::vector<std::string>::iterator it_strings = strings.begin();
        ack_frame_id = std::stoi(*it_strings++);
        resendFramesMutex.lock();
        std::map < uint32_t, ResendFrame>::iterator it_frame = resendFrames.find(ack_frame_id);
        
        // frame_id not part of the resend list.
        if(it_frame == resendFrames.end()) {
            LOG_WARN_S << "Fragment resending failed, requested frame id " << ack_frame_id << 
                    " is not available in the resendFrames list";
            resendFramesMutex.unlock();
            return -1;
        }
        
        // The received string only contains the frame id. This means
        // that all fragments have been received successfully and the fragment resend data
        // can be deleted.
        if(strings.size() == 1) {
            size_t ret = resendFrames.erase(ack_frame_id);
            if(ret > 0) {
                comInfoMutex.lock();
                mComInfo.numAcksRegistered++;  // for FRAGMENT_ACKs
                comInfoMutex.unlock();
            }
            resendFramesMutex.unlock();
            return 0;
        }
        
        // Search and send the missing fragments.
        uint32_t frag_id = 0;
        std::map <uint32_t, std::vector<char> >::iterator it_frag;
        for(; it_strings != strings.end(); it_strings++) {
            frag_id = std::stoi(*it_strings);
            it_frag = it_frame->second.getFragmentData(frag_id); // fragments.find(frag_id);
            if(it_frag == it_frame->second.fragments.end()) {
                LOG_WARN_S << "Fragment id " << frag_id << 
                        " is not available within the fragment list of frame " << ack_frame_id;
                continue;
            }
            int sent_bytes = getNetworkAdapter()->sendData(it_frag->second.data(), 
                    it_frag->second.size(), it_frame->second.receiverIp);
            if(sent_bytes <= 0) {
                LOG_WARN_S << "Fragment id " << frag_id << " of frame " << ack_frame_id << "could not be resent";
            } else {
                comInfoMutex.lock();
                mComInfo.numFragmentsRepeated++;
                mComInfo.numFragmentsSend++;
                mComInfo.allBytesSend += sent_bytes;
                comInfoMutex.unlock();
                LOG_DEBUG_S << "Fragment " << frag_id << " / " << it_frame->second.fragments.size() << 
                        " for frame " << ack_frame_id << " has been resent";
                counter_resend_fragments++;
            }
        }   
    } catch (std::invalid_argument& e) {
        LOG_WARN_S << "Invalid packet length received";
        resendFramesMutex.unlock();
        return -1;
    } catch (std::out_of_range& e) {
        LOG_WARN_S << "Received packet range is out of range";
        resendFramesMutex.unlock();
        return -1;
    }
    resendFramesMutex.unlock();
    // If no fragments have been resent an error will be reported.
    return counter_resend_fragments == 0 ? -1 : counter_resend_fragments;
}

void SequencedComm::updateUnprocessedFrames()
{
    Timer timer;
    timer.setStartTime();
    
    while(!stopUpdatingUnprocessedFrames) {
        
        float time_passed_ms = timer.getPassedTime_millisec();
        timer.setStartTime();
        
        unprocessedFramesMutex.lock();
        // Updates ttl of incomplete frames and deletes old incompleted frames.
        if (unprocessedFrames.size() > 0) {
            std::map< int, SequenceInfo >::iterator it = unprocessedFrames.begin();
            // Requests missing fragments if requested and deletes old incomplete frames.
            while (it != unprocessedFrames.end()) {

                it->second.ttl -= time_passed_ms;
                if(it->second.ackMode == FRAGMENT_ACK && it->second.resendFragmentsTimer > 0.0) {
                    it->second.resendFragmentsTimer -= time_passed_ms;
                }
                
                // Delete incomplete frame.
                if (it->second.ttl <= 0) {
                    std::vector<seq_nr> missing_fragments;      
                    if(it->second.getMissingFragments(missing_fragments)) {
                        LOG_WARN_S << "Incomplete packet dropped: " << missing_fragments.size() << 
                                " of " << it->second.end + 1 << " fragments are missing";
                                
                        std::stringstream ss;
                        ss << "Missing fragments: ";
                        for(size_t i=0; i<missing_fragments.size(); i++) {
                            ss << missing_fragments[i] << " ";
                        }
                        LOG_INFO_S << ss.str();
                        comInfoMutex.lock();
                        mComInfo.numFragmentsLost += missing_fragments.size();
                        comInfoMutex.unlock();
                    }
                    // Delete frame.
                    unprocessedFrames.erase(it++);
                    comInfoMutex.lock();
                    mComInfo.numIncompleteFramesDropped++;
                    mComInfo.numUnprocessedFrames--;
                    comInfoMutex.unlock();
                } else {
                    // Request missing fragments.
                    if(it->second.ackMode == FRAGMENT_ACK && 
                            it->second.resendFragmentsTimer <= 0.0 && 
                            it->second.numResendFragmentsTries > 0) {
                        std::vector<seq_nr> missing_fragments;
                        it->second.getMissingFragments(missing_fragments);
                        requestMissingFragments(it->first, 
                                                it->second.senderAddr, 
                                                missing_fragments);
                        it->second.numResendFragmentsTries--;
                        rttInfoMutex.lock();
                        it->second.resendFragmentsTimer = rttInfo.getResendTimer();
                        rttInfoMutex.unlock();
                    }
                    it++;
                }
            }
        }
        // Reduces id block time. If expired new unprocessed frame entries with
        // the blocked id are possible.
        blockFrameIDMutex.lock();
        std::map< int, float>::iterator it_blocked = blockFrameID.begin();
        while(it_blocked != blockFrameID.end()) {
            it_blocked->second -= time_passed_ms;
            if(it_blocked->second <= 0) {
                LOG_DEBUG_S << "Remove block for frame " << it_blocked->first;
                blockFrameID.erase(it_blocked++);
            } else {
                it_blocked++;
            }
        }
        blockFrameIDMutex.unlock();
        unprocessedFramesMutex.unlock();
        
        // Delete resendFrames which are older than ttl.
        resendFramesMutex.lock();
        std::map < uint32_t, ResendFrame>::iterator it_resend = resendFrames.begin();
        // TODO Why is empty() required?
        while(/*!resendFrames.empty() &&*/ it_resend != resendFrames.end()) {
            it_resend->second.ttl -= time_passed_ms;
            if(it_resend->second.ttl <= 0) {
                LOG_WARN_S << "No fragment acknowledge received for frame " << 
                        it_resend->first << ", fragment container is deleted";
                comInfoMutex.lock();
                mComInfo.numResendFrameDropped++;
                comInfoMutex.unlock();
                resendFrames.erase(it_resend++);
            } else {
                it_resend++;
            }
        }
        comInfoMutex.lock();
        mComInfo.resendFramesSize = resendFrames.size();
        comInfoMutex.unlock();
        resendFramesMutex.unlock();
        usleep(10000);
    }
}

} // end namespace udp











