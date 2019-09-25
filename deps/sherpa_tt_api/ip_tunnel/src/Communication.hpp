#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <udp/Config.hpp>
#include <udp/Frame.hpp>
#include <udp/Helpers.hpp>
#include <udp/ComInfos.hpp>
#include <udp/Containers.hpp>
#include <udp/network_adapters/NetworkAdapter.hpp>

#include <set>
#include <list>
#include <map>
#include <mutex>

namespace udp
{

/**
 * This is the base class to exchange data containers over the network using UDP.
 * To increase reliability for each data container acknowledgements can be requested.
 * If a container has not been acknowledged it will be resent automatically.
 * forwardPacket() adds a container to the sending queue and getPacket() returns 
 * a container from the receiving queue, both using their own thread. <br>
 * <br>
 * To each data container the receiver address (by default config.ip_addr is used),
 * the control informations, the data and the ack mode have to be added, e.g. <br>
 * DataContainer container("motion_out", 24, FRAME_ACK); <br>
 * container.addAddress("192.168.1.42"); <br>
 * comm.forwardData(container); <br>
 * <br>
 * Serialization of the control header: <br>
 * Byte 00  01 02       03 04      05 06    07 07+data_len <br>
 *      ack control_len control_id data_len control_data   ... (further control infos) <br>
 * <br>
 * ack:          0 -> no ack, 1 -> frame ack, 2 -> fragment ack (see SequencedComm) <br>
 * control_len:  Length of the following control data. <br>
 * control_id:   See enum CONTROL_IDS. <br>
 * data_len:     Length of the following control info data. <br>
 * control_data: Control informations. Numbers are coded as strings. <br>
 * <br>
 * See 'doc/ip_tunnel_protocol.pdf' for further details and examples.
 */
class Communication
{
 public:
    static int SENDER_THREAD_USLEEP_DEFAULT;
    static int SENDER_THREAD_USLEEP_MINIMUM;

    /**
     * Generates a frame id, the control header and inserts the data packet into the send queue.
     * The queue is processed in the dataForwarder thread.
     * The data packet has to contain the dataInfos PORT_NAME and PACKET_LEN.
     * the receiver addresses, the data and whether this data should be acknowledged.
     */ 
    bool forwardPacket(DataContainer& data);
       
    /**
     * Returns the next (oldest) sample of the receiver queues. 
     * \param sampleData Container to receive a copy of the sample data.
     * \param If port_name is used data from the named queue is returned, if available.
     * Otherwise the incoming order determines the returned sample.
     * \return True if a sample is available.
     */
    bool getPacket(DataContainer& data, std::string port_name = std::string());
                          
    /**
     * Returns the low level communication methods to exchange byte arrays directly.
     */
    inline NetworkAdapter* getNetworkAdapter() {
        return networkAdapter;
    }
    
    /**
     * Updates numReceivedPackets and returns a copy of the current communication statistics.
     */
    ComInfo getComInfoCopy();
                                    
 protected:    
    struct Config mConfig;
    /**
     * Collects general communication infos.
     */
    ComInfo mComInfo;
    RttInfo rttInfo;
    
    std::mutex comInfoMutex;
    std::mutex rttInfoMutex;
    
    /**
     * Stores how often each packet has been received.
     * ComInfos must not contain maps so it can be published via Rock.
     */
    std::map<std::string, uint64_t> numReceivedPackets;

    /**
     * Contains low level communication methods (sendData(), receiveData()).
     */
    NetworkAdapter* networkAdapter;
    
    /**
     * constructor of udp module
     *
     * @param config configuration for this module/library
     */
    Communication(const Config &config = Config());
    
    /**
     * destructor of udp module
     */
    virtual ~Communication();
    
    /**
     * Sends a marshalled data sample.
     *
     * @param data Data container.
     * @return bytes sent
     */
    virtual SendInfo sendData(DataContainer &data) = 0;
    
    /**
     * Requests a single, complete sample.
     * \param sampleData Container for the received sample.
     * \param bytes_received Has to be set to the overall number of bytes received
     * (for statistics).
     * \return Returns true if a complete sample is available.
     */
    virtual bool receiveData(struct DataContainer& data, int& bytes_received) = 0;
    
    /**
     * Returns a new frame id which is not part of the resend lists (starting with 1).
     * Each data container (and all the contained receivers) will use its own unique id.
     * Resend containers will use the same id, so the unprocessed frames list
     * can be reused.
     */
    frame_id nextFrameId();
      
 private: 
    pthread_t senderThread;
    pthread_t receiverThread;
    bool stopSending;
    bool stopReceiving; 
    
    std::mutex senderQueuesMutex;
    std::mutex receiverQueuesMutex;
    std::mutex receivedPortNamesMutex;
    std::mutex sendDataMutex;
    std::mutex resendContainersMutex;
    
    std::map< std::string, std::queue< DataContainer > > senderQueues;
    std::map< std::string, std::queue< DataContainer > > receiverQueues;
    
    /**
     * Contains all receivers from which no ack has been received.
     */
    std::map< frame_id, DataContainer > resendContainers;
    
    /**
     * Stores the frame id of resent frames, because resent frames must not be used 
     * for rtt calculations.
     */
    std::set<frame_id> resentFramesId;

    /**
     * Contains the names of the last received samples.
     * A name will only be added if the queue is not already full.
     */
    std::list<std::string> receivedPortNames;
    
    /**
     * Port informations filled by the config file informations.
     */
    std::map< std::string, PortInfo > portInfos;
    
    /**
     * ID of last frame sent. See nextFrameId().
     */
    frame_id lastFrameID;
    
    /**
     * If the message acknowledges the receiving of a frame with the id
     * stored in rttInfo the round trip time will be calculated.
     * If a message has been sent more than one times it will not be used for
     * the rtt calculation.
     */
    void calculateRTT(frame_id frame_id_);
    
    /**
     * Registers the ACK of the receiver 'ip_addr' for frame 'frame_id'.
     * The receiver will be removed from the resend list 'resendContainers'.
     * If all ACKs have been received the resend container will be removed.
     *
     * @param frame_id id of acked frame
     * @param ip_addr ip addr of the acks sender
     */
    void registerAckToResendContainers(frame_id frame_id_, std::string ip_addr);
    
    /**
     * Adds the data container to the resend list.
     * If the packet is not acknowledged after 'resend_after' ms it will be resent
     * 'num_resend_tries' times. A packet is acknowledged if all receivers have
     * sent their ACK (see registerAckFor()).
     */
    void prepareDataForResend(DataContainer const& data);
    
    /**
     * Resends the data containers in resendContainers. 
     */
    void processResendList(float time_passed_ms);
    
    /**
     * Use this method to initialize a thread.
     * Same method for each object, but each object uses its own dataForwarder() method.
     */
    static void* dataForwarderStatic(void* params) {
        ((Communication*)params)->dataForwarder();
        return NULL;
    }
    
    static void* dataReceiverStatic(void* params) {
        ((Communication*)params)->dataReceiver();
        return NULL;
    }
    
    /**
     * Processes the send and resend lists and uses sendData() to forward the data containers.
     * The usleep time of the thread is adapted according to the fullness of the
     * sender queues.
     */
    void* dataForwarder();
    
    /**
     * Waits for data and calls receiveData() constantly. If a complete sample has
     * been received it will be added to the receiver queue.
     * The thread processes the data as fast as possible-
     */
    void* dataReceiver();
};

}

#endif
