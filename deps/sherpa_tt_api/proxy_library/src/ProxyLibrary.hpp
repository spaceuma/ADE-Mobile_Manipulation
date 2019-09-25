#pragma once

#include <map>
#include <mutex>
#include <thread>

#include <udp/communications/SequencedComm.hpp>

#include "Config.hpp"
#include "DataType.hpp"

namespace udp {
    class SequencedComm;
}

namespace proxy_library
{    

class BaseType;

/**
 * Proxy communication class which reads (udp::SequencedComm) incoming proxy data
 * and deserializes the received data types. To use your own data types inherit
 * from this class and add your data types within the constructor of your child
 * class using the method addType().
 */
class ProxyLibrary {
     
 public:
    ProxyLibrary(Config config);
    
    ~ProxyLibrary();
    
    /**
     * Send a data type to the named port.
     * \param port_name The port name has to be an output port name of the
     * receiver proxy.
     * \param type If type is NULL the current data type within packetNameToType
     * will be used.
     * \return Returns false if a serialization error occurs or if the data 
     * cannot be forwarded.
     */
    bool sendData(std::string port_name, BaseType* type);
    
    /**
     * This defines the connection between the name / port_name and the data type.
     * This class will use the passed BaseType memory to store the received data types,
     * do not free the memory by yourself. For each new data type additional 
     * type-informations will be created. This has to be done for every packet 
     * which should be transferred.
     * \param name Name of the data type or name of the port name if the data is sent
     * to Rock and should be forwarded via this port.
     * \param base_type Memory for the according Base-Type. Do not free the memory by
     * yourself.
     * \param ack_mode NO_ACK: The data packet is sent without any ack. FRAME_ACK: 
     * If the data packet is not acknowledged the complete packet will be resend 
     * (see udp::Config for details). FRAGMENT_ACK: Each part of the data_packet has
     * to be acknowledged. Instead of resending the complete packet only the missing
     * fragments will be resend. This ack mode should be used for larger data packets
     * which should be delivered reliably.
     * \return Returns false if a type with the passed name has already be added.
     * Nevertheless you can use the method to change the ACK mode.
     */
    bool addType(std::string name, BaseType* base_type, udp::ACK_MODE ack_mode=udp::NO_ACK);
        
    /**
     * Returns the current data type according to the past name.
     * If no new data type is available false will be returned.
     * \param port_name Name of the data type.
     * \param base_type Will be filled with tht current data type.
     * \return True if a new data type is available.
     */
    template<typename T>
    bool getDataType(std::string port_name, T& base_type) {
        mutexDataTypes.lock();
        // Map received port name to type.
        std::map<std::string, DataType*>::iterator it_type = dataTypes.find(port_name);
        if(it_type == dataTypes.end()) {
            std::cerr << "No data type known for the passed port name " <<
                    port_name << std::endl;
            mutexDataTypes.unlock();
            return false;
        }
        DataType* data_type = it_type->second;
        if(!data_type->newInformations) {
            mutexDataTypes.unlock();
            return false;
        }
        data_type->mutex.lock();

        T* cv = dynamic_cast<T*>(data_type->baseType);
        if(!cv){
            std::cerr << "Error converting to BaseType for port "<<
                         port_name<<std::endl;
            mutexDataTypes.unlock();
            return false;
        }

        base_type = *cv;
        data_type->newInformations = false;
        data_type->mutex.unlock();
        mutexDataTypes.unlock();
        return true;
    }

    /**
     * Returns the underlying udp communication library.
     */
    inline udp::SequencedComm* getSequencedComm() {
        return sequencedComm;
    }
    
 protected:
     Config config;
     // Proxy communication library.
     udp::SequencedComm* sequencedComm;
     // All known data types which have been added by addType().
     // We use pointer here because of the contained non-copyable mutexes.
     std::map<std::string, DataType*> dataTypes;
     std::mutex mutexDataTypes;
    
    /**
     * Reads incoming samples from the proxy and deserializes the received
     * data types.
     */
    void* receiveData(ProxyLibrary* comm_proxy);

 private:
     // Thread for receiveData().
     std::thread receiverThread;
     // If set to false the receieData() method will return.
     bool receiverActive;
     
     /**
      * Static wrapper method for receiveData().
      */
     static void* receiveDataFunc(ProxyLibrary* comm_proxy) {
         return comm_proxy->receiveData(comm_proxy);
     }
};

} // end namespace proxy_library
