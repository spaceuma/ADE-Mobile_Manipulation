#include "ProxyLibrary.hpp"

#include <unistd.h>

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library 
{

// PUBLIC
ProxyLibrary::ProxyLibrary(Config config) : config(config),
        sequencedComm(NULL), 
        dataTypes(), 
        receiverThread(), 
        receiverActive(true) {     
    
    // Create ip_tunnel object. Throws std::invalid_argument if the host address is not valid.
    sequencedComm = new udp::SequencedComm(config.udpConfig);    
    
    // Create receiver thread.
    if(config.udpConfig.init_listen) {
        receiverThread = std::thread(&receiveDataFunc, this);
    } 
}

ProxyLibrary::~ProxyLibrary() {
    receiverActive = false;
    receiverThread.join();
    
    delete sequencedComm;
    sequencedComm = NULL;
    
    std::map<std::string, DataType*>::iterator it = dataTypes.begin();
    for(; it != dataTypes.end(); it++) {
        delete it->second;
    }
    dataTypes.clear();
}

bool ProxyLibrary::sendData(std::string port_name, BaseType* type) {
      
    udp::DataContainer data;
    uint64_t buf_size = type->getBufferSize();
    data.buffer.resize(buf_size);
    std::vector<uint8_t>::iterator it_buffer = data.buffer.begin();
    
    // Serialize data type (buf_size is decreased to zero by serialize).
    try {
        if(!type->serialize(it_buffer, buf_size)) {
            throw std::runtime_error("serialize() returned false");
        }
        if(buf_size != 0) {
            throw std::runtime_error("Buffer not empty after serialization");
        }    
    } catch (std::runtime_error& e) {
        std::cerr << "serialize(): " << e.what();
        std::cerr << "Type " << type->getTypeName() << " could not be serialized" << std::endl;
        std::cerr << "After serialization " << buf_size << " bytes remained in the buffer" << std::endl;
        return false;
    }   

    // Build control map.
    data.setControl(udp::PORT_NAME, port_name);
    std::string buf_size_str = std::to_string(type->getBufferSize());
    data.setControl(udp::PACKET_LEN, buf_size_str);
    data.ackMode = udp::NO_ACK;
    mutexDataTypes.lock();
    // If defined use the ACK mode defined within the type informations. 
    std::map<std::string, DataType*>::iterator it_datatype = dataTypes.find(port_name);
    if(it_datatype != dataTypes.end()) {
        data.ackMode = it_datatype->second->ackMode;
    }  
    mutexDataTypes.unlock();
    
    if(!sequencedComm->forwardPacket(data)) {
        std::cerr << "Error forwarding packet " << port_name << std::endl;
        return false;
    }
    return true;
}

bool ProxyLibrary::addType(std::string port_name, BaseType* base_type, udp::ACK_MODE ack_mode) {
    std::map<std::string, DataType*>::iterator ret = dataTypes.find(port_name);
    if(ret == dataTypes.end()) { // Add new type.
        DataType* data_type = new DataType(base_type, ack_mode);
        mutexDataTypes.lock();
        dataTypes.insert(std::pair<std::string, DataType*>(port_name, data_type));
        mutexDataTypes.unlock();
        return true;
    } else { // Update ack mode, but return false nevertheless. 
        mutexDataTypes.lock();
        ret->second->ackMode = ack_mode;
        mutexDataTypes.unlock();
    }
    return false;
}

// PROTECTED
void* ProxyLibrary::receiveData(ProxyLibrary* comm_proxy) {
 
    udp::DataContainer data;

    // TODO What kind of usleep should be used here?
    while (receiverActive) {
        while(sequencedComm->getPacket(data)) {
            
            // The data which is returned by getPacket() always contains a port name.
            std::string port_name = data.getControlContent(udp::PORT_NAME);
            
            DataType* data_type = NULL;
            mutexDataTypes.lock();
            // Map received port name to type.
            std::map<std::string, DataType*>::iterator it_type = dataTypes.find(port_name);
            if(it_type == dataTypes.end()) {
                std::cerr << "No data type known for the received port name " << 
                        port_name << std::endl;
                mutexDataTypes.unlock();
                continue;
            }
            data_type = it_type->second;
            mutexDataTypes.unlock();
            
            // Lock data type which belongs to the received port name.
            data_type->mutex.lock();
            
            // Clears variables and lists (list elements are just pushed back).
            data_type->baseType->clear();
            
            // Tries to unmarshal / deserialize the received data.
            std::vector<uint8_t>::const_iterator it_data = data.buffer.cbegin();
            uint64_t buf_size = data.buffer.size();
            try {
                if(!data_type->baseType->deserialize(it_data, buf_size)) {
                    throw std::runtime_error("derserialize() returns false");
                }
                if(buf_size != 0) {
                    throw std::runtime_error("Buffer not empty after deserialization");
                }
            } catch (std::runtime_error& e) {
                std::cerr << "deserialize(): " << e.what();
                std::cerr << "Received data " << port_name << 
                        " could not be deserialized" << std::endl;
                std::cerr << "After deserialization " << buf_size << 
                        " bytes remained in the buffer" << std::endl;
                data_type->mutex.unlock();
                continue;
            }
            
            if(config.printReceivedTypeInfos) {
                std::cout << "Received data sample " << port_name << std::endl;
                std::cout << "    " << data_type->baseType->toString() << std::endl;
            }
            // Note that new data is available, port name is known.
            data_type->newInformations = true;
            data_type->mutex.unlock();
            
            // TODO Use better sleep values?
            usleep(1000);
        }
        // Nothing to read, sleep a little longer.
        usleep(10000);
    }
    return NULL;
}

    
} // end namespace proxy_library
