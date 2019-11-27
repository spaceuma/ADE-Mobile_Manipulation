#pragma once

#include <proxy_library/types/BaseType.hpp>

#include "Vector3.h"

namespace proxy_library{
    
class IMU : public BaseType{
 public:    
    uint64_t m_time;
    Vector3 m_acc;
    Vector3 m_gyro;
    Vector3 m_mag;
        
    IMU();
    IMU(uint64_t time, Vector3& acc, Vector3& gyro, Vector3& mag);

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 8 + 3 * m_acc.getBufferSize();
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "IMU";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "\tm_time: " << m_time << 
        "\n\tm_acc: " << m_acc.toString() << 
        "\n\tm_gyro: " << m_gyro.toString() << 
        "\n\tm_mag: " << m_mag.toString();
        return ss.str();
    }
};
} // end namespace proxy_library
