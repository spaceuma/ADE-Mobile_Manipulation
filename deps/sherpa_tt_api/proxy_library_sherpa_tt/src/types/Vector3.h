#pragma once

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library{
class Vector3 : public BaseType {
  public:
    double m_x;
    double m_y;
    double m_z;
      
    Vector3();
    Vector3(double x, double y, double z);
    
    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 24;
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "Vector3";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "m_x: " << m_x << ", m_y: " << m_y << ", m_z: " << m_z;
        return ss.str();
    }
};
}

