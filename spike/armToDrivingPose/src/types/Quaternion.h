#pragma once

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library{

class Quaternion : public BaseType{
  public:
    double m_x;
    double m_y;
    double m_z;
    double m_w;  

    Quaternion();
    Quaternion(double x, double y, double z, double w);
    
    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 32;
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "Quaternion";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "m_x: " << m_x << ", m_y: " << m_y << ", m_z: " << m_z << ", m_w: " << m_w;
        return ss.str();
    }
};
} // end namespace proxy_library
