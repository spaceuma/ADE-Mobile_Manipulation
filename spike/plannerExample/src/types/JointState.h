#pragma once

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library{
    
class JointState : public BaseType {
  public:
    double m_position;
    double m_speed;
    double m_acceleration;
    double m_effort;  
      
    JointState();
    JointState(double m_position, double m_speed, double m_acceleration, double m_effort);

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 32;
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "JointState";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "m_position: " << m_position << ", m_speed: " << m_speed << 
                ", m_acceleration: " << m_acceleration << ", m_effort: " << m_effort;
        return ss.str();
    }
};
} // end namespace proxy_library