#pragma once

#include <proxy_library/types/BaseType.hpp>

#include "Vector3.h"
#include "Quaternion.h"

namespace proxy_library{
    
class Pose : public BaseType {
 public:
    Vector3 m_position;
    Quaternion m_orientation;
    std::string m_referenceFrame;
    std::string m_frameName;   
      
    Pose();
    Pose(Vector3& position, Quaternion& orientation, std::string referenceFrame,
        std::string frameName);

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return m_position.getBufferSize() + 
                m_orientation.getBufferSize() +
                2 * NUM_ELEMENTS_SIZE + m_referenceFrame.size() + m_frameName.size();
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "Pose";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "m_position: " << m_position.toString() << 
        ", m_orientation: " << m_orientation.toString() << 
        ", m_referenceFrame: " << m_referenceFrame <<
        ", m_frameName: " << m_frameName;
        return ss.str();
    }
};

} // end namespace proxy_library
