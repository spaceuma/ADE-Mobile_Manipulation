#pragma once

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library{
    
class MotionCommand : public BaseType {
 public:
    int m_manoeuvreType; //0: Ackermann, 1: PointTurn
    double m_curvature_radm; //in radians/meter
    double m_speed_ms;  //in meters/seconds
    double m_turnRate_rads; //in radians/seconds
     
    MotionCommand();
    MotionCommand(int manoeuvreType, double curvature_radm, double speed_ms, double turnRate_rads);
    ~MotionCommand();
    
    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 32; // Contains 4 byte padding after the integer.
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "MotionCommand";
    }
    
    std::string toString() const {
        return getTypeName() + ": toString() not implemented";
    }
};
}

