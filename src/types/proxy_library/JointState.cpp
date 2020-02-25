#include "JointState.h"

namespace proxy_library {

JointState::JointState()
{
  m_position = 0;
  m_speed = 0.0;
  m_acceleration = 0.0;
  m_effort = 0.0;
}

JointState::JointState(double position, double speed, double acceleration, double effort)
  :m_position(position), m_speed(speed), m_acceleration(acceleration), m_effort(effort)
{
}

bool JointState::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    uint8_t* start_p = (uint8_t*)&m_position;      
    serializeBlock(start_p, start_p + getBufferSize(), it_buffer, buf_size);
    return true;
}

bool JointState::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    uint8_t* start_p = (uint8_t*)&m_position; 
    deserializeBlock(it_buffer, buf_size, start_p, start_p + getBufferSize());
    return true;
}

} // end namespace proxy_library
