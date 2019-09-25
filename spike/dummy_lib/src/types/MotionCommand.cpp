#include "MotionCommand.h"

namespace proxy_library{
MotionCommand::MotionCommand()
{
	// TODO Auto-generated constructor stub

  m_curvature_radm = 0.0;
  m_manoeuvreType = 0;
  m_speed_ms = 0.0;
  m_turnRate_rads = 0.0;
}

MotionCommand::MotionCommand(int manoeuvreType, 
                             double curvature_radm, 
                             double speed_ms, 
                             double turnRate_rads)
  :m_manoeuvreType(manoeuvreType), 
  m_curvature_radm(curvature_radm), 
  m_speed_ms(speed_ms), 
  m_turnRate_rads(turnRate_rads)
{
  // TODO
}

MotionCommand::~MotionCommand()
{
	// TODO Auto-generated destructor stub
}

bool MotionCommand::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    uint8_t* start_p = (uint8_t*)&m_manoeuvreType;      
    serializeBlock(start_p, start_p + getBufferSize(), it_buffer, buf_size);
    return true;
}

bool MotionCommand::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    uint8_t* start_p = (uint8_t*)&m_manoeuvreType; 
    deserializeBlock(it_buffer, buf_size, start_p, start_p + getBufferSize());
    return true;
}

} // end namespace proxy_library

