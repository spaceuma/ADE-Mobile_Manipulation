#include "Vector3.h"

namespace proxy_library {

Vector3::Vector3()
{
  m_x = 0.0;
  m_y = 0.0;
  m_z = 0.0;
}

Vector3::Vector3(double x, double y, double z)
  :m_x(x), m_y(y), m_z(z)
{
	// TODO Auto-generated constructor stub

}

bool Vector3::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    uint8_t* start_p = (uint8_t*)&m_x;      
    serializeBlock(start_p, start_p + getBufferSize(), it_buffer, buf_size);
    return true;
}

bool Vector3::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    uint8_t* start_p = (uint8_t*)&m_x; 
    deserializeBlock(it_buffer, buf_size, start_p, start_p + getBufferSize());
    return true;
}

} // end namespace proxy_library


