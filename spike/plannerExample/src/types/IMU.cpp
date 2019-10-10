#include "IMU.h"

namespace proxy_library{

IMU::IMU()
{
  // TODO Auto-generated destructor stub
  m_time = 0;
  m_acc = Vector3();
  m_gyro = Vector3();
  m_mag = Vector3();
}

IMU::IMU(uint64_t time, Vector3& acc, Vector3& gyro, Vector3& mag)
  :m_time(time), m_acc(acc), m_gyro(gyro), m_mag(mag)
{
}

bool IMU::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {  
    serializeVariable(m_time, it_buffer, buf_size);
    m_acc.serialize(it_buffer, buf_size);
    m_gyro.serialize(it_buffer, buf_size);
    m_mag.serialize(it_buffer, buf_size);
    return true;
}

bool IMU::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    deserializeVariable(it_buffer, buf_size, m_time);
    m_acc.deserialize(it_buffer, buf_size);
    m_gyro.deserialize(it_buffer, buf_size);
    m_mag.deserialize(it_buffer, buf_size);
    return true;
}

} // end namespace proxy_library
