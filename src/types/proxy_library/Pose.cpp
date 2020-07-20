#include "Pose.h"

namespace proxy_library{
    
Pose::Pose()
{
  m_position = Vector3();
  m_orientation = Quaternion();
  m_referenceFrame = "";
  m_frameName = "";
}

Pose::Pose(Vector3& position, Quaternion& orientation, std::string referenceFrame, std::string frameName)
  :m_position(position), m_orientation(orientation),
   m_referenceFrame(referenceFrame), m_frameName(frameName)
{
}

bool Pose::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    m_position.serialize(it_buffer, buf_size);
    m_orientation.serialize(it_buffer, buf_size);
    serializeString(m_referenceFrame, it_buffer, buf_size);
    serializeString(m_frameName, it_buffer, buf_size);
    return true;
}

bool Pose::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    m_position.deserialize(it_buffer, buf_size);
    m_orientation.deserialize(it_buffer, buf_size);
    deserializeString(it_buffer, buf_size, m_referenceFrame);
    deserializeString(it_buffer, buf_size, m_frameName);
    return true;
}

} // end namespace proxy_library


