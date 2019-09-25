#include "DEM.h"

namespace proxy_library{

DEM::DEM()
{
}

bool DEM::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    int size_floats = 12;
    uint8_t* start_p = (uint8_t*)&m_metersPerPixelX;      
    serializeBlock(start_p, start_p + size_floats, it_buffer, buf_size);
    uint32_t padding;
    serializeVariable(padding, it_buffer, buf_size);
    m_heightMap.serialize(it_buffer, buf_size);
    m_validityMap.serialize(it_buffer, buf_size);
    serializeString(m_referenceFrame, it_buffer, buf_size);
    uint32_t ref_int = (uint32_t)m_pointOfReference;
    serializeVariable(ref_int, it_buffer, buf_size);
    return true;
}

bool DEM::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    int size_floats = 12;
    uint8_t* start_p = (uint8_t*)&m_metersPerPixelX; 
    deserializeBlock(it_buffer, buf_size, start_p, start_p + size_floats);
    uint32_t padding;
    deserializeVariable(it_buffer, buf_size, padding);
    m_heightMap.deserialize(it_buffer, buf_size);
    m_validityMap.deserialize(it_buffer, buf_size);
    deserializeString(it_buffer, buf_size, m_referenceFrame);
    uint32_t ref_int = 0;;
    deserializeVariable(it_buffer, buf_size, ref_int);
    m_pointOfReference = (enum PointOfReference) ref_int;
    return true;
}

} // end namespace proxy_library



