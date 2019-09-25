#include "Image.h"

namespace proxy_library{

Image::Image()
{
  m_time = 0.0;
  m_width = 0.0;
  m_height = 0.0;
  m_frameMode = UNDEFINED;
}

Image::Image(uint64_t time, uint16_t width, uint16_t height, enum FrameMode frame_mode, 
             std::vector<uint8_t>& data)
  :m_time(time), m_width(width), m_height(height), m_frameMode(frame_mode), m_data(data)
{ 
}

bool Image::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    serializeVariable(m_time, it_buffer, buf_size);
    serializeVariable(m_width, it_buffer, buf_size);
    serializeVariable(m_height, it_buffer, buf_size);
    serializeVariable((uint32_t)m_frameMode, it_buffer, buf_size);
    serializeVector(m_data, it_buffer, buf_size);
    return true;
}

bool Image::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    deserializeVariable(it_buffer, buf_size, m_time);
    deserializeVariable(it_buffer, buf_size, m_width);
    deserializeVariable(it_buffer, buf_size, m_height);
    uint32_t enum_frame_mode;
    deserializeVariable(it_buffer, buf_size, enum_frame_mode);
    m_frameMode = (enum FrameMode)enum_frame_mode;
    deserializeVector(it_buffer, buf_size, m_data);
    return true;
}

size_t Image::getPixelSizeByte() const {
    return getPixelDepthByte()*getNChannels();
}

size_t Image::getRowSizeByte() const {
    return getPixelSizeByte() * m_width;
}

size_t Image::getBufferIndex(size_t column, size_t row) const{
    return row*getRowSizeByte()+column*getPixelSizeByte();
}


size_t Image::getNChannels() const {
    if(        m_frameMode == BGR_UINT8
               || m_frameMode == RGB_UINT8
               || m_frameMode == RGB_UINT16
            || m_frameMode == BGR_UINT16
            || m_frameMode == RGB_FLOAT32
            || m_frameMode == BGR_FLOAT32
            || m_frameMode == BGR_FLOAT64
            || m_frameMode == RGB_FLOAT64){
        return 3;
    }
    else if (m_frameMode == MONO_UINT8
             || m_frameMode == MONO_UINT16
             || m_frameMode == MONO_FLOAT32
             || m_frameMode == MONO_FLOAT64) {
        return 1;
    }
    else{
        throw("Unexpected frame mode");
    }
}

size_t Image::getPixelDepthBit() const{
    if( m_frameMode == BGR_UINT8 ||
            m_frameMode == RGB_UINT8 ||
            m_frameMode == MONO_UINT8){
        return 8;
    }
    else if (m_frameMode == RGB_UINT16
             || m_frameMode == BGR_UINT16
             || m_frameMode == MONO_UINT16) {
        return 16;
    }
    else if (m_frameMode == MONO_FLOAT32
             || m_frameMode == RGB_FLOAT32
             || m_frameMode == BGR_FLOAT32) {
        return 32;
    }
    else if (m_frameMode == MONO_FLOAT64
             || m_frameMode == RGB_FLOAT64
             || m_frameMode == BGR_FLOAT64) {
        return 64;
    }
    else{
        throw("Unexpected frame mode");
    }
}

size_t Image::getPixelDepthByte() const
{
    return getPixelDepthBit()/8;
}

} // end namespace proxy_library
