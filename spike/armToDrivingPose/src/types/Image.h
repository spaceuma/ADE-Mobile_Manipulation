#pragma once

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library{
    
typedef std::vector<uint8_t> FrameBuffer;

enum FrameMode{
    UNDEFINED = 0,
    RGB_UINT8, // Bit je Kanal
    RGB_UINT16,
    RGB_FLOAT32,
    RGB_FLOAT64,
    BGR_UINT8,
    BGR_UINT16,
    BGR_FLOAT32,
    BGR_FLOAT64,
    MONO_UINT8 = 128,
    MONO_UINT16,
    MONO_FLOAT32,
    MONO_FLOAT64,
    JPEG = 256
};

class Image : public BaseType{
public:
    uint64_t m_time;
    uint16_t m_width;
    uint16_t m_height;
    FrameMode m_frameMode; // has to be serialized as uint32_t
    FrameBuffer m_data;

    Image();

    Image(uint64_t time, uint16_t width, uint16_t height, enum FrameMode frame_mode, 
          std::vector<uint8_t>& data);

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 12 + sizeof(m_frameMode) + NUM_ELEMENTS_SIZE + m_data.size();
    }
    
    void clear() {
        m_data.clear();
    }
    
    std::string getTypeName() const {
        return "Image";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "m_time: " << m_time << 
                ", m_width: " << m_width << 
                ", m_height: " << m_height << 
                ", m_frameMode: " << (int)m_frameMode << 
                ", size m_data: " << m_data.size();
        return ss.str();
    }
    
    bool isIsGreyScale() const {
        return m_frameMode >= MONO_UINT8 && m_frameMode <= MONO_FLOAT32;
    }

    bool isIsRgb() const {
        return m_frameMode >= RGB_UINT8 && m_frameMode <= BGR_FLOAT32;
    }
    
    bool isCompressed() const {
        return m_frameMode == JPEG;
    }
    
    size_t getPixelSizeByte() const;

    size_t getRowSizeByte() const;

    size_t getBufferIndex(size_t column, size_t row) const;

    size_t getNChannels() const;
    
    size_t getPixelDepthBit() const;
    
    size_t getPixelDepthByte() const;
    
    template<typename Pixel>
    Pixel getPixel(size_t column, size_t row)
    {
        size_t index = getBufferIndex(column, row);
        assert(m_data.size() >= index);
        Pixel* ptr = (Pixel*)&(m_data[index]);
        return *ptr;
    }
};
} // end namespace proxy_library
