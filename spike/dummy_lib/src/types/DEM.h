#pragma once

#include <proxy_library/types/BaseType.hpp>

#include <fstream> 
#include <iostream>
#include <stdio.h>

#include "Image.h"

namespace proxy_library{
    
enum PointOfReference{
    Pixel00,
    Center
};

class DEM : public BaseType{
  public:
    float m_metersPerPixelX;
    float m_metersPerPixelY;
    float m_metersPerIntensity;
    //Will be FrameMode::MONO_FLOAT32 or FrameMode::MONO_UINT16
    Image m_heightMap;
    //Will be FrameMode::MONO_UINT8 with zeroes cells with unknown
    //values and ones for cells with valid measurements
    Image m_validityMap;
    std::string m_referenceFrame;
    PointOfReference m_pointOfReference;  
      
    DEM();
    
    DEM(    float meters_per_pix_x, 
            float meters_per_pix_y, 
            float m_meters_per_intensity, 
            uint64_t time, 
            uint16_t width, 
            uint16_t height, 
            enum FrameMode frame_mode, 
            std::vector<uint8_t>& data, 
            std::string ref_frame, 
            PointOfReference por) :
            
            m_metersPerPixelX(meters_per_pix_x), 
            m_metersPerPixelY(meters_per_pix_y), 
            m_metersPerIntensity(m_meters_per_intensity), 
            m_heightMap(time, width, height, frame_mode, data),  
            m_validityMap(time, width, height, frame_mode, data), 
            m_referenceFrame(ref_frame), 
            m_pointOfReference(por) {
    }

    DEM(DEM & rhs){
        m_heightMap = rhs.m_heightMap;
        m_metersPerIntensity = rhs.m_metersPerIntensity;
        m_metersPerPixelX = rhs.m_metersPerPixelX;
        m_metersPerPixelY = rhs.m_metersPerPixelY;
        m_pointOfReference = rhs.m_pointOfReference;
        m_referenceFrame = rhs.m_referenceFrame;
        m_validityMap = rhs.m_validityMap;
    }

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 3 * 4 + 
                4 + // padding
                m_heightMap.getBufferSize() + 
                m_validityMap.getBufferSize() +
                NUM_ELEMENTS_SIZE + m_referenceFrame.size() + 
                sizeof(m_pointOfReference);
    }
    
    void clear() {
        m_heightMap.clear();
        m_validityMap.clear();
    }
    
    std::string getTypeName() const {
        return "DEM";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "m_metersPerPixelX: " << m_metersPerPixelX << ", m_metersPerPixelY: " << m_metersPerPixelY << 
        ", m_metersPerIntensity: " << m_metersPerIntensity << ", m_heightMap: " << m_heightMap.toString() << 
        ", m_validityMap: " << m_validityMap.toString() << ", m_referenceFrame: " << m_referenceFrame << 
        ", m_pointOfReference: " << (int)m_pointOfReference;
        return ss.str();
    }
    
    bool storeToFile(std::string filename) {
        std::vector<uint8_t> buffer;
        uint64_t buffer_size = getBufferSize();
        buffer.resize(buffer_size);
        std::vector<uint8_t>::iterator it = buffer.begin();
        uint64_t buf_size = buffer_size;
        if(!serialize(it, buf_size)) {
            std::cerr << "Serialization failed, DEM could not be stored to " << 
                    filename << std::endl;
            return false;
        }
        std::ofstream outfile (filename, std::ofstream::binary);
        if(outfile.is_open()) {
            outfile.write((const char*)buffer.data(), buffer.size());
            outfile.close();
            return true;
        }
        std::cerr << "File " << filename << 
                "could not be opened, DEM could not be stored" << std::endl;
        return false;
    }
    
    bool loadFromFile(std::string filename) {
        std::ifstream infile (filename, std::ifstream::in | std::ifstream::binary| std::ifstream::ate);
        if(infile.is_open()) {
            uint64_t size = infile.tellg();
            std::vector<uint8_t> buffer;
            buffer.resize(size);
            infile.seekg (0, std::ifstream::beg);
            infile.read ((char*)buffer.data(), size);
            infile.close();
        
            std::vector<uint8_t>::const_iterator it = buffer.cbegin();
            uint64_t buf_size = size;
            if(!deserialize(it, buf_size)) {
                std::cerr << "Deserialization failed, DEM could not be loaded from " << 
                    filename << std::endl;
                return false;
            }
            return true;
        } else {
            std::cerr << "File " << filename << 
                    "could not be opened, DEM could not be loaded" << std::endl;
        }
        return false;
    }
};
} // end namespace proxy_library
