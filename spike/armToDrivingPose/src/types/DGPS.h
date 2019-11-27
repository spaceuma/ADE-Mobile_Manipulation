#pragma once

#include <proxy_library/types/BaseType.hpp>

namespace proxy_library{
    
class DGPS : public BaseType{
  public:
    uint64_t m_time;
    double m_latitude;
    double m_longitude;
    int m_noOfSatellites;
    double m_altitude;
    double m_geoidalSeparation;
    double m_ageOfDifferentialCorrections;
    double m_deviationLatitude;
    double m_deviationLongitude;
    double m_deviationAltitude;  
     
    DGPS();
    DGPS(uint64_t time, double latitude, double longitude, int noOfSatellites, double altitude,
         double geoidalSeparation, double ageOfDifferentialCorrections,
         double deviationLatitude, double deviationLongitude, double deviationAltitude);

    ~DGPS();

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        return 80; // containing the four padding bytes after the integer.
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "DGPS";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss <<   "m_time: " << m_time << 
                ", m_latitude: " << m_latitude << 
                ", m_longitude: " << m_longitude <<
                ", m_noOfSatellites: " << m_noOfSatellites << 
                ", m_altitude: " << m_altitude << 
                ", m_geoidalSeparation: " << m_geoidalSeparation<< 
                ", m_ageOfDifferentialCorrections: " << m_ageOfDifferentialCorrections << 
                ", m_deviationLatitude: " << m_deviationLatitude << 
                ", m_deviationLongitude: " << m_deviationLongitude << 
                ", m_deviationAltitude: " << m_deviationAltitude;
        return ss.str();
    }
};
} // end namespace proxy_library
 
