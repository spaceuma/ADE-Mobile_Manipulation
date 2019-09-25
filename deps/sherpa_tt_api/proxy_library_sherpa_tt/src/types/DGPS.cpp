#include "DGPS.h"

namespace proxy_library{

DGPS::DGPS()
{
}

DGPS::DGPS(uint64_t time, double latitude, double longitude, int noOfSatellites, double altitude,
double geoidalSeparation, double ageOfDifferentialCorrections,
double deviationLatitude, double deviationLongitude,
double deviationAltitude)
  :m_time(time), m_latitude(latitude), m_longitude(longitude), m_noOfSatellites(noOfSatellites),
   m_altitude(altitude), m_geoidalSeparation(geoidalSeparation),
   m_ageOfDifferentialCorrections(ageOfDifferentialCorrections),
   m_deviationLatitude(deviationLatitude), m_deviationLongitude(deviationLongitude),
   m_deviationAltitude(deviationAltitude)
{
}

DGPS::~DGPS()
{
  m_time = 0;
  m_latitude = 0.0;
  m_longitude = 0.0;
  m_noOfSatellites = 0;
  m_altitude = 0.0;
  m_geoidalSeparation = 0.0;
  m_ageOfDifferentialCorrections = 0.0;
  m_deviationLatitude = 0.0;
  m_deviationLongitude = 0.0;
  m_deviationAltitude = 0.0;
}

bool DGPS::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    uint8_t* start_p = (uint8_t*)&m_time;      
    serializeBlock(start_p, start_p + getBufferSize(), it_buffer, buf_size);
    return true;
}

bool DGPS::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    uint8_t* start_p = (uint8_t*)&m_time; 
    deserializeBlock(it_buffer, buf_size, start_p, start_p + getBufferSize());
    return true;
}

}

