#ifndef COM_HELPERS_HPP
#define COM_HELPERS_HPP

#include <time.h>

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>

#include <udp/Frame.hpp>

namespace udp 
{

class Timer {
 public:  
    
  Timer() : startTime(){
  }
  
  void setStartTime() {
      clock_gettime(CLOCK_MONOTONIC, &startTime);
  }
  
  float getPassedTime_sec() {
      struct timespec endTime;
      clock_gettime(CLOCK_MONOTONIC, &endTime);
      return ((endTime.tv_sec - startTime.tv_sec) + 
            (endTime.tv_nsec - startTime.tv_nsec) / 1000000000.0);
  }
  
  float getPassedTime_millisec() {
      struct timespec endTime;
      clock_gettime(CLOCK_MONOTONIC, &endTime);
      return ((endTime.tv_sec - startTime.tv_sec) * 1000.0 + 
            (endTime.tv_nsec - startTime.tv_nsec) / 1000000.0);
  }
  
  float getPassedTime_microsec() {
      struct timespec endTime;
      clock_gettime(CLOCK_MONOTONIC, &endTime);
      return ((endTime.tv_sec - startTime.tv_sec) * 1000000.0 + 
            (endTime.tv_nsec - startTime.tv_nsec) / 1000.0);
  }
  
  float getPassedTime_nanosec() {
      struct timespec endTime;
      clock_gettime(CLOCK_MONOTONIC, &endTime);
      return ((endTime.tv_sec - startTime.tv_sec) * 1000000000.0 + 
            (endTime.tv_nsec - startTime.tv_nsec));
  }
  
   struct timespec startTime;
    
};

class ExtractControlInformations {
 public:
    ExtractControlInformations(std::vector<uint8_t>& ctrl_infos) {
        size_t data_pos = 0;
        size_t data_pos_end = ctrl_infos.size();
    
        while (data_pos < data_pos_end) {
            control_data_id cd_id = ((uint16_t)ctrl_infos[data_pos + 1] << 8) | ctrl_infos[data_pos];
            data_pos += sizeof(control_data_id);

            control_field_len ctrl_field_len = ((uint16_t)ctrl_infos[data_pos + 1] << 8) | ctrl_infos[data_pos];
            data_pos += sizeof(control_field_len);

            controlData[static_cast<CONTROL_IDS>(cd_id)].assign(ctrl_infos.begin() + 
                    data_pos, ctrl_infos.begin() + data_pos + ctrl_field_len);
            data_pos += ctrl_field_len;
        }
    }
    
    bool isControlInformationAvailable(enum CONTROL_IDS ctrl_id) {
        return controlData.find(ctrl_id) != controlData.end();
    }
    
    /**
     * Returns an empty string if the control information is not available.
     */
    std::string getControlInformation(enum CONTROL_IDS ctrl_id) {
        std::map< CONTROL_IDS, std::vector<uint8_t> >::iterator it = controlData.find(ctrl_id);
        std::string str_ret;
        if(it == controlData.end()) {
            return str_ret;
        }
        str_ret.assign(it->second.begin(), it->second.end());
        return str_ret;
    }
    
 private:
    std::map< CONTROL_IDS, std::vector<uint8_t> > controlData;
};

std::vector<std::string> split(const std::string &s, char delim);

} // end namespace udp

#endif