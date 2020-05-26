#ifndef __JOINTS__
#define __JOINTS__

#pragma once

#include "BaseType.hpp"

#include "JointState.h"

namespace proxy_library{
    
class proxy_library::Joints : public BaseType {
    
 public:
    uint64_t m_time;
    std::vector<JointState> m_jointStates;
    std::vector<std::string> m_jointNames;  
      
    Joints();
    Joints(uint64_t time, std::vector<JointState>& jointStates);

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const;  
    

    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size);
    

    int64_t getBufferSize() const {
        
        // Calculate string lengths.
        std::vector< std::string >::const_iterator it = m_jointNames.cbegin();
        uint64_t num_chars = 0;
        for(; it != m_jointNames.cend(); it++) {
            num_chars += it->size();
        }
        JointState js;
        // serializing of vector of strings: Each string is serialized by 
        // NUM_ELEMENTS_SIZE (number of characters) plus the characters.
        return 8 + 
                NUM_ELEMENTS_SIZE + m_jointStates.size() * js.getBufferSize() +
                NUM_ELEMENTS_SIZE + m_jointNames.size() * NUM_ELEMENTS_SIZE + num_chars;
    }
    
    void clear() {
        m_jointStates.clear();
        m_jointNames.clear();
    }
    
    std::string getTypeName() const {
        return "Joints";
    }
    
    std::string toString() const {
        std::stringstream ss;
        
        ss << "m_time: " << m_time << std::endl;
        std::vector<JointState>::const_iterator it = m_jointStates.cbegin();
        ss << "JointStates: " << std::endl;
        for(int i=0; it!= m_jointStates.cend(); it++, i++) {
            ss << "\t" << i << ": " << it->toString() << std::endl;
        }
        std::vector<std::string>::const_iterator it_names = m_jointNames.cbegin();
        ss << "JointNames: " << std::endl;
        for(int i=0; it_names!= m_jointNames.cend(); it_names++, i++) {
            ss << "\t" << i << ": " << *it_names << std::endl;
        }
        
        return ss.str();
    }
};
} // end namespace proxy_library

#endif
