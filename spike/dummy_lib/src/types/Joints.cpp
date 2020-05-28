#include  "Joints.h"

namespace proxy_library {

Joints::Joints()
{
    // TODO Auto-generated constructor stub

    m_time = 0;
    m_jointStates = std::vector<JointState>();
}

Joints::Joints(uint64_t time, std::vector<JointState>& jointStates)
    : m_time(time), m_jointStates(jointStates)
{
    // TODO
}

bool Joints::serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {
    serializeVariable(m_time, it_buffer, buf_size);
    // sizeof(jointStates) is 40 not 32 because of the template functions, so we have to
    // do it by hand.. first the number of elements, after that the elements.
    serializeVariable((uint64_t)m_jointStates.size(), it_buffer, buf_size);
    std::vector<JointState>::const_iterator it = m_jointStates.cbegin();
    for(; it != m_jointStates.cend(); it++) {
        it->serialize(it_buffer, buf_size);
    }
    serializeVariable((uint64_t)m_jointNames.size(), it_buffer, buf_size);
    std::vector<std::string>::const_iterator it_names = m_jointNames.cbegin();
    for(; it_names != m_jointNames.cend(); it_names++) {
        serializeString(*it_names, it_buffer, buf_size);
    }
    return true;
}

bool Joints::deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
    deserializeVariable(it_buffer, buf_size, m_time);
    // sizeof(jointStates) is 40 not 32 because of the template functions, so we have to
    // do it by hand.. first the size after that the elements.
    uint64_t num_states = 0;
    deserializeVariable(it_buffer, buf_size, num_states);
    JointState j_state;
    for(unsigned int i=0; i<num_states; i++) {
        j_state.deserialize(it_buffer, buf_size);
        m_jointStates.push_back(j_state);
    }
    
    uint64_t num_names = 0;
    deserializeVariable(it_buffer, buf_size, num_names);
    std::string name;
    for(unsigned int i=0; i<num_names; i++) {
        deserializeString(it_buffer, buf_size, name);
        m_jointNames.push_back(name);
    } 
    return true;
}

} // end namespace proxy_library
