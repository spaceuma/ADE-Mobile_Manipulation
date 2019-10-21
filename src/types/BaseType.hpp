#pragma once

#include <assert.h>

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

//#include <proxy_library/types/HelperTypes.hpp>

namespace proxy_library
{

// Vectors are serialized with: <number_of_elements (8 bytes)> <element1> <element2>..
static uint64_t NUM_ELEMENTS_SIZE = 8;

/**
 * Parent class for all types.
 * This class contains all methods required for serialization / deserialization.
 * Currently only little endian format is supported.
 * No unicode strings are supported.
 */
class BaseType
{
 public:      
     virtual ~BaseType() {
     }
     
    /**
     * Serializes the current type to the passed buffer.
     * \param it_buffer Pointer to the buffer the data type should beserialized to.
     * The caller has to take care that the buffer is big enough for which getSize() 
     * should be used.
     * \param buf_size Remaining buf_size.
     * \return False if the passed buffer is too small.
     */
    virtual bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const = 0;  
    
    /**
     * Fills this type with the passed content. 
     * \param it_buffer Iterator to the serialized data type which should be used to fill this object.
     * After this call it_buffer will point at the end of this data type.
     * \param buf_size Size of the (remaining) buffer, which is reduced during deserialization.
     * \return If not enough data has been received false will be returned. 
     * No checks are performed but the remaining buffer size is assumed to be zero after
     * deserialization.
     */
    virtual bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) = 0;
    
    /**
     * Returns the number of bytes the current serialized object would require.
     * The size of variables do not change due to serialization. Serialized vectors are 
     * represented by an uint64_t containing the number of elements of the vector
     * followed by all its elements.
     * \warning Do not use sizeof(this) because of padding.
     */
    virtual int64_t getBufferSize() const = 0;
    
    /**
     * Has to be implemented if the data type contains any lists.
     * These lists have to be cleared within this method.
     */
    virtual void clear() {
    }
    
    virtual std::string getTypeName() const {
        return "BaseType";
    }
    
    virtual std::string toString() const {
        return getTypeName() + ": toString() not implemented";
    }
    
 protected:
    /**
     * Serializes the passed vector to the buffer.
     * \param list Vector which should be serialized.
     * \param it_buffer This buffer receives the serialized data.
     * \param available_buffer_size Size of the passed buffer. checkAvailableBuffer()
     * checks if the data fits into the buffer and reduces the available buffer size.
     * \return The returned iterator points to the next free buffer address.
     */     
    template <typename T>
    void serializeVector(
            std::vector<T>const& list, 
            std::vector<uint8_t>::iterator& it_buffer,
            uint64_t& available_buffer_size) const
    {        
        // Checks buffer size.
        uint64_t num_elements = list.size();
        
        // Copy number of elements to the end of the buffer.
        checkAvailableBuffer(available_buffer_size, NUM_ELEMENTS_SIZE);
        uint8_t* elements_p = (uint8_t*)&num_elements;
        it_buffer = std::copy(elements_p, elements_p + 8, it_buffer);
        
        // Copy all elements.
        uint64_t num_bytes = num_elements * sizeof(T);
        checkAvailableBuffer(available_buffer_size, num_bytes);
        // Get a uint8_t pointer to the start address of the vector.
        uint8_t* list_p = (uint8_t*) &(*list.begin());
        it_buffer = std::copy(list_p, list_p+num_bytes, it_buffer);
    }
    
    /**
     * Special string serialization. First copies the string conten to a vector
     * and uses serializeVector() after that.
     */
    void serializeString(
            std::string const& str, 
            std::vector<uint8_t>::iterator& it_buffer,
            uint64_t& available_buffer_size) const
    {     
        std::vector<char> vec(str.begin(), str.end());
        serializeVector<char>(vec, it_buffer, available_buffer_size);
    }
    
    /**
     * Writes the passed variable to the buffer.
     * \param variable Variable which should be serialized.
     * \param it_buffer This buffer receives the serialized data.std::string toString()
     * \param available_buffer_size Size of the passed buffer. checkAvailableBuffer()
     * checks if the data fits into the buffer and reduces the available buffer size.
     * \return The returned iterator points to the next free buffer address.
     */ 
    template <typename T>
    void serializeVariable(
            T const& variable, 
            std::vector<uint8_t>::iterator& it_buffer,
            uint64_t& available_buffer_size) const
    {
        uint64_t var_size = sizeof(T);
        
        checkAvailableBuffer(available_buffer_size, var_size);
            
        // Copies variable to buffer.
        uint8_t* t_p = (uint8_t*)&variable;
        it_buffer = std::copy(t_p, t_p + var_size, it_buffer);
    }
    
    /**
     * Serializes a data block.
     * \param start Start address of the block.
     * \param end One byte after the block, so [start, end) are serialized.
     * \param it_buffer This buffer receives the serialized data.
     * \param available_buffer_size Size of the passed buffer. checkAvailableBuffer()
     * checks if the data fits into the buffer and reduces the available buffer size.
     * \return The returned iterator points to the next free buffer address.
     */
    void serializeBlock(
            uint8_t const* start_p,
            uint8_t const* end_p,
            std::vector<uint8_t>::iterator& it_buffer,
            uint64_t& available_buffer_size) const
    {
        assert(end_p > start_p);
        uint64_t block_size = end_p - start_p;
        
        checkAvailableBuffer(available_buffer_size, block_size);
            
        // Copy parameter to buffer.
        it_buffer = std::copy(start_p, end_p, it_buffer);
    }
     
    /**
     * Fills the passed list with data extracted from the stream.
     * \param it_buffer Iterator pointing to the 8-byte block holding
     * the number of vector-elements followed by all the elements.
     * \param available_buffer_size Size of the remaining buffer. This size
     * reduces during the deserialization.
     * \param list Vector will be filled with the elements extracted from the buffer.
     * \return Iterator pointing to the first byte in the buffer after the serialized vector.
     */
    template <typename T>
    void deserializeVector(
            std::vector<uint8_t>::const_iterator& it_buffer, 
            uint64_t& available_buffer_size,
            std::vector<T>& list) {
        // Extract number of elements.
        checkAvailableBuffer(available_buffer_size, NUM_ELEMENTS_SIZE);
        uint64_t num_elements = 0;
        uint8_t* num_elements_p = (uint8_t*)&num_elements;
        std::copy(it_buffer, it_buffer+8, num_elements_p);
        std::advance(it_buffer, 8);
        // Fill the past list with the elements extracted from the stream.
        T element;
        uint8_t* element_p = (uint8_t*)&element;
        uint64_t element_size = sizeof(T);
        for(unsigned int i=0; i<num_elements; ++i) {
            checkAvailableBuffer(available_buffer_size, element_size);
            std::copy(it_buffer, it_buffer+element_size, element_p);
            list.push_back(element);
            std::advance(it_buffer, element_size);
        }
    } 
    
    /**
     * Special string deserialization. Uses deserializeVector() and copies
     * the content to the past string.
     */
    void deserializeString(
            std::vector<uint8_t>::const_iterator& it_buffer, 
            uint64_t& available_buffer_size,
            std::string& str) {
        std::vector<char> vec;
        deserializeVector<char>(it_buffer, available_buffer_size, vec);
        str = std::string(vec.begin(), vec.end());
    }
    
    /**
     * Fills the passed variable with data extracted from the stream.
     * \param it_buffer Iterator pointing to the serialized variable in the buffer.
     * \param available_buffer_size Size of the remaining buffer. This size is
     * reduced during the deserialization by checkAvailableBuffer().
     * \param variable Container for the extracted variable.
     * \return Iterator pointing to the first byte after the variable.
     */
    template <typename T>
    void deserializeVariable(
            std::vector<uint8_t>::const_iterator& it_buffer,
            uint64_t& available_buffer_size,
            T& variable) 
    {
        // Extracts the variable from the buffer and advances the buffer position.
        uint64_t size_para = sizeof(variable);
        checkAvailableBuffer(available_buffer_size, size_para);
        uint8_t* variable_p = (uint8_t*)&variable;
        std::copy(it_buffer, it_buffer+size_para, variable_p);
        std::advance(it_buffer, size_para);
    }
    
    /**
     * Fills the memory block from start_p to end_p with data extracted from the stream.
     * \param it_buffer Iterator pointing to the buffer.
     * \param available_buffer_size Size of the remaining buffer. This size is
     * reduced during the deserialization by checkAvailableBuffer().
     * \param start_p Start address within this object the buffer data is written to.
     * \param end_p Points to the first byte after the block, so [start_p, end_p) is used.
     * \return Iterator pointing to the first byte after the block.
     */
    void deserializeBlock(
            std::vector<uint8_t>::const_iterator& it_buffer,
            uint64_t& available_buffer_size,
            uint8_t* start_p,
            uint8_t* end_p) 
    {
        // Copies a block of data from the buffer to start_p.
        assert(end_p > start_p);
        uint64_t block_size = end_p - start_p;
        checkAvailableBuffer(available_buffer_size, block_size);
        std::copy(it_buffer, it_buffer+block_size, start_p);
        std::advance(it_buffer, block_size);
    }
    
    /**
     * Throws an exception if the object size exceeds the available buffer size.
     * If the data fits into the buffer, available_buffer_size will be reduced.
     * \param available_buffer_size Remaining size in the buffer.
     * \param object_size Size of the data which should be read from or written to
     * the buffer.
     * \throws runtime_error If the object does not fit into the buffer a runtime_error
     * is thrown.
     */
    void checkAvailableBuffer(uint64_t& available_buffer_size, uint64_t object_size) const {
         if(available_buffer_size < object_size) {
             std::stringstream ss;
             ss << "Available buffer (" << available_buffer_size << ") is too small, " << 
                    object_size << " byte are required";
             throw std::runtime_error(ss.str().c_str());
         } else {
             available_buffer_size -= object_size;
         }
    }
};

} // end namespace proxy_library


