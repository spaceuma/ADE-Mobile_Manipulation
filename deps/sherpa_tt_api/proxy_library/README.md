proxy_exit
=============
Rock independent library to communicate with the Rock proxy module.

To use your own data types you have to inherit from the ProxyLibrary and add
your types within the constructor of the child class.

class ProxyLibrarySherpaTT : public ProxyLibrary {
}

ProxyLibrarySherpaTT::ProxyLibrarySherpaTT(Config config) : ProxyLibrary(config) {
    addTypeInfos("map_in", new DEM);
    addTypeInfos("gps_in", new DGPS);
}

The new types have to inherit the class BaseType and implement the virtual functions.
BaseTypes contains some helper functions for serialization.
-------------------------------------------------------
#pragma once

#include <proxy_library/types/BaseType.hpp>

#include "Vector3.h"

namespace proxy_library{
    
class IMU : public BaseType{
 public:    
    uint64_t m_time;
    Vector3 m_acc;
    Vector3 m_gyro;
    Vector3 m_mag;
        
    IMU();
    IMU(uint64_t time, Vector3& acc, Vector3& gyro, Vector3& mag);

    bool serialize(std::vector<uint8_t>::iterator& it_buffer, uint64_t& buf_size) const {  
        serializeVariable(m_time, it_buffer, buf_size);
        m_acc.serialize(it_buffer, buf_size);
        m_gyro.serialize(it_buffer, buf_size);
        m_mag.serialize(it_buffer, buf_size);
        return true;
    }
        
    bool deserialize(std::vector<uint8_t>::const_iterator& it_buffer, uint64_t& buf_size) {
        deserializeVariable(it_buffer, buf_size, m_time);
        m_acc.deserialize(it_buffer, buf_size);
        m_gyro.deserialize(it_buffer, buf_size);
        m_mag.deserialize(it_buffer, buf_size);
        return true;
    }
    
    int64_t getBufferSize() const {
        return 80;
    }
    
    void clear() {
    }
    
    std::string getTypeName() const {
        return "IMU";
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << "\tm_time: " << m_time << 
        "\n\tm_acc: " << m_acc.toString() << 
        "\n\tm_gyro: " << m_gyro.toString() << 
        "\n\tm_mag: " << m_mag.toString();
        return ss.str();
    }
};
} // end namespace proxy_library
-------------------------------------------------------


License
-------
dummy-license

Installation
------------
The easiest way to build and install this package is to use Rock's build system.
See [this page](http://rock-robotics.org/stable/documentation/installation.html)
on how to install Rock.

However, if you feel that it's too heavy for your needs, Rock aims at having
most of its "library" packages (such as this one) to follow best practices. See
[this page](http://rock-robotics.org/stable/documentation/packages/outside_of_rock.html)
for installation instructions outside of Rock.

Rock CMake Macros
-----------------

This package uses a set of CMake helper shipped as the Rock CMake macros.
Documentations is available on [this page](http://rock-robotics.org/stable/documentation/packages/cmake_macros.html).

Rock Standard Layout
--------------------

This directory structure follows some simple rules, to allow for generic build
processes and simplify reuse of this project. Following these rules ensures that
the Rock CMake macros automatically handle the project's build process and
install setup properly.

### Folder Structure

| directory         |       purpose                                                        |
| ----------------- | ------------------------------------------------------               |
| src/              | Contains all header (*.h/*.hpp) and source files                     |
| build/ *          | The target directory for the build process, temporary content        |
| bindings/         | Language bindings for this package, e.g. put into subfolders such as |
| ruby/             | Ruby language bindings                                               |
| viz/              | Source files for a vizkit plugin / widget related to this library    |
| resources/        | General resources such as images that are needed by the program      |
| configuration/    | Configuration files for running the program                          |
| external/         | When including software that needs a non standard installation process, or one that can be easily embedded include the external software directly here |
| doc/              | should contain the existing doxygen file: doxygen.conf               |
