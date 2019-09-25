#pragma once

#include <osg/Group>

#include <proxy_library/types/TypeInfos.hpp>
#include <proxy_library/types/Types.hpp>

namespace proxy_library
{    

/**
 * Base group object for type-visualizations.
 */
class VBaseObject : public osg::Group {
 public: 
    VBaseObject(std::string type_name, TypeInfos* type_info) : 
            typeName(type_name), typeInfo(type_info) {
    }
     
    virtual ~VBaseObject() {
    }
    
    /**
     * Call is mutex protected.
     */
    virtual void updateData() = 0;
    
    std::string getTypeName() {
        return typeName;
    }
    
    TypeInfos* getTypeInfo() {
        return typeInfo;
    }
    
 protected:
    std::string typeName; // (port name)
    TypeInfos* typeInfo;
    
};

} // end namespace proxy_library