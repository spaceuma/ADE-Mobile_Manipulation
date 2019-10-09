#pragma once

#include <osg/Geode>
#include <osg/Geometry>

namespace proxy_library
{
    
class GridNode : public osg::Group
{
 public:
    /**
     * Creates a new sub scene graph for a grid which has the its center at 0/0
     *
     * \param rows The number of rows
     * \param cols The number of cols 
     * \param dx The size of a cell in x direction
     * \param dx The size of a cell in y direction
     * \param color The color of the grid
     */
    GridNode(int rows,int cols,float dx, float dy, bool show_coordinates = false, 
             const ::osg::Vec4 &color=::osg::Vec4());
};

}

