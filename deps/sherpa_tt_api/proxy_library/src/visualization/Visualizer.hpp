#pragma once

#include <unistd.h>

#include <iostream>
#include <memory>
#include <thread>

#include <osg/Geode>
#include <osg/ShapeDrawable>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/SphericalManipulator>

#include <proxy_library/types/TypeInfos.hpp>
#include <proxy_library/visualization/CoordinateFrame.hpp>
#include <proxy_library/visualization/GridNode.hpp>

namespace proxy_library
{   
    
class BaseType;
class Frame;
class FramePair;
class VBaseObject;

/**
 * Class uses osg to visualize the data.
 */
class Visualizer {
 public:    
    /**
     * \param type_infos Can be retrieved from ProxyLibrary and contains
     * additional content for each type like mutexes and new-data-available.
     */
    Visualizer(std::map<std::string, TypeInfos*> type_infos) : typeInfos(type_infos), 
        viewerThreadActive(true), vObjects() {
        osgRoot = new osg::Group();
        viewer.setSceneData(osgRoot);
        configureViewer(viewer);
        viewer.realize(); 
        viewerThread = std::thread(&startViewerFunc, this);
        
        osgRoot->addChild(new CoordinateFrame());
        osgRoot->addChild(new GridNode(20, 20, 1.0, 1.0, true));
    }
    
    ~Visualizer();
    
    /**
     * Adds a type to visualize.
     * \param type Data type to visualize.
     * \param type_name Type/port name of the data type. The name is used to identify
     * the correct type infos (data type mutex, new data available).
     */
    bool addType(std::string type_name, BaseType* type); 
    
    void updateTypes();
 
 protected:
    void configureViewer(osgViewer::Viewer& viewer);
    void* startViewer(Visualizer* visualizer);
    /**
     * Adds a visualization object and adds the osg object to the tree as
     * a child of camera 0.
     */
    bool addFrame(std::string type_name, TypeInfos* type_info, Frame* frame);
    
    /**
     * Adds a frame pair visualization object which contains two frames.
     */
    bool addFramePair(std::string type_name, TypeInfos* type_info, FramePair* frame_pair);
    
 private:
     std::map<std::string, TypeInfos*> typeInfos;
     osgViewer::Viewer viewer;
     osg::ref_ptr<osg::Group> osgRoot;
     std::thread viewerThread;
     bool viewerThreadActive;
     std::vector<VBaseObject*> vObjects;
     
     static void* startViewerFunc(Visualizer* visualizer) {
         return visualizer->startViewer(visualizer);
     }
    
};

} // end namespace proxy_library