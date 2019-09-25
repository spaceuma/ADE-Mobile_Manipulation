#include <proxy_library/visualization/Visualizer.hpp>

#include <proxy_library/types/BaseType.hpp>
#include <proxy_library/types/Types.hpp>

#include <proxy_library/visualization/VBaseObject.hpp>
#include <proxy_library/visualization/VPointcloud.hpp>
#include <proxy_library/visualization/VFrame.hpp>
#include <proxy_library/visualization/VFramePair.hpp>
#include <proxy_library/visualization/VPose.hpp>

namespace proxy_library 
{
    
Visualizer::~Visualizer() {
    viewerThreadActive = false;
    viewerThread.join();
    
    std::vector<VBaseObject*>::iterator it = vObjects.begin();
    for(; it != vObjects.end(); ++it) {
        delete (*it);
    }
    vObjects.clear();
}
    
bool Visualizer::addType(std::string type_name, BaseType* type) {
    std::map<std::string, TypeInfos*>::iterator it_info = typeInfos.find(type_name);
    if(it_info == typeInfos.end()) {
        std::cerr << "No type informations have been found for " << type_name << std::endl;
        return false;
    }
    
    // Depending on the passed type a visualizer object is created and added to root.
    Pointcloud* p = dynamic_cast<Pointcloud*>(type);
    if(p != NULL) {
        VPointcloud* vp = new VPointcloud(type_name, it_info->second, p, false);
        vObjects.push_back(vp);
        osgRoot->addChild(vp);
        return true;
    }
    
    Pose* pose = dynamic_cast<Pose*>(type);
    if(pose != NULL) {
        VPose* vpose = new VPose(type_name, it_info->second, pose, 0.2);
        vObjects.push_back(vpose);
        osgRoot->addChild(vpose);
        return true;
    }
    
    Frame* f = dynamic_cast<Frame*>(type);
    if(f != NULL) {
        bool ret = addFrame(type_name, it_info->second, f);
        return ret;
    }
    
    FramePair* fp = dynamic_cast<FramePair*>(type);
    if(fp != NULL) {
        // Both images use the same port-name and therefore the same mutex.
        bool ret = addFramePair(type_name, it_info->second, fp);
        return ret;
    }
    
    return false;
}

void Visualizer::updateTypes() {
    // Update data.
    std::vector<VBaseObject*>::iterator it_vobj = vObjects.begin();
    for(; it_vobj != vObjects.end(); ++it_vobj) {
        TypeInfos* type_info = (*it_vobj)->getTypeInfo();
        if(type_info->newInformations) {
            type_info->mutex.lock();
            (*it_vobj)->updateData();
            type_info->mutex.unlock();
            type_info->newInformations = false;
        } 
    }
}

// PROTECTED
void Visualizer::configureViewer(osgViewer::Viewer& viewer) {
    viewer.setUpViewInWindow(0, 0, 800, 600); 
    viewer.setCameraManipulator(new osgGA::SphericalManipulator);
    viewer.setRunFrameScheme(osgViewer::Viewer::ON_DEMAND);
}

void* Visualizer::startViewer(Visualizer* visualizer) {
    
    while(!viewer.done() && viewerThreadActive) {
        viewer.frame();
    }
    
    //viewer.run();
    return NULL;
}

bool Visualizer::addFrame(std::string type_name, TypeInfos* type_info, Frame* frame) {
    VFrame* vf = new VFrame(type_name, type_info, frame);
    vObjects.push_back(vf);
    // Frames are attached to the camera so they can be used as a HUD.
    std::vector<osg::Camera*> cameras;
    viewer.getCameras(cameras);
    if(cameras.size() < 1) {
        std::cerr << "Viewer does not contain any cameras" << std::endl;
        return false;
    } 
    cameras[0]->addChild(vf);
    return true;
}

bool Visualizer::addFramePair(std::string type_name, TypeInfos* type_info, 
                              FramePair* frame_pair) {
    VFramePair* vfp = new VFramePair(type_name, type_info, frame_pair);
    vObjects.push_back(vfp);
    // Frames are attached to the camera so they can be used as a HUD.
    std::vector<osg::Camera*> cameras;
    viewer.getCameras(cameras);
    if(cameras.size() < 1) {
        std::cerr << "Viewer does not contain any cameras" << std::endl;
        return false;
    } 
    // The group visualization framepair already contains both visualization frames.
    // cameras[0] -- frame_pair -- frame1
    //                          \  frame2
    cameras[0]->addChild(vfp);
    return true;
}
    
    
} // end namespace proxy_library