#pragma once

#include <map>
#include <thread>

#include <proxy_library/ProxyLibrary.hpp>
#include <proxy_library_sherpa_tt/types/Types.hpp>

namespace proxy_library
{    

/**
 * Proxy communication class which reads (udp::SequencedComm) incoming proxy data
 * and which deserializes the received data types.
 */
class ProxyLibrarySherpaTTSim : public ProxyLibrary {
     
 public:
     /**
      * Currently a fix test configuration is used.
      */
    ProxyLibrarySherpaTTSim(Config config);
    
    ~ProxyLibrarySherpaTTSim();
    
    
    virtual bool getSimPTUCameraImageLeft(Image& img) {
        return getDataType("mastcam_ptu_camera_left_image_in", img);
    }

    virtual bool getSimPTUCameraImageRight(Image& img) {
        return getDataType("mastcam_ptu_camera_right_image_in", img);
    }

    virtual bool getSimFixCameraImageLeft(Image& img) {
        return getDataType("mastcam_fixed_camera_left_image_in", img);
    }

    virtual bool getSimFixCameraImageRight(Image& img) {
        return getDataType("mastcam_fixed_camera_right_image_in", img);
    }

    /*
    virtual bool getSimHRESCameraImage(Image& img) {
        return getDataType("avionics_box_hres_camera_image_in", img);
    }

    virtual bool getSimIRCameraImage(Image& img) {
        return getDataType("avionics_box_ir_camera_image_in", img);
    }*/

    virtual bool getSimPTUJointsState(Joints& joints) {
        return getDataType("mastcam_ptu_joints_in", joints);
    }

    virtual bool sendSimPTUJointsCommand(Joints* jointsCommand) {
        return sendData("mastcam_ptu_joints_out", jointsCommand);
    }

};

} // end namespace proxy_library
