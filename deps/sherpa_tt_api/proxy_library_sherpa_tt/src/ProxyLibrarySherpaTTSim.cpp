#include "ProxyLibrarySherpaTTSim.hpp"

#include <sys/time.h>
#include <proxy_library_sherpa_tt/types/Types.hpp>

namespace proxy_library 
{

    // PUBLIC
    ProxyLibrarySherpaTTSim::ProxyLibrarySherpaTTSim(Config config) : ProxyLibrary(config) {

        addType("mastcam_ptu_camera_left_image_in", new Image);
        addType("mastcam_ptu_camera_right_image_in", new Image);
        addType("mastcam_fixed_camera_left_image_in", new Image);
        addType("mastcam_fixed_camera_right_image_in", new Image);
        //addType("avionics_box_hres_camera_image_in", new Image);
        //addType("avionics_box_ir_camera_image_in", new Image);
        addType("mastcam_ptu_joints_in", new Joints);
        //addType("mastcam_ptu_joints_cmd_out", new Joints);
    }

    ProxyLibrarySherpaTTSim::~ProxyLibrarySherpaTTSim() {
    }

} // end namespace proxy_library
