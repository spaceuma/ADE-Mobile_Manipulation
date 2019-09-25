#include "ProxyLibrarySherpaTT.hpp"

#include <sys/time.h>
#include <proxy_library_sherpa_tt/types/Types.hpp>

namespace proxy_library 
{

    // PUBLIC
    ProxyLibrarySherpaTT::ProxyLibrarySherpaTT(Config config) : ProxyLibrary(config) {
        //addType("map_in", new DEM);
        addType("gps_in", new DGPS);
        addType("gps_pose_in", new Pose);
        //addType("frontal_camera_image_in", new Image);
        addType("gripper_camera_image_in", new Image);
        addType("imu_in", new IMU);
        addType("manipulator_joints_in", new Joints);
        addType("mobile_base_joints_in", new Joints);
        //addType("motion_out", new MotionCommand, udp::FRAME_ACK);
        addType("motion_out", new MotionCommand);
        addType("pose_in", new Pose);
        //addType("slam_pose_in", new Pose);
        addType("odometry_in", new Pose);

        // What about manipulator_joints_out?
    }

    ProxyLibrarySherpaTT::~ProxyLibrarySherpaTT() {
    }

} // end namespace proxy_library
