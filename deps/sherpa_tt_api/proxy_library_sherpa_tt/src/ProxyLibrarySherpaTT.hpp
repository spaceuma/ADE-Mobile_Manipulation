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
class ProxyLibrarySherpaTT : public ProxyLibrary {
     
 public:
     /**
      * Currently a fix test configuration is used.
      */
    ProxyLibrarySherpaTT(Config config);
    
    ~ProxyLibrarySherpaTT();
    
    /*
    virtual bool getDEM(DEM& dem) {
        return getDataType("map_in", dem);
    }*/

    virtual bool getDGPS(DGPS& dgps) {
        return getDataType("gps_in", dgps);;
    }

    virtual bool getDGPSPose(Pose& pose) {
        return getDataType("gps_pose_in", pose);;
    }

    /*
    virtual bool getFrontalCameraImage(Image& img){
        return getDataType("frontal_camera_image_in", img);;
    }*/

    virtual bool getGripperCameraImage(Image& img) {
        return getDataType("gripper_camera_image_in", img);;
    }

    virtual bool getIMUData(IMU& imu) {
        return getDataType("imu_in", imu);;
    }

    virtual bool getManipulatorJointState(Joints& joints) {
        return getDataType("manipulator_joints_in", joints);;
    }

    virtual bool getMobileBaseJointState(Joints& joints) {
        return getDataType("mobile_base_joints_in", joints);;
    }

    virtual bool getPose(Pose& pose) {
        return getDataType("pose_in", pose);
    }

    /*
    virtual bool getSLAMPose(Pose& pose) {
        return getDataType("slam_pose_in", pose);;
    }*/

    virtual bool getOdometry(Pose& pose) {
        return getDataType("odometry_in", pose);
    }

    virtual bool sendMotionCommand(MotionCommand* eIngCommand) {
        return sendData("motion_out", eIngCommand);
    }

    virtual bool sendManipulatorJointCommand(Joints* eIngCommand) {
        return sendData("manipulator_joints_out", eIngCommand);
    }
};

} // end namespace proxy_library
