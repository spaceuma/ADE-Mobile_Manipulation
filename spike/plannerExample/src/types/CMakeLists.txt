rock_library(proxy_library_sherpa_tt_types
    SOURCES DEM.cpp
            DGPS.cpp
            Image.cpp
            IMU.cpp
            Joints.cpp
            JointState.cpp
            MotionCommand.cpp
            Pose.cpp
            Quaternion.cpp
            Vector3.cpp
    HEADERS DEM.h
            DGPS.h
            Image.h
            IMU.h
            Joints.h
            JointState.h
            MotionCommand.h
            Pose.h
            Quaternion.h
            Vector3.h
    DEPS_PKGCONFIG proxy_library
)

