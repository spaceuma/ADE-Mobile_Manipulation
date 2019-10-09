#add_subdirectory(types)

rock_library(proxy_library_sherpa_tt_types
    SOURCES types/DEM.cpp
            types/DGPS.cpp
            types/Image.cpp
            types/IMU.cpp
            types/Joints.cpp
            types/JointState.cpp
            types/MotionCommand.cpp
            types/Pose.cpp
            types/Quaternion.cpp
            types/Vector3.cpp
    HEADERS types/DEM.h
            types/DGPS.h
            types/Image.h
            types/IMU.h
            types/Joints.h
            types/JointState.h
            types/MotionCommand.h
            types/Pose.h
            types/Quaternion.h
            types/Vector3.h
            types/Types.hpp
    DEPS_PKGCONFIG proxy_library
)


rock_library(proxy_library_sherpa_tt
    SOURCES ProxyLibrarySherpaTT.cpp
    HEADERS ProxyLibrarySherpaTT.hpp
    SOURCES ProxyLibrarySherpaTTSim.cpp
    HEADERS ProxyLibrarySherpaTTSim.hpp
    DEPS_PKGCONFIG proxy_library
    DEPS proxy_library_sherpa_tt_types
)

