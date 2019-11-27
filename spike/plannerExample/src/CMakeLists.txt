rock_library(biFMplanner_lib
    SOURCES biFMplanner.cpp
    HEADERS biFMplanner.hpp)

TARGET_LINK_LIBRARIES(biFMplanner_lib pthread)

rock_library(sherpa_types
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

rock_executable(biFMplanner_example Main.cpp
    DEPS biFMplanner_lib
    DEPS sherpa_types
    DEPS_PKGCONFIG proxy_library_sherpa_tt)

