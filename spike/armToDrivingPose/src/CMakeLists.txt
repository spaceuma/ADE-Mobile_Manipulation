find_package(PkgConfig REQUIRED)

include_directories(../../../deps/sherpa_tt_api)

pkg_check_modules(proxy_library REQUIRED proxy_library)
foreach(proxy_library_lib ${proxy_library_LIBRARIES})
          set(_proxy_library_lib NOTFOUND)
          find_library(_proxy_library_lib NAMES ${proxy_library_lib} HINTS ${proxy_library_LIBRARY_DIRS})
          if (NOT _proxy_library_lib)
            set(_proxy_library_lib ${proxy_library_lib})
          endif()
          list(APPEND _proxy_library_LIBRARIES ${_proxy_library_lib})
endforeach()
list(APPEND _proxy_library_LIBRARIES ${proxy_library_LDFLAGS_OTHER})
set(proxy_library_LIBRARIES ${_proxy_library_LIBRARIES} CACHE INTERNAL "")
add_definitions(${proxy_library_CFLAGS_OTHER})
include_directories(${proxy_library_INCLUDE_DIRS})

pkg_check_modules(proxy_library_sherpa_tt REQUIRED proxy_library_sherpa_tt)
foreach(proxy_library_sherpa_tt_lib ${proxy_library_sherpa_tt_LIBRARIES})
          set(_proxy_library_sherpa_tt_lib NOTFOUND)
          find_library(_proxy_library_sherpa_tt_lib NAMES ${proxy_library_sherpa_tt_lib} HINTS ${proxy_library_sherpa_tt_LIBRARY_DIRS})
          if (NOT _proxy_library_sherpa_tt_lib)
            set(_proxy_library_sherpa_tt_lib ${proxy_library_sherpa_tt_lib})
          endif()
          list(APPEND _proxy_library_sherpa_tt_LIBRARIES ${_proxy_library_sherpa_tt_lib})
endforeach()
list(APPEND _proxy_library_sherpa_tt_LIBRARIES ${proxy_library_sherpa_tt_LDFLAGS_OTHER})
set(proxy_library_sherpa_tt_LIBRARIES ${_proxy_library_sherpa_tt_LIBRARIES} CACHE INTERNAL "")
add_definitions(${proxy_library_sherpa_tt_CFLAGS_OTHER})
include_directories(${proxy_library_sherpa_tt_INCLUDE_DIRS})


add_library(api_types
            types/DEM.cpp
            types/DGPS.cpp
            types/Image.cpp
            types/IMU.cpp
            types/Joints.cpp
            types/JointState.cpp
            types/MotionCommand.cpp
            types/Pose.cpp
            types/Quaternion.cpp
            types/Vector3.cpp
            types/DEM.h
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
)

add_executable(ArmToDrivingPose
               Main.cpp
               Main.hpp
)

target_link_libraries(ArmToDrivingPose api_types pthread)
