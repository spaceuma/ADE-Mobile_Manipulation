cmake_minimum_required(VERSION 2.6)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )
find_package(DART 6.6.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(../../src)
include_directories(data)
set(WP_DIR ../../deps/ESA-Trajectory_Control)

file(GLOB MM_SRC
    "../../src/*.h"
    "../../src/*.cpp"
)

file(GLOB MM_types_SRC
    "../../src/types/*.h"
    "../../src/types/*.cpp"
)

file(GLOB MM_types_SRC_base
    "../../src/types/base/*.h"
    "../../src/types/base/*.hpp"
    "../../src/types/base/*.cpp"
)

file(GLOB MM_types_SRC_proxy_library
    "../../src/types/proxy_library/*.h"
    "../../src/types/proxy_library/*.hpp"
    "../../src/types/proxy_library/*.cpp"
)

file(GLOB MMML_SRC
    "../../src/mmFileManagerLib/*.h"
    "../../src/mmFileManagerLib/*.cpp"
)

file(GLOB FM_SRC
    "../../deps/UMA-PathAndMotion_Planning/*.h"
    "../../deps/UMA-PathAndMotion_Planning/*.cpp"
)

file(GLOB WP_SRC
    "../../deps/ESA-Trajectory_Control/WaypointNavigation.hpp"
    "../../deps/ESA-Trajectory_Control/WaypointNavigation.cpp"
)  

file(GLOB CC_SRC
    "../../deps/UMA-Coupled_Control/src/coupledControl.hpp"
    "../../deps/UMA-Coupled_Control/src/coupledControl.cpp"
)  

add_executable(harnessexample
               HarnessExample.cpp
               ${MM_SRC}
               ${MM_types_SRC}
               ${MM_types_SRC_base}
               ${MM_types_SRC_proxy_library}
	       ${MMML_SRC}
	       ${FM_SRC}
               ${WP_SRC}
	       ${CC_SRC})
target_include_directories(harnessexample PRIVATE ../../src/types)
target_include_directories(harnessexample PRIVATE ../../src/types/base)
target_include_directories(harnessexample PRIVATE ../../src/types/proxy_library)
target_include_directories(harnessexample PRIVATE ../../src/mmFileManagerLib)
target_include_directories(harnessexample PRIVATE ../../deps/UMA-PathAndMotion_Planning)
target_include_directories(harnessexample PRIVATE ../../deps/UMA-Coupled_Control/src)
target_include_directories(harnessexample PRIVATE ../../deps/ESA-Trajectory_Control)
target_link_libraries (harnessexample ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread Eigen3::Eigen dart dart-utils-urdf dart-gui)

