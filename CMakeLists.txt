cmake_minimum_required(VERSION 2.6)

# Locate GTest
find_package(GTest)
include_directories(${GTEST_INCLUDE_DIRS})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )
find_package(DART 6.6.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(src)
include_directories(data)
include_directories(src/types/rg)
set(WP_DIR deps/control-waypoint_navigation)

file(GLOB MM_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB MM_types_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/*.cpp"
)

file(GLOB MM_types_SRC_base
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/base/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/base/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/base/*.cpp"
)

file(GLOB MM_types_SRC_proxy_library
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/proxy_library/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/proxy_library/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/types/proxy_library/*.cpp"
)

file(GLOB MMML_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mmFileManagerLib/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mmFileManagerLib/*.cpp"
)

file(GLOB FM_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/planner/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/planner/*.cpp"
)

file(GLOB WP_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/ESA-Trajectory_Control/WaypointNavigation.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/ESA-Trajectory_Control/WaypointNavigation.cpp"
)  

file(GLOB UMA_DEPS_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/UMA-PathAndMotion_Planning/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/UMA-PathAndMotion_Planning/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/UMA-Coupled_Control/src/coupledControl.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/UMA-Coupled_Control/src/coupledControl.cpp"
)


add_library(mobilemanipulation SHARED
    ${MM_SRC}
    ${MM_types_SRC}
    ${MM_types_SRC_base}
    ${MMML_SRC}
    ${FM_SRC}
    ${WP_SRC}
    ${UMA_DEPS_SRC}
)

MESSAGE(STATUS "Files included:    
-- MM_SRC: ${MM_SRC}
-- MM_types_SRC: ${MM_types_SRC}
-- MM_types_SRC_base: ${MM_types_SRC_base}
-- MMML_SRC: ${MMML_SRC}
-- FM_SRC: ${FM_SRC}
-- WP_SRC: ${WP_SRC}
-- UMA_DEPS_SRC: ${UMA_DEPS_SRC}")

set(MM_INCLUDEPATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/types
    ${CMAKE_CURRENT_SOURCE_DIR}/src/types/base
    ${CMAKE_CURRENT_SOURCE_DIR}/src/types/proxy_library
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mmFileManagerLib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/UMA-PathAndMotion_Planning
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/UMA-Coupled_Control/src
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ESA-Trajectory_Control
)
target_include_directories(mobilemanipulation PRIVATE ${MM_INCLUDEPATHS})
target_link_libraries (mobilemanipulation ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread Eigen3::Eigen dart dart-utils-urdf dart-gui)

