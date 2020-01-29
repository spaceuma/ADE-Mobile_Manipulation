cmake_minimum_required(VERSION 3.1)

project(ADE_MobileManipulation VERSION 0.5
                               DESCRIPTION " ADE (OG10) library for Mobile Manipulation purposes "
                               LANGUAGES CXX)
# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(src)
set(WP_DIR deps/control-waypoint_navigation)

file(GLOB MM_SRC
    "src/*.h"
    "src/*.cpp"
)

file(GLOB MM_types_SRC
    "src/types/*.h"
    "src/types/*.cpp"
)

file(GLOB MMML_SRC
    "src/mmFileManagerLib/*.h"
    "src/mmFileManagerLib/*.cpp"
)

file(GLOB FM_SRC
    "src/planner/*.h"
    "src/planner/*.cpp"
)

file(GLOB WP_SRC
    "deps/control-waypoint_navigation/WaypointNavigation.hpp"
    "deps/control-waypoint_navigation/WaypointNavigation.cpp"
)  

file(GLOB CC_SRC
    "deps/coupled_control/coupledControl.hpp"
    "deps/coupled_control/coupledControl.cpp"
)  

add_library(mmlib
            ${MM_SRC}
            ${MM_types_SRC}
	    ${MMML_SRC}
	    ${FM_SRC}
            ${WP_SRC}
	    ${CC_SRC})

add_executable(harnesstest
               test/harnessExample/HarnessExample.cpp)

target_include_directories(mmlib PUBLIC src/types)
target_include_directories(mmlib PUBLIC src/mmFileManagerLib)
target_include_directories(mmlib PUBLIC src/planner)
target_include_directories(mmlib PUBLIC deps/coupled_control)
target_include_directories(mmlib PUBLIC deps/control-waypoint_navigation)
target_link_libraries (mmlib ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread Eigen3::Eigen)

target_link_libraries(harnesstest mmlib)

