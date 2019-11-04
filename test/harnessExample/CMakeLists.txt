cmake_minimum_required (VERSION 3.1)

project (HarnessExample)

find_package(PkgConfig REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )

set(MM_DIRECTORY ../../src)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${MM_DIRECTORY})
set(WP_DIR ../../deps/control-waypoint_navigation)


file(GLOB MM_SRC
    "../../src/*.h"
    "../../src/*.cpp"
)

file(GLOB MM_types_SRC
    "../../src/types/*.h"
    "../../src/types/*.cpp"
)  

file(GLOB WP_SRC
    "../../deps/control-waypoint_navigation/WaypointNavigation.hpp"
    "../../deps/control-waypoint_navigation/WaypointNavigation.cpp"
) 

add_executable(harnessexample
               HarnessExample.cpp
               ${MM_SRC}
               ${MM_types_SRC}
               ${WP_SRC})
target_include_directories(harnessexample PRIVATE ../../src/types)
target_include_directories(harnessexample PRIVATE ../../deps/control-waypoint_navigation)
target_link_libraries (harnessexample Eigen3::Eigen ${OpenCV_LIBS})






