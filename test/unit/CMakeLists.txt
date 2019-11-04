cmake_minimum_required(VERSION 2.6)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(../../src)
include_directories(data)
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

# Link runTests with what we want to test and the GTest and pthread library
add_executable(runUnitTests 
               UnitTests.cpp 
               MobileManipMotionPlannerTests.cpp
               MobileManipMapTests.cpp
               MotionPlanTests.cpp
               ${MM_SRC}
               ${MM_types_SRC}
               ${WP_SRC})
target_include_directories(runUnitTests PRIVATE ../../src/types)
target_include_directories(runUnitTests PRIVATE ../../deps/control-waypoint_navigation)
target_link_libraries(runUnitTests ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread Eigen3::Eigen)
