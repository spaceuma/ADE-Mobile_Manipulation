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
    "../../src/planner/*.h"
    "../../src/planner/*.cpp"
)

file(GLOB WP_SRC
    "../../deps/control-waypoint_navigation/WaypointNavigation.hpp"
    "../../deps/control-waypoint_navigation/WaypointNavigation.cpp"
)  

file(GLOB CC_SRC
    "../../deps/coupled_control/coupledControl.hpp"
    "../../deps/coupled_control/coupledControl.cpp"
)  

# Link runTests with what we want to test and the GTest and pthread library
add_executable(runUnitTests 
               UnitTests.cpp 
               MobileManipMapTests.cpp
	       MotionPlanTests.cpp
	       MobileManipExecutorTests.cpp
               ${MM_SRC}
               ${MM_types_SRC}
               ${MM_types_SRC_base}
               ${MM_types_SRC_proxy_library}
	       ${MMML_SRC}
	       ${FM_SRC}
               ${WP_SRC}
	       ${CC_SRC})
target_include_directories(runUnitTests PRIVATE ../../src/types)
target_include_directories(runUnitTests PRIVATE ../../src/types/base)
target_include_directories(runUnitTests PRIVATE ../../src/types/proxy_library)
target_include_directories(runUnitTests PRIVATE ../../src/mmFileManagerLib)
target_include_directories(runUnitTests PRIVATE ../../src/planner)
target_include_directories(runUnitTests PRIVATE ../../deps/control-waypoint_navigation)
target_include_directories(runUnitTests PRIVATE ../../deps/coupled_control)
target_link_libraries(runUnitTests ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread Eigen3::Eigen)
