cmake_minimum_required(VERSION 3.5.1)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(armCollisionsViewer)

find_package(DART 6.6.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

include_directories(../../src)
include_directories(../../src/types/rg)
include_directories(data)
set(WP_DIR ../../deps/ESA-Trajectory_Control)

file(GLOB srcs "*.cpp" "*.hpp")
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
	"../../deps/UMA-Coupled_Control/coupledControl.hpp"
	"../../deps/UMA-Coupled_Control/coupledControl.cpp"
)  
add_executable(${PROJECT_NAME} ${srcs}
               #${MM_SRC}
               ${MM_types_SRC}
               ${MM_types_SRC_base}
               ${MM_types_SRC_proxy_library}
      	       ${MMML_SRC}
	             ${FM_SRC}
               ${WP_SRC}
	             #${CC_SRC}
               )
target_include_directories(${PROJECT_NAME} PRIVATE ../../src/types)
target_include_directories(${PROJECT_NAME} PRIVATE ../../src/types/base)
target_include_directories(${PROJECT_NAME} PRIVATE ../../src/types/proxy_library)
target_include_directories(${PROJECT_NAME} PRIVATE ../../src/mmFileManagerLib)
target_include_directories(${PROJECT_NAME} PRIVATE ../../deps/UMA-PathAndMotion_Planning)
target_include_directories(${PROJECT_NAME} PRIVATE ../../deps/ESA-Trajectory_Control)
#target_include_directories(${PROJECT_NAME} PRIVATE ../../deps/UMA-Coupled_Control)
target_link_libraries(${PROJECT_NAME} PUBLIC dart dart-utils-urdf dart-gui)
