cmake_minimum_required (VERSION 3.1)

project (HarnessExample)

find_package(PkgConfig REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(MM_DIRECTORY ../../src)

include_directories(${MM_DIRECTORY})


file(GLOB MM_SRC
    "../../src/*.h"
    "../../src/*.cpp"
)

file(GLOB MM_types_SRC
    "../../src/types/*.h"
    "../../src/types/*.cpp"
)  

add_executable(harnessexample
               HarnessExample.cpp
               ${MM_SRC}
               ${MM_types_SRC})
target_include_directories(harnessexample PRIVATE ../../src/types)
target_link_libraries (harnessexample Eigen3::Eigen)






