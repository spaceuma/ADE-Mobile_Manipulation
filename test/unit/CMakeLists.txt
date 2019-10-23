cmake_minimum_required(VERSION 2.6)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(../../src)


file(GLOB MM_SRC
    "../../src/*.h"
    "../../src/*.cpp"
)

file(GLOB MM_types_SRC
    "../../src/types/*.h"
    "../../src/types/*.cpp"
)  

# Link runTests with what we want to test and the GTest and pthread library
add_executable(runUnitTests 
               UnitTests.cpp 
               MobileManipMotionPlannerTests.cpp
               ${MM_SRC}
               ${MM_types_SRC})
target_include_directories(runUnitTests PRIVATE ../../src/types)
target_link_libraries(runUnitTests ${GTEST_LIBRARIES} pthread Eigen3::Eigen)
