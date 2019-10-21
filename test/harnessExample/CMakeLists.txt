cmake_minimum_required (VERSION 3.1)

project (HarnessExample)

find_package(PkgConfig REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(../../src)

add_executable(harnessexample
               HarnessExample.cpp)
target_link_libraries (harnessexample Eigen3::Eigen)






