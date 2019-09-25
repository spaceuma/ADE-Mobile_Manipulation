# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
set(ROCK_TEST_ENABLED 1)
cmake_minimum_required(VERSION 2.6)
include(${PROJECT_SOURCE_DIR}/cmake/Rock.cmake)
rock_init(proxy_library_sherpa_tt 0.1)
rock_standard_layout()
add_subdirectory(examples)
