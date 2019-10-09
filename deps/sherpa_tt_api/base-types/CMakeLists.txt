cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(base 1.0)

include_directories(BEFORE ${PROJECT_SOURCE_DIR})

rock_standard_layout()

