cmake_minimum_required(VERSION 2.6)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(pgmMapreader 
               pgmMapreader.cpp)

target_link_libraries(pgmMapreader ${OpenCV_LIBS})
