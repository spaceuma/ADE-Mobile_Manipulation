rock_library(udp
    SOURCES Communication.cpp 
            communications/SequencedComm.cpp 
            Helpers.cpp
            network_adapters/NetworkAdapter.cpp 
            network_adapters/UDP.cpp 
    HEADERS
            Config.hpp
            ComInfos.hpp
            Helpers.hpp
            Enumerations.hpp
            Frame.hpp
            Containers.hpp
            Communication.hpp 
            communications/SequencedComm.hpp   
            network_adapters/NetworkAdapter.hpp 
            network_adapters/UDPAdapter.hpp 
            network_adapters/UDP.hpp
    DEPS_PKGCONFIG base-types
    DEPS_PKGCONFIG base-lib)
    
rock_executable(test_sequenced_comm tests/TestSequencedComm.cpp
    DEPS udp)

## Compiler flags, use for valgrind debugging.
#if(CMAKE_COMPILER_IS_GNUCXX)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall")        ## Optimize
#    # set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
#endif()
    
target_link_libraries (test_sequenced_comm pthread) 
