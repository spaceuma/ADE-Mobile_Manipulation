rock_library(
    base-types 
        Time.cpp

    HEADERS
        Time.hpp

    DEPS_PKGCONFIG 
        base-logging
)

configure_file(base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
