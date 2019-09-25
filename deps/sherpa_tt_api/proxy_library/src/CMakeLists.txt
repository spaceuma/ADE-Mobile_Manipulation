find_package(osg)

if($OSG_FOUND)
    rock_library(proxy_library_visualization
        HEADERS visualization/Visualizer.hpp 
                visualization/VBaseObject.hpp
                visualization/content/CoordinateFrame.hpp
                visualization/content/GridNode.hpp
        SOURCES visualization/Visualizer.cpp 
                visualization/VBaseObject.cpp
                visualization/content/CoordinateFrame.cpp
                visualization/content/GridNode.cpp
    )

    TARGET_LINK_LIBRARIES(proxy_library_visualization pthread osg osgViewer)
endif($OSG_FOUND)

rock_library(proxy_library
    HEADERS ProxyLibrary.hpp
            Config.hpp
            DataType.hpp
            types/BaseType.hpp
    SOURCES ProxyLibrary.cpp
    DEPS_PKGCONFIG detached_logger
    DEPS_PKGCONFIG udp
)

if($OSG_FOUND)
    TARGET_LINK_LIBRARIES(proxy_library proxy_library_visualization pthread osgGA)
else($OSG_FOUND)
    TARGET_LINK_LIBRARIES(proxy_library pthread)
endif($OSG_FOUND)

rock_executable(proxy_library_test ${PROJECT_SOURCE_DIR}/test/proxy_library_test.cpp
    DEPS proxy_library)

