project(osgQt LANGUAGES CXX)

add_library(osgQt SHARED
    src/CullVisitorEx.cpp
    src/GraphicsWindowEx.cpp
    src/osgQOpenGLWidget.cpp
    src/OSGRenderer.cpp
    src/RenderStageEx.cpp
    src/StateEx.cpp
    # Headers with Q_OBJECT - needed for AUTOMOC
    include/osgQOpenGLWidget.h
    include/OSGRenderer.h
)

target_compile_definitions(osgQt PRIVATE OSGQT_LIB)

target_include_directories(osgQt
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(osgQt
    PUBLIC
        OpenSceneGraph::osg
        OpenSceneGraph::osgDB
        OpenSceneGraph::osgUtil
        OpenSceneGraph::osgGA
        OpenSceneGraph::osgViewer
        OpenThreads::OpenThreads
        ${QT_LIB_PREFIX}::Widgets
        ${QT_LIB_PREFIX}::OpenGL
)

set_target_properties(osgQt PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    # Per-config output dirs for Windows
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
)