project(MspaceOctree LANGUAGES CXX)

# ── PCL ──
find_package(PCL 1.14 REQUIRED COMPONENTS common io)

# ── 先构建 LASzip ──
add_subdirectory(libs/laszip)

add_library(MspaceOctree SHARED
    src/chunker_countsort_laszip.cpp
    src/indexer.cpp
    src/MSpaceNode.cpp
    src/MSpaceOCTree.cpp
    # 内部模块源文件
    modules/LasLoader/LasLoader.cpp
    modules/unsuck/unsuck_platform_specific.cpp
)

target_compile_definitions(MspaceOctree PRIVATE MSPACEOCTREE_LIB OT_LIBRARY_STATIC)

target_include_directories(MspaceOctree
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    # 内部模块头文件（供本库内部使用）
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/LasLoader
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/unsuck
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/tools
    # json.hpp
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs/json
    # laszip 头文件
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/laszip
    PUBLIC ${PCL_INCLUDE_DIRS}
)

target_link_libraries(MspaceOctree
    PUBLIC laszip
    PUBLIC unofficial::osg::osg unofficial::osg::osgDB unofficial::osg::osgUtil unofficial::osg::osgViewer
    PUBLIC ${PCL_LIBRARIES}
    PUBLIC ${libLAS_LIBRARY}
)

set_target_properties(MspaceOctree PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
)