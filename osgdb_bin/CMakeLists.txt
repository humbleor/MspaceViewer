project(osgdb_bin LANGUAGES CXX)

# OSG 插件需要命名为 osgdb_<extension>
add_library(osgdb_bin SHARED
    src/osgdb_bin.cpp
)

target_compile_definitions(osgdb_bin PRIVATE OSDBD_BIN_LIB OT_LIBRARY_STATIC)

target_include_directories(osgdb_bin
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    # unsuck.hpp 通过相对路径引用
    PRIVATE ${CMAKE_SOURCE_DIR}/MspaceOctree/modules/unsuck
)

target_link_libraries(osgdb_bin
    PUBLIC unofficial::osg::osg unofficial::osg::osgDB unofficial::osg::osgUtil
)

# OSG 插件输出到版本化目录（OSG 默认在此查找插件）
set_target_properties(osgdb_bin PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug/osgPlugins-3.6.5"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release/osgPlugins-3.6.5"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin/RelWithDebInfo/osgPlugins-3.6.5"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin/MinSizeRel/osgPlugins-3.6.5"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug/osgPlugins-3.6.5"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release/osgPlugins-3.6.5"
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin/RelWithDebInfo/osgPlugins-3.6.5"
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin/MinSizeRel/osgPlugins-3.6.5"
)