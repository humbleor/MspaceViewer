project(MainWindow LANGUAGES CXX)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ── Qt UI 文件 ──
set(UI_FILES
    resource/MainWindow.ui
    resource/TLSRegistration.ui
    resource/TLS-ULSRegistration.ui
    resource/OutlierRemovalRegistration.ui
    resource/ForestRegistration.ui
    resource/PositionRegistration.ui
)

# ── Forest_TLS_Reg 核心源文件（直接编译进 MainWindow）──
set(FOREST_REG_SOURCES
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/Hlp.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/HashRegObj.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/GenAlg.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/Point2img.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/FEC.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/CSF.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/Cloth.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/Constraint.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/Particle.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/Rasterization.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/XYZReader.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/point_cloud.cpp
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src/c2cdist.cpp
)

# ── 主程序源文件（不含 osgQt 重复文件，osgQt 作为独立库链接）──
set(MAIN_SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/FormSettings.cpp
    src/RegistrationForm_ULS.cpp
    src/RegistrationForm_TLS.cpp
    src/OutlierRemovalRegistration.cpp
    src/ForestRegistration.cpp
    src/PositionRegistration.cpp
    ${FOREST_REG_SOURCES}
    # src/TreeEvaluation.cpp  # Incomplete - depends on missing TreeEvaluation module
    # src/StemCurveEvaluation.cpp
)

set(MAIN_HEADERS
    include/MainWindow.h
    include/FormSettings.h
    include/RegistrationForm_ULS.h
    include/RegistrationForm_TLS.h
    include/OutlierRemovalRegistration.h
    include/ForestRegistration.h
    include/PositionRegistration.h
    # include/StemCurveEvaluation.h
    # include/TreeEvaluation.h
)

add_executable(MspaceViewer
    ${MAIN_SOURCES}
    ${MAIN_HEADERS}
    ${UI_FILES}
    resource/MainWindow.qrc
)

target_include_directories(MspaceViewer
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    # 相对路径 ../../osgQt/include/ 和 ../../MSpaceOctree/include/ 从此目录解析
    PRIVATE ${CMAKE_SOURCE_DIR}
    # Forest_TLS_Reg headers
    PRIVATE ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/include
    PRIVATE ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src
    # libLAS & OpenCV (found by root CMakeLists.txt)
    PRIVATE ${libLAS_INCLUDE_DIR}
    PRIVATE ${OpenCV_INCLUDE_DIRS}
    # zlib (transitive, for xlsx parser)
    PRIVATE ${ZLIB_INCLUDE_DIR}
)

target_link_libraries(MspaceViewer
    PRIVATE
        # Qt
        ${QT_LIB_PREFIX}::Widgets
        ${QT_LIB_PREFIX}::Concurrent
        ${QT_LIB_PREFIX}::Core
        ${QT_LIB_PREFIX}::Gui
        # OSG
        unofficial::osg::osg
        unofficial::osg::osgDB
        unofficial::osg::osgUtil
        unofficial::osg::osgGA
        unofficial::osg::osgViewer
        unofficial::osg::osgText
        unofficial::osg::osgVolume
        # 项目内部库
        osgQt
        MspaceOctree
        ULS-TLS
        D4PCR
        TLSRegistration
        # PCL（OutlierRemovalRegistration 直接使用 PCL）
        pcl_common pcl_io pcl_filters pcl_features pcl_registration pcl_search pcl_kdtree
        Eigen3::Eigen
        # Forest_TLS_Reg dependencies
        opencv_world
        ${libLAS_LIBRARY}
        ${libLAS_C_LIBRARY}
        Boost::regex Boost::serialization
        # zlib (for xlsx parser)
        ${ZLIB_LIBRARY_RELEASE}
        # small_gicp (header-only, embedded)
        small_gicp
        # OpenMP
        OpenMP::OpenMP_CXX
)

set_target_properties(MspaceViewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/resource"
)

# ── Position Registration Test ──
add_executable(test_position_reg
    ${CMAKE_SOURCE_DIR}/test/test_position_reg.cpp
    ${FOREST_REG_SOURCES}
)

target_include_directories(test_position_reg
    PRIVATE ${CMAKE_SOURCE_DIR}
    PRIVATE ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/include
    PRIVATE ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src
    PRIVATE ${ZLIB_INCLUDE_DIR}
    PRIVATE ${OpenCV_INCLUDE_DIRS}
    PRIVATE ${libLAS_INCLUDE_DIR}
    PRIVATE ${PCL_INCLUDE_DIRS}
    PRIVATE ${FLANN_INCLUDE_DIR}
)

target_link_libraries(test_position_reg
    PRIVATE
        pcl_common pcl_io pcl_filters pcl_registration pcl_search pcl_kdtree
        Eigen3::Eigen
        opencv_world
        ${ZLIB_LIBRARY_RELEASE}
        ${libLAS_LIBRARY}
        ${libLAS_C_LIBRARY}
        Boost::regex Boost::serialization
        small_gicp
        OpenMP::OpenMP_CXX
)

# ── Simple Parse Test ──
add_executable(test_parse_only
    ${CMAKE_SOURCE_DIR}/test/test_parse_only.cpp
)
target_include_directories(test_parse_only
    PRIVATE ${ZLIB_INCLUDE_DIR}
    PRIVATE ${PCL_INCLUDE_DIRS}
    PRIVATE ${FLANN_INCLUDE_DIR}
)
target_link_libraries(test_parse_only
    PRIVATE ${ZLIB_LIBRARY_RELEASE}
    PRIVATE Eigen3::Eigen
    PRIVATE pcl_common
)
set_target_properties(test_parse_only PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(test_position_reg PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ── 运行时拷贝翻译文件和 OSG 插件 ──
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resource"
        "$<TARGET_FILE_DIR:MspaceViewer>/resource"
    COMMENT "Copying resource files to output directory"
)

# ── 自动拷贝所有 transitive runtime DLL（OSG, libLAS, Qt 等）──
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_RUNTIME_DLLS:MspaceViewer>"
        "$<TARGET_FILE_DIR:MspaceViewer>"
    COMMAND_EXPAND_LISTS
    COMMENT "Copying runtime DLL dependencies..."
)

# ── 拷贝 Qt platform 插件 ──
get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION)
cmake_path(GET _qt_core_loc PARENT_PATH _qt_bin_dir)
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
        "${_qt_bin_dir}/../plugins/platforms"
        "$<TARGET_FILE_DIR:MspaceViewer>/platforms"
    COMMENT "Copying Qt platform plugin..."
)