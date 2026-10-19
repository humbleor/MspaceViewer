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
    ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/ForestPairRegistration.cpp
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
    include/AsyncRegistration.h
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

# Keep the embedded catalogs synchronized with TS sources in every build.
get_target_property(_translation_qt_core Qt6::Core IMPORTED_LOCATION)
cmake_path(GET _translation_qt_core PARENT_PATH _translation_qt_bin)
find_program(MSPACE_LRELEASE NAMES lrelease.exe lrelease HINTS "${_translation_qt_bin}" REQUIRED)
set(_mspace_translation_files)
foreach(_language zh en)
    set(_ts "${CMAKE_CURRENT_SOURCE_DIR}/resource/MainWindow_${_language}.ts")
    set(_qm "${CMAKE_CURRENT_SOURCE_DIR}/resource/MainWindow_${_language}.qm")
    add_custom_command(OUTPUT "${_qm}"
        COMMAND "${MSPACE_LRELEASE}" "${_ts}" -qm "${_qm}"
        DEPENDS "${_ts}"
        VERBATIM)
    list(APPEND _mspace_translation_files "${_qm}")
endforeach()
add_custom_target(MspaceTranslations DEPENDS ${_mspace_translation_files})
add_dependencies(MspaceViewer MspaceTranslations)
set_property(TARGET MspaceViewer APPEND PROPERTY AUTOGEN_TARGET_DEPENDS MspaceTranslations)

if(D4PCR_ENABLE_GLOBAL_OPTIMIZATION)
    add_dependencies(MspaceViewer D4PCRGlobalOptimization)
endif()

option(FOREST_ENABLE_GLOBAL_OPTIMIZATION "Build Forest + GICP + GTSAM global registration" ON)
if(FOREST_ENABLE_GLOBAL_OPTIMIZATION)
    if(NOT TARGET gtsam)
        find_package(GTSAM CONFIG REQUIRED)
    endif()
    add_executable(ForestGlobalOptimization
        ${CMAKE_SOURCE_DIR}/cli/ForestGlobalOptimizationMain.cpp
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/ForestGlobalRegistration.cpp
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/ForestGlobalIO.cpp
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/src/utils/ForestLasExport.cpp
        ${FOREST_REG_SOURCES})
    target_include_directories(ForestGlobalOptimization PRIVATE
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/include
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src
        ${libLAS_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
        ${PCL_ROOT}/3rdParty/FLANN/include)
    target_link_libraries(ForestGlobalOptimization PRIVATE
        Qt6::Core gtsam small_gicp opencv_world
        PCL::common PCL::io PCL::filters PCL::features PCL::registration
        PCL::search PCL::kdtree PCL::segmentation
        ${libLAS_LIBRARY} ${libLAS_C_LIBRARY} Boost::regex Boost::serialization OpenMP::OpenMP_CXX)
    set_target_properties(ForestGlobalOptimization PROPERTIES
        AUTOMOC OFF AUTOUIC OFF AUTORCC OFF
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>/runtime")
    if(MSVC)
        target_compile_options(ForestGlobalOptimization PRIVATE /utf-8 /bigobj)
    endif()
    target_compile_definitions(MspaceViewer PRIVATE FOREST_HAS_GLOBAL_OPTIMIZATION=1)
    add_dependencies(MspaceViewer ForestGlobalOptimization)
    install(TARGETS ForestGlobalOptimization RUNTIME DESTINATION runtime)
endif()

if(MSVC)
    set_source_files_properties(src/OutlierRemovalRegistration.cpp src/ForestRegistration.cpp
        PROPERTIES COMPILE_OPTIONS "/utf-8")
endif()

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
        # PCL — use imported targets so TARGET_RUNTIME_DLLS can resolve DLLs
        PCL::common PCL::io PCL::filters PCL::features PCL::registration PCL::search PCL::kdtree
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

option(BUILD_D4PCR_GUI_SMOKE "Build the D4PCR GUI integration smoke test" OFF)

if(D4PCR_ENABLE_GLOBAL_OPTIMIZATION AND BUILD_D4PCR_GUI_SMOKE)
    add_executable(D4PCRGuiSmoke EXCLUDE_FROM_ALL
        ${CMAKE_SOURCE_DIR}/D4PCR/test/src/GuiIntegrationSmoke.cpp
        src/OutlierRemovalRegistration.cpp
        include/OutlierRemovalRegistration.h
        resource/OutlierRemovalRegistration.ui)
    target_include_directories(D4PCRGuiSmoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(D4PCRGuiSmoke PRIVATE D4PCR Qt6::Widgets Qt6::Concurrent)
    add_dependencies(D4PCRGuiSmoke D4PCRGlobalOptimization)
    set_target_properties(D4PCRGuiSmoke PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
endif()



# ── 运行时拷贝翻译文件和 OSG 插件 ──
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resource"
        "$<TARGET_FILE_DIR:MspaceViewer>/resource"
    COMMENT "Copying resource files to output directory"
)

# ── 自动拷贝所有 transitive runtime DLL（OSG, libLAS, Qt 等）──
# Filter out .lib import libraries that TARGET_RUNTIME_DLLS may include on Windows
set(_RUNTIME_DLLS "$<TARGET_RUNTIME_DLLS:MspaceViewer>")
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<LIST:FILTER,${_RUNTIME_DLLS},EXCLUDE,.*\\.lib$>"
        "$<TARGET_FILE_DIR:MspaceViewer>"
    COMMAND_EXPAND_LISTS
    COMMENT "Copying runtime DLL dependencies..."
)

# GTSAM links against vcpkg Boost; PCL carries another Boost build. Copy the
# vcpkg release DLLs too, including dependencies hidden by PCL's imported targets.
if(D4PCR_ENABLE_GLOBAL_OPTIMIZATION OR FOREST_ENABLE_GLOBAL_OPTIMIZATION)
    if(VCPKG_INSTALLED_DIR)
        set(_d4pcr_runtime_root "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}")
    elseif(GTSAM_RELEASE_ROOT)
        set(_d4pcr_runtime_root "${GTSAM_RELEASE_ROOT}")
    else()
        set(_d4pcr_runtime_root "F:/vcpkg/installed/x64-windows-release")
    endif()
    file(GLOB _d4pcr_runtime_dlls CONFIGURE_DEPENDS "${_d4pcr_runtime_root}/bin/*.dll")
    if(_d4pcr_runtime_dlls)
        add_custom_command(TARGET MspaceViewer POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_d4pcr_runtime_dlls} "$<TARGET_FILE_DIR:MspaceViewer>"
            COMMAND_EXPAND_LISTS
            COMMENT "Copying GTSAM and vcpkg runtime dependencies...")
        install(FILES ${_d4pcr_runtime_dlls} DESTINATION .)
    endif()
endif()

# ── 拷贝 PCL 间接依赖 DLL（未通过链接链传递的）──
set(_PCL_MISSING_DLLS
    "${PCL_ROOT}/bin/pcl_segmentation.dll"
    "${PCL_ROOT}/bin/pcl_io_ply.dll"
    "${PCL_ROOT}/bin/pcl_octree.dll"
    "${PCL_ROOT}/bin/pcl_sample_consensus.dll"
    "${PCL_ROOT}/bin/pcl_ml.dll"
    "C:/Program Files/OpenNI2/Redist/OpenNI2.dll"
)
list(FILTER _PCL_MISSING_DLLS EXCLUDE REGEX "^$")
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${_PCL_MISSING_DLLS}
        "$<TARGET_FILE_DIR:MspaceViewer>"
    COMMAND_EXPAND_LISTS
    COMMENT "Copying PCL indirect dependency DLLs (segmentation, io_ply, OpenNI2)..."
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
