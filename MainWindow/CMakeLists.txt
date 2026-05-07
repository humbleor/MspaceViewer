project(MainWindow LANGUAGES CXX)

# ── Qt UI 文件 ──
set(UI_FILES
    resource/MainWindow.ui
    resource/TLSRegistration.ui
    resource/TLS-ULSRegistration.ui
    resource/OutlierRemovalRegistration.ui
)

# ── 主程序源文件（不含 osgQt 重复文件，osgQt 作为独立库链接）──
set(MAIN_SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/FormSettings.cpp
    src/RegistrationForm_ULS.cpp
    src/RegistrationForm_TLS.cpp
    src/OutlierRemovalRegistration.cpp
    # src/TrreeEvaluation.cpp  # Incomplete - depends on missing TreeEvaluation module
    # src/StemCurveEvaluation.cpp
)

set(MAIN_HEADERS
    include/MainWindow.h
    include/FormSettings.h
    include/RegistrationForm_ULS.h
    include/RegistrationForm_TLS.h
    include/OutlierRemovalRegistration.h
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
)

target_link_libraries(MspaceViewer
    PRIVATE
        # Qt
        ${QT_LIB_PREFIX}::Widgets
        ${QT_LIB_PREFIX}::Concurrent
        ${QT_LIB_PREFIX}::Core
        ${QT_LIB_PREFIX}::Gui
        # OSG
        OpenSceneGraph::osg
        OpenSceneGraph::osgDB
        OpenSceneGraph::osgUtil
        OpenSceneGraph::osgGA
        OpenSceneGraph::osgViewer
        OpenSceneGraph::osgText
        OpenSceneGraph::osgVolume
        # 项目内部库
        osgQt
        MspaceOctree
        ULS-TLS
        D4PCR
        TLSRegistration
        # PCL（OutlierRemovalRegistration 直接使用 PCL）
        PCL::common PCL::io PCL::filters PCL::features PCL::registration PCL::search
        Eigen3::Eigen
)

set_target_properties(MspaceViewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/resource"
)

# ── 运行时拷贝翻译文件和 OSG 插件 ──
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resource"
        "$<TARGET_FILE_DIR:MspaceViewer>/resource"
    COMMENT "Copying resource files to output directory"
)

# ── 拷贝 vcpkg 所有 runtime DLL（Release/Debug 分别从对应目录拷贝）──
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND cmd /c "copy /y \"$<IF:$<CONFIG:Debug>,${OSG_BIN_DIR_DEBUG},${OSG_BIN_DIR}>\\*.dll\" \"$<TARGET_FILE_DIR:MspaceViewer>\" > nul"
    COMMENT "Copying vcpkg runtime DLLs..."
)

# ── 拷贝 Qt platform 插件 ──
add_custom_command(TARGET MspaceViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "$<IF:$<CONFIG:Debug>,${OSG_BIN_DIR_DEBUG}/../plugins/platforms,${OSG_BIN_DIR}/../plugins/platforms>"
        "$<TARGET_FILE_DIR:MspaceViewer>/platforms"
    COMMENT "Copying Qt platform plugin..."
)