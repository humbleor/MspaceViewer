cmake_minimum_required(VERSION 3.16)
project(MspaceViewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ── 依赖根目录（find_package 找不到时的回退路径）──
set(DEP_ROOT "D:/vcpkg/installed/x64-windows" CACHE PATH "Fallback root directory for dependencies")

# ── Qt5 ──
# 独立安装：-DCMAKE_PREFIX_PATH=D:/Qt/5.15.2/msvc2019_64
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Concurrent Core Gui OpenGL)
set(QT_LIB_PREFIX "Qt5")

# ── OpenSceneGraph ──
# 优先使用 CMake 内置查找（设 OSG_ROOT 环境变量或 -DOSG_DIR=...）
# 找不到时回退到 DEP_ROOT 手动构造
find_package(OpenSceneGraph QUIET COMPONENTS osg osgDB osgUtil osgGA osgViewer osgText osgVolume OpenThreads)

if(OPENSCENEGRAPH_FOUND)
    message(STATUS "Found OpenSceneGraph: ${OPENSCENEGRAPH_INCLUDE_DIRS}")
    set(_OSG_INC "${OPENSCENEGRAPH_INCLUDE_DIRS}")

    # 为每个组件创建 imported target（内置 FindOpenSceneGraph 只设变量，不设 target）
    function(_osg_from_find target libvar)
        if(NOT TARGET OpenSceneGraph::${target} AND ${libvar})
            get_filename_component(_lib_dir "${${libvar}}" DIRECTORY)
            get_filename_component(_lib_name "${${libvar}}" NAME_WE)
            add_library(OpenSceneGraph::${target} UNKNOWN IMPORTED)
            set_target_properties(OpenSceneGraph::${target} PROPERTIES
                IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
                IMPORTED_LOCATION_RELEASE "${${libvar}}"
                IMPORTED_LOCATION_DEBUG "${_lib_dir}/${_lib_name}d.lib"
                INTERFACE_INCLUDE_DIRECTORIES "${_OSG_INC}")
        endif()
    endfunction()

    _osg_from_find(osg       OSG_LIBRARY)
    _osg_from_find(osgDB     OSGDB_LIBRARY)
    _osg_from_find(osgUtil   OSGUTIL_LIBRARY)
    _osg_from_find(osgGA     OSGGA_LIBRARY)
    _osg_from_find(osgViewer OSGVIEWER_LIBRARY)
    _osg_from_find(osgText   OSGTEXT_LIBRARY)
    _osg_from_find(osgVolume OSGVOLUME_LIBRARY)

    if(OPENTHREADS_FOUND AND NOT TARGET OpenThreads::OpenThreads)
        get_filename_component(_ot_dir "${OPENTHREADS_LIBRARY}" DIRECTORY)
        get_filename_component(_ot_name "${OPENTHREADS_LIBRARY}" NAME_WE)
        add_library(OpenThreads::OpenThreads UNKNOWN IMPORTED)
        set_target_properties(OpenThreads::OpenThreads PROPERTIES
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_LOCATION_RELEASE "${OPENTHREADS_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${_ot_dir}/${_ot_name}d.lib"
            INTERFACE_INCLUDE_DIRECTORIES "${_OSG_INC}")
    endif()
else()
    # 回退：从 DEP_ROOT 手动构造（适用于 vcpkg 或无 CMake config 的独立安装）
    # 独立安装：-DDEP_ROOT=D:/OSG
    message(STATUS "OpenSceneGraph not auto-detected, using DEP_ROOT: ${DEP_ROOT}")
    set(_OSG_INC "${DEP_ROOT}/include")

    function(_osg target libname)
        if(NOT TARGET OpenSceneGraph::${target})
            add_library(OpenSceneGraph::${target} UNKNOWN IMPORTED)
            set_target_properties(OpenSceneGraph::${target} PROPERTIES
                IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
                IMPORTED_LOCATION_RELEASE "${DEP_ROOT}/lib/${libname}.lib"
                IMPORTED_LOCATION_DEBUG "${DEP_ROOT}/debug/lib/${libname}d.lib"
                INTERFACE_INCLUDE_DIRECTORIES "${_OSG_INC}")
        endif()
    endfunction()

    _osg(osg       osg)
    _osg(osgDB     osgDB)
    _osg(osgUtil   osgUtil)
    _osg(osgGA     osgGA)
    _osg(osgViewer osgViewer)
    _osg(osgText   osgText)
    _osg(osgVolume osgVolume)

    if(NOT TARGET OpenThreads::OpenThreads)
        add_library(OpenThreads::OpenThreads UNKNOWN IMPORTED)
        set_target_properties(OpenThreads::OpenThreads PROPERTIES
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_LOCATION_RELEASE "${DEP_ROOT}/lib/OpenThreads.lib"
            IMPORTED_LOCATION_DEBUG "${DEP_ROOT}/debug/lib/OpenThreadsd.lib"
            INTERFACE_INCLUDE_DIRECTORIES "${_OSG_INC}")
    endif()
endif()

# ── OSG runtime DLL directories ──
if(OPENSCENEGRAPH_FOUND)
    get_filename_component(OSG_LIB_DIR "${OSGDB_LIBRARY}" DIRECTORY)
    get_filename_component(OSG_ROOT_DIR "${OSG_LIB_DIR}" DIRECTORY)
    set(OSG_BIN_DIR "${OSG_ROOT_DIR}/bin")
    set(OSG_BIN_DIR_DEBUG "${OSG_ROOT_DIR}/bin")
else()
    set(OSG_BIN_DIR "${DEP_ROOT}/bin")
    set(OSG_BIN_DIR_DEBUG "${DEP_ROOT}/debug/bin")
endif()
message(STATUS "OSG DLL dir (Release): ${OSG_BIN_DIR}")
message(STATUS "OSG DLL dir (Debug):   ${OSG_BIN_DIR_DEBUG}")

# ── PCL ──
# 独立安装：-DCMAKE_PREFIX_PATH=D:/PCL 或 -DPCL_DIR=D:/PCL/cmake
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters features registration search segmentation)

# 为 PCL 未提供 target 的组件补建 imported target
function(_pcl target)
    if(NOT TARGET PCL::${target})
        string(TOUPPER "${target}" UPPER)
        if(NOT EXISTS "${DEP_ROOT}/lib/pcl_${target}.lib")
            add_library(PCL::${target} INTERFACE IMPORTED)
            set_target_properties(PCL::${target} PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${PCL_${UPPER}_INCLUDE_DIRS}")
        else()
            add_library(PCL::${target} UNKNOWN IMPORTED)
            set_target_properties(PCL::${target} PROPERTIES
                IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
                IMPORTED_LOCATION_RELEASE "${DEP_ROOT}/lib/pcl_${target}.lib"
                IMPORTED_LOCATION_DEBUG "${DEP_ROOT}/debug/lib/pcl_${target}d.lib"
                INTERFACE_INCLUDE_DIRECTORIES "${PCL_${UPPER}_INCLUDE_DIRS}")
        endif()
    endif()
endfunction()

_pcl(common)
_pcl(io)
_pcl(filters)
_pcl(features)
_pcl(registration)
_pcl(search)
_pcl(segmentation)
_pcl(visualization)

# ── Eigen3 ──
find_package(Eigen3 REQUIRED)

# ── Boost ──
find_package(Boost REQUIRED)

# ── 子模块 ──
add_subdirectory(D4PCR)
add_subdirectory(ULS-TLS)
add_subdirectory(TLSRegistration)
add_subdirectory(MspaceOctree)
add_subdirectory(osgQt)
add_subdirectory(osgdb_bin)
add_subdirectory(MainWindow)
