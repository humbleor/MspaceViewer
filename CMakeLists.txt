cmake_minimum_required(VERSION 3.21)
project(MspaceViewer LANGUAGES CXX)

# ═══════════════════════════════════════════════════
# C++ Standard
# ═══════════════════════════════════════════════════
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(_USE_MATH_DEFINES)

# ═══════════════════════════════════════════════════
# CMAKE PREFIX PATH — all precompiled libraries
# ═══════════════════════════════════════════════════
list(APPEND CMAKE_PREFIX_PATH
    "D:/Qt/6.11.1/msvc2022_64"              # Qt 6.11.1 MSVC 2022
    "D:/Program Files/PCL 1.14.1"            # PCL + Eigen + Boost + FLANN
    "D:/Software/opencv/build"               # OpenCV 4.7
)

# ═══════════════════════════════════════════════════
# Qt 6
# ═══════════════════════════════════════════════════
find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent Core Gui OpenGL OpenGLWidgets)
set(QT_LIB_PREFIX "Qt6")  # backward compatibility for sub-projects

# ═══════════════════════════════════════════════════
# PCL
# ═══════════════════════════════════════════════════
find_package(PCL 1.14 REQUIRED COMPONENTS common io filters features registration search kdtree)

# Create IMPORTED targets for PCL so TARGET_RUNTIME_DLLS can resolve DLLs
# PCLConfig.cmake sets PCL_INCLUDE_DIRS and PCL_LIBRARIES but not proper imported targets
foreach(_pcl_comp common io filters features registration search kdtree octree sample_consensus segmentation visualization ml)
    if(NOT TARGET PCL::${_pcl_comp})
        string(TOUPPER ${_pcl_comp} _pcl_comp_upper)
        find_library(_pcl_${_pcl_comp}_lib
            NAMES pcl_${_pcl_comp}
            PATHS "${PCL_ROOT}/lib"
            NO_DEFAULT_PATH
        )
        find_library(_pcl_${_pcl_comp}_debug_lib
            NAMES pcl_${_pcl_comp}d
            PATHS "${PCL_ROOT}/lib"
            NO_DEFAULT_PATH
        )
        if(_pcl_${_pcl_comp}_lib)
            # IMPORTED_LOCATION must point to the DLL, IMPORTED_IMPLIB to the .lib
            set(_pcl_${_pcl_comp}_dll "${PCL_ROOT}/bin/pcl_${_pcl_comp}.dll")
            set(_pcl_${_pcl_comp}_debug_dll "${PCL_ROOT}/bin/pcl_${_pcl_comp}d.dll")
            add_library(PCL::${_pcl_comp} SHARED IMPORTED)
            set_target_properties(PCL::${_pcl_comp} PROPERTIES
                IMPORTED_LOCATION "${_pcl_${_pcl_comp}_dll}"
                IMPORTED_IMPLIB "${_pcl_${_pcl_comp}_lib}"
                INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
            )
            if(EXISTS "${_pcl_${_pcl_comp}_debug_dll}")
                set_target_properties(PCL::${_pcl_comp} PROPERTIES
                    IMPORTED_LOCATION_DEBUG "${_pcl_${_pcl_comp}_debug_dll}"
                    IMPORTED_IMPLIB_DEBUG "${_pcl_${_pcl_comp}_debug_lib}"
                )
            endif()
            message(STATUS "PCL::${_pcl_comp} -> ${_pcl_${_pcl_comp}_dll}")
        endif()
    endif()
endforeach()

# ═══════════════════════════════════════════════════
# Eigen3 (use PCL's bundled copy)
# PCLConfig.cmake already sets Eigen3_DIR & calls find_package(Eigen3)
# ═══════════════════════════════════════════════════
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()

# ═══════════════════════════════════════════════════
# Boost (use PCL's bundled copy, includes compiled libs)
# PCL's find_boost() calls find_package(Boost system iostreams filesystem serialization)
# but does NOT include regex — we add it manually from PCL's Boost lib directory
# ═══════════════════════════════════════════════════
if(NOT TARGET Boost::regex)
    find_library(BOOST_REGEX_LIBRARY
        NAMES boost_regex
        PATHS "${PCL_ROOT}/3rdParty/Boost/lib"
        NO_DEFAULT_PATH
        REQUIRED
    )
    add_library(Boost::regex UNKNOWN IMPORTED)
    set_target_properties(Boost::regex PROPERTIES
        IMPORTED_LOCATION "${BOOST_REGEX_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIR}")
    message(STATUS "Boost::regex (from PCL): ${BOOST_REGEX_LIBRARY}")
endif()

# ═══════════════════════════════════════════════════
# OpenCV 4.7 (only vc16 binaries available)
# ═══════════════════════════════════════════════════
set(OpenCV_ARCH "x64")
set(OpenCV_RUNTIME "vc16")
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ═══════════════════════════════════════════════════
# OpenMP
# ═══════════════════════════════════════════════════
find_package(OpenMP REQUIRED)

# ═══════════════════════════════════════════════════
# OSG (from vcpkg manifest, uses unofficial targets)
# ═══════════════════════════════════════════════════
find_package(unofficial-osg CONFIG REQUIRED)
set(OSG_LIBS
    unofficial::osg::osg
    unofficial::osg::osgDB
    unofficial::osg::osgUtil
    unofficial::osg::osgGA
    unofficial::osg::osgViewer
    unofficial::osg::osgText
    unofficial::osg::osgVolume
    unofficial::osg::OpenThreads
)
# unofficial-osg targets don't carry INTERFACE_INCLUDE_DIRECTORIES
find_path(OSG_INCLUDE_DIR NAMES osg/PagedLOD)
if(OSG_INCLUDE_DIR)
    foreach(_osg_target ${OSG_LIBS})
        target_include_directories(${_osg_target} INTERFACE ${OSG_INCLUDE_DIR})
    endforeach()
endif()

# ═══════════════════════════════════════════════════
# libLAS (from vcpkg manifest)
# ═══════════════════════════════════════════════════
find_path(libLAS_INCLUDE_DIR NAMES liblas/liblas.hpp)
find_library(libLAS_LIBRARY_PATH NAMES liblas)
find_library(libLAS_C_LIBRARY_PATH NAMES liblas_c)
if(NOT libLAS_INCLUDE_DIR OR NOT libLAS_LIBRARY_PATH)
    message(FATAL_ERROR "libLAS not found. Ensure vcpkg manifest mode is active:\n"
                        "  cmake .. -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake\n"
                        "           -DVCPKG_TARGET_TRIPLET=x64-windows-release")
endif()

# Create IMPORTED targets so $<TARGET_RUNTIME_DLLS:...> can resolve their DLLs
add_library(libLAS SHARED IMPORTED)
set_target_properties(libLAS PROPERTIES
    IMPORTED_LOCATION "${libLAS_LIBRARY_PATH}"
    IMPORTED_IMPLIB "${libLAS_LIBRARY_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${libLAS_INCLUDE_DIR}")
# Backward-compat variable for MainWindow
set(libLAS_LIBRARY libLAS)

add_library(libLAS_C SHARED IMPORTED)
set_target_properties(libLAS_C PROPERTIES
    IMPORTED_LOCATION "${libLAS_C_LIBRARY_PATH}"
    IMPORTED_IMPLIB "${libLAS_C_LIBRARY_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${libLAS_INCLUDE_DIR}")
set(libLAS_C_LIBRARY libLAS_C)

# ═══════════════════════════════════════════════════
# small_gicp (embedded, compiled as static library)
# Only kept: include/ headers + src/small_gicp/registration/*.cpp
# Removed: benchmark/, ros/, examples/, tests/, python/
# ═══════════════════════════════════════════════════
if(NOT TARGET small_gicp)
    set(SMALL_GICP_DIR "${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/small_gicp")
    if(EXISTS "${SMALL_GICP_DIR}/include/small_gicp")
        add_library(small_gicp STATIC
            ${SMALL_GICP_DIR}/src/small_gicp/registration/registration.cpp
            ${SMALL_GICP_DIR}/src/small_gicp/registration/registration_helper.cpp
        )
        target_include_directories(small_gicp
            PUBLIC ${SMALL_GICP_DIR}/include
        )
        target_link_libraries(small_gicp
            PUBLIC Eigen3::Eigen
            PUBLIC OpenMP::OpenMP_CXX
        )
        message(STATUS "small_gicp: embedded at ${SMALL_GICP_DIR}")
    else()
        message(FATAL_ERROR "small_gicp not found at ${SMALL_GICP_DIR}")
    endif()
endif()

# ═══════════════════════════════════════════════════
# Sub-projects
# ═══════════════════════════════════════════════════
add_subdirectory(MspaceOctree)
add_subdirectory(osgQt)
add_subdirectory(osgdb_bin)
add_subdirectory(TLSRegistration)
add_subdirectory(ULS-TLS)
add_subdirectory(D4PCR)
add_subdirectory(MainWindow)

# ── Optional CLI tools ──
option(BUILD_CLI_TOOLS "Build command-line registration tools" OFF)

if(BUILD_CLI_TOOLS)
    # ── D4PCR CLI tool (outlier removal registration) ──
    add_executable(OutlierRemovalRegCLI ${CMAKE_SOURCE_DIR}/cli/OutlierRemovalRegCLI.cpp)
    target_link_libraries(OutlierRemovalRegCLI D4PCR ${PCL_LIBRARIES} Eigen3::Eigen)
    set_target_properties(OutlierRemovalRegCLI PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )

    # ── TLSRegistration CLI tool (TLS↔TLS diamond + FPFH + ICP) ──
    add_executable(TLSRegistrationCLI ${CMAKE_SOURCE_DIR}/cli/TLSRegistrationCLI.cpp)
    target_include_directories(TLSRegistrationCLI PRIVATE
        ${CMAKE_SOURCE_DIR}/MainWindow/include
        ${libLAS_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
    )
    target_link_libraries(TLSRegistrationCLI
        TLSRegistration
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${libLAS_LIBRARY}
        Boost::boost
    )
    set_target_properties(TLSRegistrationCLI PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )

    # ── ForestReg CLI tool (HashRegDescManager coarse + small_gicp fine) ──
    # Mirrors MainWindow/src/ForestRegistration.cpp minus Qt. Compiles the same
    # FOREST_REG_SOURCES that MainWindow bundles so the algorithm stays in sync.
    add_executable(ForestRegCLI ${CMAKE_SOURCE_DIR}/cli/ForestRegCLI.cpp
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
    target_include_directories(ForestRegCLI PRIVATE
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/include
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/include/utils
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/include/dst
        ${CMAKE_SOURCE_DIR}/Forest_TLS_Reg/thirdParty/CSF-master/src
        ${libLAS_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PCL_ROOT}/3rdParty/FLANN/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(ForestRegCLI
        PCL::common PCL::io PCL::filters PCL::features PCL::registration PCL::search PCL::kdtree PCL::segmentation
        Eigen3::Eigen
        opencv_world
        ${libLAS_LIBRARY}
        ${libLAS_C_LIBRARY}
        Boost::regex Boost::serialization
        small_gicp
        OpenMP::OpenMP_CXX
    )
    set_target_properties(ForestRegCLI PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endif()


# ═══════════════════════════════════════════════════
# CMake Install + CPack
# ═══════════════════════════════════════════════════
include(GNUInstallDirs)

set(INSTALL_PREFIX "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_INSTALL_PREFIX "${INSTALL_PREFIX}" CACHE PATH "Install prefix" FORCE)

# ── Project DLLs (internal targets) ──
install(TARGETS MspaceViewer MspaceOctree laszip osgQt osgdb_bin D4PCR ULS-TLS TLSRegistration
    RUNTIME DESTINATION .)

# ── Resource directories ──
install(DIRECTORY "${CMAKE_SOURCE_DIR}/MainWindow/resource"
    DESTINATION .
    FILES_MATCHING PATTERN "*.qm" PATTERN "*.ts" PATTERN "*.ui" PATTERN "*.qrc"
    PATTERN "*.ico" PATTERN "*.png" PATTERN "*.jpg")

# OSG plugins directory
install(DIRECTORY "${CMAKE_BINARY_DIR}/bin/Release/osgPlugins-3.6.5"
    DESTINATION .
    USE_SOURCE_PERMISSIONS)

# ── Qt6 DLLs (only modules actually linked) ──
get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION)
cmake_path(GET _qt_core_loc PARENT_PATH _qt_bin_dir)
set(_QT6_NEEDED Core Gui Widgets Concurrent OpenGL OpenGLWidgets)
set(_QT6_DLLS)
foreach(_mod ${_QT6_NEEDED})
    file(GLOB _mod_dll "${_qt_bin_dir}/Qt6${_mod}*.dll")
    list(FILTER _mod_dll EXCLUDE REGEX "d\\.dll$")
    list(APPEND _QT6_DLLS ${_mod_dll})
endforeach()
install(FILES ${_QT6_DLLS} DESTINATION .)

# ── Qt platform plugins ──
install(DIRECTORY "${_qt_bin_dir}/../plugins/platforms"
    DESTINATION .
    FILES_MATCHING PATTERN "*.dll"
    PATTERN "*d.dll" EXCLUDE)

# ── Third-party DLLs ──
# vcpkg installed directory (OSG, libLAS, Boost, freetype, etc)
set(VCPKG_BIN_DIR "${CMAKE_SOURCE_DIR}/build/vcpkg_installed/x64-windows-release/bin")
file(GLOB _VCPKG_DLLS "${VCPKG_BIN_DIR}/*.dll")
list(FILTER _VCPKG_DLLS EXCLUDE REGEX "_d\\.dll$")
install(FILES ${_VCPKG_DLLS} DESTINATION .)

# PCL DLLs (non-debug only)
set(_PCL_INSTALL_DLLS
    "${PCL_ROOT}/bin/pcl_common.dll"
    "${PCL_ROOT}/bin/pcl_features.dll"
    "${PCL_ROOT}/bin/pcl_filters.dll"
    "${PCL_ROOT}/bin/pcl_io.dll"
    "${PCL_ROOT}/bin/pcl_io_ply.dll"
    "${PCL_ROOT}/bin/pcl_kdtree.dll"
    "${PCL_ROOT}/bin/pcl_ml.dll"
    "${PCL_ROOT}/bin/pcl_octree.dll"
    "${PCL_ROOT}/bin/pcl_registration.dll"
    "${PCL_ROOT}/bin/pcl_sample_consensus.dll"
    "${PCL_ROOT}/bin/pcl_search.dll"
    "${PCL_ROOT}/bin/pcl_segmentation.dll"
)
install(FILES ${_PCL_INSTALL_DLLS} DESTINATION .)

# VTK DLLs
file(GLOB _VTK_DLLS "${PCL_ROOT}/3rdParty/VTK/bin/vtk*.dll")
list(FILTER _VTK_DLLS EXCLUDE REGEX "d\\.dll$")
install(FILES ${_VTK_DLLS} DESTINATION .)

# FLANN DLL
install(FILES "${PCL_ROOT}/3rdParty/FLANN/bin/flann_cpp.dll" DESTINATION .)

# OpenCV DLL
install(FILES "D:/Software/opencv/build/x64/vc16/bin/opencv_world470.dll" DESTINATION .)

# OpenNI2 DLL
install(FILES "C:/Program Files/OpenNI2/Redist/OpenNI2.dll" DESTINATION .)

# ═══════════════════════════════════════════════════
# CPack
# ═══════════════════════════════════════════════════
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "MspaceViewer")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "LiDAR point cloud visualization and registration application")
set(CPACK_SOURCE_IGNORE_FILES
    "/\\.git/"
    "/build/"
    "/MspaceViewer_Release/")
include(CPack)
