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
