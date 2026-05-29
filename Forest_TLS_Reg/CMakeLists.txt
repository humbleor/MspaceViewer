cmake_minimum_required(VERSION 3.16)
project(ForestTLSReg)

# Use MSVC on Windows
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# vcpkg toolchain file - use x64-windows triplet (has PCL, OpenCV, Eigen, Boost)
set(VCPKG_TARGET_TRIPLET "x64-windows")
set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

# Find packages
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters segmentation kdtree features registration sample_consensus search surface)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Boost REQUIRED COMPONENTS regex serialization)
find_package(OpenMP REQUIRED)

# Packages from x64-windows-release triplet (manual paths)
set(LIBLAS_RELEASE_ROOT "D:/vcpkg/installed/x64-windows-release")
set(GTSAM_RELEASE_ROOT "D:/vcpkg/installed/x64-windows-release")
set(SMALL_GICP_RELEASE_ROOT "D:/vcpkg/installed/x64-windows-release")

# libLAS (manual)
find_path(libLAS_INCLUDE_DIR NAMES liblas/liblas.hpp PATHS "${LIBLAS_RELEASE_ROOT}/include" NO_DEFAULT_PATH)
find_library(libLAS_LIBRARY NAMES liblas PATHS "${LIBLAS_RELEASE_ROOT}/lib" NO_DEFAULT_PATH)
find_library(libLAS_C_LIBRARY NAMES liblas_c PATHS "${LIBLAS_RELEASE_ROOT}/lib" NO_DEFAULT_PATH)
set(libLAS_LIBRARIES ${libLAS_LIBRARY} ${libLAS_C_LIBRARY})

# GTSAM (manual)
find_path(GTSAM_INCLUDE_DIR NAMES gtsam PATHS "${GTSAM_RELEASE_ROOT}/include" NO_DEFAULT_PATH)
find_library(GTSAM_LIBRARY NAMES gtsam PATHS "${GTSAM_RELEASE_ROOT}/lib" NO_DEFAULT_PATH)

# small_gicp (has precompiled library in x64-windows-release)
set(small_gicp_INCLUDE_DIR "${SMALL_GICP_RELEASE_ROOT}/include")
find_library(small_gicp_LIBRARY NAMES small_gicp PATHS "${SMALL_GICP_RELEASE_ROOT}/lib" NO_DEFAULT_PATH)

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# CSF sources - build from source
set(CSF_SOURCES
    thirdParty/CSF-master/src/CSF.cpp
    thirdParty/CSF-master/src/Cloth.cpp
    thirdParty/CSF-master/src/Constraint.cpp
    thirdParty/CSF-master/src/Particle.cpp
    thirdParty/CSF-master/src/Rasterization.cpp
    thirdParty/CSF-master/src/XYZReader.cpp
    thirdParty/CSF-master/src/point_cloud.cpp
    thirdParty/CSF-master/src/c2cdist.cpp
)

# Include directories
include_directories(
    include
    thirdParty/CSF-master/src
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${libLAS_INCLUDE_DIR}
    ${small_gicp_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

# Add CSF definitions to avoid build errors
add_compile_definitions(
    PROJECT_PATH="${CMAKE_SOURCE_DIR}"
    _USE_MATH_DEFINES
    NOMINMAX
)

# Link directories
link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

# Global source files
file(GLOB utils_sources ./src/utils/*.cpp)
file(GLOB gtsam_opti_sources ./src/gtsam_opti/*.cpp)

# Common link libraries
set(COMMON_LIBS
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${libLAS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${small_gicp_LIBRARY}
)

set(GTSAM_LIBS
    ${COMMON_LIBS}
    ${GTSAM_LIBRARY}
)

##########################################################
#####################Useful Function######################
##########################################################
add_executable(pcdToLas ./src/function/pcdToLas.cpp ./src/utils/Hlp.cpp)
target_compile_options(pcdToLas PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(pcdToLas ${COMMON_LIBS})

add_executable(TransToOriPose ./src/function/TransToOriPos.cpp ./src/utils/Hlp.cpp)
target_link_libraries(TransToOriPose ${COMMON_LIBS})

add_executable(TransToCommon ./src/function/TransToCommon.cpp ./src/utils/Hlp.cpp)
target_link_libraries(TransToCommon ${COMMON_LIBS})

add_executable(TransToCommonByOripose ./src/function/TransToCommonByOripose.cpp ./src/utils/Hlp.cpp)
target_link_libraries(TransToCommonByOripose ${COMMON_LIBS})
##########################################################
#####################Useful Function######################
##########################################################


##########################################################
########################Demo Node#########################
##########################################################
add_library(CSF STATIC ${CSF_SOURCES})
target_include_directories(CSF PRIVATE thirdParty/CSF-master/src ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(CSF ${EIGEN3_LIBRARIES})

# Reg two TLS stations
add_executable(Reg2TLSPoints ./src/demo/Reg2TLS.cpp ${utils_sources})
target_compile_options(Reg2TLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(Reg2TLSPoints
    ${COMMON_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Reg TLS Chain
add_executable(ChainRegTLSPoints ./src/demo/ChainRegTLS.cpp ${utils_sources})
target_compile_options(ChainRegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ChainRegTLSPoints
    ${COMMON_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Multi Reg TLS
add_executable(MultiRegTLSPoints ./src/demo/MultiRegTLS.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegTLSPoints
    ${GTSAM_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Reg Genhe tree dataset
add_executable(MultiRegGenhe ./src/demo/MultiRegTLS_GenHe.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegGenhe PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegGenhe
    ${GTSAM_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Reg Tongji tree dataset
add_executable(MultiRegTongji ./src/demo/MultiRegTongji.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegTongji PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegTongji
    ${GTSAM_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Multi Reg FGI
add_executable(MultiRegFGI ./src/demo/MultiRegFGI.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegFGI PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegFGI
    ${GTSAM_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Reg the ETH
add_executable(MultiRegETH ./src/demo/MultiRegETH.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegETH PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegETH
    ${GTSAM_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)
##########################################################
########################Demo Node#########################
##########################################################


##########################################################
########################Test Node#########################
##########################################################

# Reg between two tower station
add_executable(TowerReg ./src/test/TowerReg.cpp ${utils_sources})
target_compile_options(TowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(TowerReg
    ${COMMON_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Reg Multi tower stations
add_executable(MultiTowerReg ./src/test/MultiTowerReg.cpp ${utils_sources})
target_compile_options(MultiTowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiTowerReg
    ${COMMON_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# Chain Reg tower stations
add_executable(ChainTowerReg ./src/test/ChainTowerReg.cpp ${utils_sources})
target_compile_options(ChainTowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ChainTowerReg
    ${COMMON_LIBS}
    ${OpenMP_CXX_FLAGS}
    CSF
)

# test the dfs algorithm
add_executable(dfsTest ./src/test/dfsTest.cpp)

##########################################################
########################Test Node#########################
##########################################################
