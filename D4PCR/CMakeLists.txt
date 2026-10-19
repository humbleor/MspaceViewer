project(D4PCR LANGUAGES CXX)

# 全局优化流程（手写 4-DOF ICP + GTSAM 位姿图）需要一个可用的 GTSAM。
# 默认开启；在没有 GTSAM 的机器上可用 -DD4PCR_ENABLE_GLOBAL_OPTIMIZATION=OFF 跳过。
option(D4PCR_ENABLE_GLOBAL_OPTIMIZATION
    "Build the D4PCR global-optimization pipeline (requires GTSAM)" ON)

# ── PCL ── (found by root CMakeLists.txt via CMAKE_PREFIX_PATH)
find_package(PCL 1.14 REQUIRED COMPONENTS common io filters features registration search segmentation visualization)

# ── Eigen3 ── (found by root CMakeLists.txt from PCL's 3rdParty)
if(NOT TARGET Eigen3::Eigen)
    message(FATAL_ERROR "Eigen3::Eigen target not found. Ensure root CMakeLists.txt sets it up.")
endif()

# ═══════════════════════════════════════════════════
# GTSAM — required by the global-optimization pipeline
# ═══════════════════════════════════════════════════
# 仅在启用全局优化时才查找 GTSAM：
#   * 优先使用 vcpkg 的 CONFIG 包；
#   * 否则回退到与传统 Forest_TLS_Reg 相同的“手动路径”查找方式。
if(D4PCR_ENABLE_GLOBAL_OPTIMIZATION)
    find_package(GTSAM CONFIG QUIET)

    if(NOT TARGET gtsam)
        if(DEFINED GTSAM_RELEASE_ROOT)
            set(_d4pcr_gtsam_root "${GTSAM_RELEASE_ROOT}")
        else()
            set(_d4pcr_gtsam_root "F:/vcpkg/installed/x64-windows-release")
        endif()

        find_path(GTSAM_INCLUDE_DIR
            NAMES gtsam/geometry/Pose3.h
            PATHS "${_d4pcr_gtsam_root}/include"
            NO_DEFAULT_PATH
        )
        find_library(GTSAM_LIBRARY
            NAMES gtsam
            PATHS "${_d4pcr_gtsam_root}/lib"
            NO_DEFAULT_PATH
        )

        if(GTSAM_INCLUDE_DIR AND GTSAM_LIBRARY)
            add_library(gtsam SHARED IMPORTED)
            set_target_properties(gtsam PROPERTIES
                IMPORTED_LOCATION "${_d4pcr_gtsam_root}/bin/gtsam.dll"
                IMPORTED_IMPLIB "${GTSAM_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIR}"
            )
            message(STATUS "gtsam (manual) -> ${GTSAM_LIBRARY}")
        else()
            message(FATAL_ERROR
                "GTSAM not found. The D4PCR global-optimization pipeline requires it.\n"
                "  * Option 1 (vcpkg):  F:/vcpkg/vcpkg.exe install gtsam:x64-windows-release\n"
                "                       then pass -DCMAKE_PREFIX_PATH=<vcpkg installed dir> or\n"
                "                       -DGTSAM_DIR=<dir containing GTSAMConfig.cmake>\n"
                "  * Option 2 (manual): -DGTSAM_RELEASE_ROOT=<dir with include/ and lib/>\n"
                "  * Option 3 (skip):   build with -DD4PCR_ENABLE_GLOBAL_OPTIMIZATION=OFF")
        endif()
    endif()
endif()

set(D4PCR_GLOBAL_OPTIMIZATION_SOURCES)
if(D4PCR_ENABLE_GLOBAL_OPTIMIZATION)
    set(D4PCR_GLOBAL_OPTIMIZATION_SOURCES src/GlobalOptimizationPipeline.cpp)
endif()

add_library(D4PCR SHARED
    src/Decoupling.cpp
    src/Generator.cpp
    src/IntervalStabbing.cpp
    src/RelationshipConstruction.cpp
    ${D4PCR_GLOBAL_OPTIMIZATION_SOURCES}
)

target_compile_definitions(D4PCR PRIVATE D4PCR_DLL_EXPORT)

target_include_directories(D4PCR
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC ${PCL_INCLUDE_DIRS}
)

target_link_libraries(D4PCR
    PUBLIC ${PCL_LIBRARIES}
    PUBLIC PCL::common PCL::io PCL::filters PCL::features PCL::registration
           PCL::search PCL::kdtree PCL::segmentation PCL::visualization
    PUBLIC Eigen3::Eigen
    PUBLIC ${libLAS_LIBRARY}
)

if(D4PCR_ENABLE_GLOBAL_OPTIMIZATION)
    target_link_libraries(D4PCR PUBLIC gtsam)
    target_compile_definitions(D4PCR PUBLIC D4PCR_HAS_GLOBAL_OPTIMIZATION=1)
    add_executable(D4PCRGlobalOptimization src/GlobalOptimizationMain.cpp)
    target_link_libraries(D4PCRGlobalOptimization PRIVATE D4PCR)
    # 工人进程放到 runtime/ 子目录，避免与主程序并列混淆；DLL 仍留在根目录。
    set_target_properties(D4PCRGlobalOptimization PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>/runtime")
    install(TARGETS D4PCRGlobalOptimization RUNTIME DESTINATION runtime)
endif()

if(MSVC)
    # 重写过的源文件为 UTF-8，其余原始文件保持原编码。
    set_source_files_properties(
        src/Decoupling.cpp
        src/GlobalOptimizationPipeline.cpp
        src/IntervalStabbing.cpp
        PROPERTIES COMPILE_OPTIONS "/utf-8;/bigobj")
endif()

set_target_properties(D4PCR PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib"
)

# ── 算法实验 / 全局优化测试程序（独立目标，不参与主程序运行）──
option(BUILD_D4PCR_TESTS "Build the D4PCR experiment/test executables" OFF)
if(BUILD_D4PCR_TESTS)
    add_subdirectory(test)
endif()
