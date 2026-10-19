# D4PCR 算法实验与全局优化测试程序（独立于 MspaceViewer 主程序构建）
#
# 与库的关系：
#   * Decoupling / IntervalStabbing 由 D4PCR 共享库提供（不在此重复编译）。
#   * 四个实验模块只被本可执行程序使用，因此直接编译进本目标，避免改动库的导出面。
#   * 全局优化流程（手写 4-DOF ICP + GTSAM 位姿图）位于 D4PCR 库中，
#     通过 D4PCR_ENABLE_GLOBAL_OPTIMIZATION 打开后由本程序调用。

project(D4PCRTest LANGUAGES CXX)

if(NOT TARGET D4PCR)
    message(FATAL_ERROR "D4PCR test requires the D4PCR target (add_subdirectory(D4PCR) first).")
endif()

if(NOT PCL_FOUND)
    find_package(PCL 1.14 REQUIRED COMPONENTS common io filters features registration search kdtree visualization)
endif()

set(D4PCR_TEST_SOURCES
    src/main.cpp
    src/GravityExperiment.cpp
    src/NoiseExperiment.cpp
    src/OutlierStageExperiment.cpp
    src/ThreeDMatchExperiment.cpp
)

add_executable(D4PCRTest EXCLUDE_FROM_ALL ${D4PCR_TEST_SOURCES})

target_include_directories(D4PCRTest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(D4PCRTest PRIVATE
    D4PCR
    ${PCL_LIBRARIES}
)

target_compile_definitions(D4PCRTest PRIVATE ${PCL_DEFINITIONS})

if(MSVC)
    # 本目录下新增/改写的源文件为 UTF-8，原始实验文件保持原编码。
    set_source_files_properties(
        src/main.cpp
        PROPERTIES COMPILE_OPTIONS "/utf-8")
endif()

set_target_properties(D4PCRTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
