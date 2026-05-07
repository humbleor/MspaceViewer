project(D4PCR LANGUAGES CXX)

add_library(D4PCR SHARED
    src/Decoupling.cpp
    src/Generator.cpp
    src/IntervalStabbing.cpp
    src/RelationshipConstruction.cpp
)

target_compile_definitions(D4PCR PRIVATE D4PCR_DLL_EXPORT)

target_include_directories(D4PCR
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(D4PCR
    PUBLIC PCL::common PCL::io PCL::filters PCL::features PCL::registration PCL::search PCL::segmentation PCL::visualization
    PUBLIC Eigen3::Eigen
)

# Windows DLL 输出目录
set_target_properties(D4PCR PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
)