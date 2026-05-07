project(TLSRegistration LANGUAGES CXX)

add_library(TLSRegistration SHARED
    src/Function.cpp
)

target_compile_definitions(TLSRegistration PRIVATE TLS_DLL_EXPORT)

target_include_directories(TLSRegistration
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(TLSRegistration
    PUBLIC PCL::common PCL::io PCL::filters PCL::features PCL::registration PCL::search PCL::segmentation PCL::visualization
    PUBLIC Eigen3::Eigen
    PUBLIC Boost::boost
)

set_target_properties(TLSRegistration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
)