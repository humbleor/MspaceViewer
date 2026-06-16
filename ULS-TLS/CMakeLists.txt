project(ULS-TLS LANGUAGES CXX)

# ── PCL ──
find_package(PCL 1.14 REQUIRED COMPONENTS common io)

add_library(ULS-TLS SHARED
    src/LoadLasFile.cpp
    src/RegistrationU2T.cpp
)

target_compile_definitions(ULS-TLS PRIVATE ULS_TLS_DLL_EXPORT)

target_include_directories(ULS-TLS
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    # laszip_api.h 通过 <laszip/laszip_api.h> 引用，需将 libs 目录加入路径
    PUBLIC ${CMAKE_SOURCE_DIR}/MspaceOctree/libs
    PUBLIC ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ULS-TLS
    PUBLIC laszip
    PUBLIC ${PCL_LIBRARIES}
    PUBLIC ${libLAS_LIBRARY}
)

set_target_properties(ULS-TLS PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
)