project(TLSRegistration LANGUAGES CXX)

# ── PCL ── (found by root CMakeLists.txt via CMAKE_PREFIX_PATH)
find_package(PCL 1.14 REQUIRED COMPONENTS common io filters features registration search segmentation visualization)

# ── Eigen3 ── (found by root CMakeLists.txt from PCL's 3rdParty)
if(NOT TARGET Eigen3::Eigen)
    message(FATAL_ERROR "Eigen3::Eigen target not found. Ensure root CMakeLists.txt sets it up.")
endif()

# ── Boost ── (found by root CMakeLists.txt from PCL's 3rdParty)
if(NOT TARGET Boost::boost)
    message(FATAL_ERROR "Boost::boost target not found. Ensure root CMakeLists.txt sets it up.")
endif()

add_library(TLSRegistration SHARED
    src/Function.cpp
)

target_compile_definitions(TLSRegistration PRIVATE TLS_DLL_EXPORT)

target_include_directories(TLSRegistration
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC ${PCL_INCLUDE_DIRS}
)

target_link_libraries(TLSRegistration
    PUBLIC ${PCL_LIBRARIES}
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