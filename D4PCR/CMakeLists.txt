project(D4PCR LANGUAGES CXX)

# ── PCL ── (found by root CMakeLists.txt via CMAKE_PREFIX_PATH)
find_package(PCL 1.14 REQUIRED COMPONENTS common io filters features registration search segmentation visualization)

# ── Eigen3 ── (found by root CMakeLists.txt from PCL's 3rdParty)
if(NOT TARGET Eigen3::Eigen)
    message(FATAL_ERROR "Eigen3::Eigen target not found. Ensure root CMakeLists.txt sets it up.")
endif()

add_library(D4PCR SHARED
    src/Decoupling.cpp
    src/Generator.cpp
    src/IntervalStabbing.cpp
    src/RelationshipConstruction.cpp
)

target_compile_definitions(D4PCR PRIVATE D4PCR_DLL_EXPORT)

target_include_directories(D4PCR
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC ${PCL_INCLUDE_DIRS}
)

target_link_libraries(D4PCR
    PUBLIC ${PCL_LIBRARIES}
    PUBLIC Eigen3::Eigen
    PUBLIC ${libLAS_LIBRARY}
)

set_target_properties(D4PCR PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
)