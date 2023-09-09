cmake_minimum_required(VERSION 3.14)
project(gcs LANGUAGES CXX)

# find_package(MAVLink REQUIRED)
add_library(GCSMavlink SHARED
    GCSMavlink.cpp
    GCSMavlink.h
    UDPLink.cpp
    UDPLink.h
)

target_include_directories(GCSMavlink PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/src/lib/mavlink/include
)

target_link_libraries(GCSMavlink PRIVATE mavlink)
export(TARGETS GCSMavlink FILE GCSMavlinkTargets.cmake)
