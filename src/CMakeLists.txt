cmake_minimum_required(VERSION 3.14)
project(flow LANGUAGES CXX)

set(FLOW_SRCS)

# Split the code into separete submodules, each containing
# libraries, tests, modules and other optional modules
include_directories(
    modules
    utils
)

message("Building 3rd party")
add_subdirectory(lib)
add_subdirectory(modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(estimator
    main.cpp
    Estimator.cpp
    Estimator.h
    params.h
    utils/DCOffsetFilter.cpp
    utils/DCOffsetFilter.h
)

target_compile_options(estimator PRIVATE
    -Wno-address-of-packed-member
)

target_include_directories(estimator PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/src/lib/mavlink/include
)

target_link_libraries(estimator PRIVATE 
    CvInterface
    GCSMavlink

    opencv_core 
    opencv_highgui 
    opencv_imgproc 
    opencv_video
    opencv_videoio

    ${EIGEN3_LIBRARIES}
)
