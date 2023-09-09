cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(CvInterface SHARED
    CvInterface.cpp
    CvInterface.h
)

target_include_directories(CvInterface PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
)

target_link_libraries(CvInterface PRIVATE 
    opencv_core 
    opencv_highgui 
    opencv_imgproc 
    opencv_video
    opencv_videoio
    ${EIGEN3_LIBRARIES}
)
