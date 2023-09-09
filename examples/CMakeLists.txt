cmake_minimum_required(VERSION 3.5)
project(scripts)

option(USE_MAVLINK_PROTOCOL "Compiling with MAVLink support" OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_COLOR_DIAGNOSTICS ON)

file(GLOB_RECURSE CPP_SOURCES "cpp/*.cpp")
file(GLOB_RECURSE PYTHON_SOURCES "*.py")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/utils
)

if (USE_MAVLINK_PROTOCOL)
    find_package(MAVLink REQUIRED)
    add_compile_definitions(MAVLINK_UDP_ENABLED)
    include_directories(${MAVLINK_INCLUDE_DIRS})
endif()

foreach(CPP_SOURCE ${CPP_SOURCES})
    get_filename_component(CPP_TARGET_NAME ${CPP_SOURCE} NAME_WE)
    add_executable(${CPP_TARGET_NAME} ${CPP_SOURCE})
    target_include_directories(${CPP_TARGET_NAME} PRIVATE include include/utils)
    target_link_libraries(${CPP_TARGET_NAME} PRIVATE 
        opencv_core 
        opencv_highgui 
        opencv_imgproc 
        opencv_video
        opencv_videoio
        ${EIGEN3_LIBRARIES}
    )
    if (USE_MAVLINK_PROTOCOL)
        target_link_libraries(${CPP_TARGET_NAME} PRIVATE MAVLink::mavlink)
    endif()
endforeach()

foreach(PYTHON_SOURCE ${PYTHON_SOURCES})
    get_filename_component(PYTHON_TARGET_NAME ${PYTHON_SOURCE} NAME_WE)
    add_custom_target(${PYTHON_TARGET_NAME} COMMAND python ${PYTHON_SOURCE})
endforeach()
