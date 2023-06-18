cmake_minimum_required(VERSION 3.5)
project(scripts)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_COLOR_DIAGNOSTICS ON)

file(GLOB_RECURSE CPP_SOURCES "cpp/*.cpp")
file(GLOB_RECURSE PYTHON_SOURCES "*.py")

find_package(OpenCV REQUIRED)

foreach(CPP_SOURCE ${CPP_SOURCES})
    get_filename_component(CPP_TARGET_NAME ${CPP_SOURCE} NAME_WE)
    add_executable(${CPP_TARGET_NAME} ${CPP_SOURCE})
    target_include_directories(${CPP_TARGET_NAME} PRIVATE include)
    target_link_libraries(${CPP_TARGET_NAME} PRIVATE 
        opencv_core 
        opencv_highgui 
        opencv_imgproc 
        opencv_video
        opencv_videoio
    )
endforeach()

foreach(PYTHON_SOURCE ${PYTHON_SOURCES})
    get_filename_component(PYTHON_TARGET_NAME ${PYTHON_SOURCE} NAME_WE)
    add_custom_target(${PYTHON_TARGET_NAME} COMMAND python ${PYTHON_SOURCE})
endforeach()
