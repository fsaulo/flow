cmake_minimum_required(VERSION 3.14)

set(FLOW_VERSION_MAJOR 0)
set(FLOW_VERSION_MINOR 1)
set(FLOW_VERSION_PATCH 0)

set(CMAKE_COLOR_MAKEFILE ON)
add_compile_options(-fdiagnostics-color=always)

math (EXPR FLOW_VERSION_NUMERIC "10000 * ${FLOW_VERSION_MAJOR} + 100 * ${FLOW_VERSION_MINOR} + ${FLOW_VERSION_PATCH}")
project(flow LANGUAGES CXX C VERSION "${FLOW_VERSION_MAJOR}.${FLOW_VERSION_MINOR}.${FLOW_VERSION_PATCH}")

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

add_subdirectory(tests)
add_subdirectory(src)
add_subdirectory(examples)
