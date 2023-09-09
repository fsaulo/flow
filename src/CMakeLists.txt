cmake_minimum_required(VERSION 3.14)
project(flow LANGUAGES CXX)

# Split the code into separete submodules, each containing
# libraries, tests, modules and other optional modules
set(FLOW_SUBDIRS
    modules
    # utils
    # containers
)

set(FLOW_SRCS)

message("Building 3rd party")
add_subdirectory(lib)

# assemble core libaries
foreach(subdir ${FLOW_SUBDIRS})
    # Build convenience libraries
    file(GLOB_RECURSE subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
	# file(GLOB subdir_test_files "${subdir}/tests/*")
	# list(REMOVE_ITEM subdir_srcs ${subdir_test_files})
    set(${subdir}_SRCS ${subdir_srcs})

    # Build local library and tests
    message(STATUS "Building ${subdir}")
    add_subdirectory(${subdir})
endforeach(subdir)
