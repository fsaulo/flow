find_program(Python_EXECUTABLE python3 python)
message(STATUS "Using Python interpreter: ${Python_EXECUTABLE}")

# Build Mavlink library and set compile definitions
add_subdirectory(mavlink)

target_compile_options(mavlink INTERFACE
    -Wno-address-of-packed-member
)

target_compile_definitions(mavlink INTERFACE
    MAVLINK_DIALECT=common
    MAVLINK_VERSION=2
)
