cmake_minimum_required(VERSION 3.16)

project(rotary_inverted_pendulum LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(FW_VERSION "0.1.0-dev"
    CACHE STRING
    "Firmware semantic version")

set(RIP_TARGET "stm32f103"
    CACHE STRING
    "Firmware target; supported values: stm32f103, none")
set_property(CACHE RIP_TARGET PROPERTY STRINGS stm32f103 none)

option(BUILD_HOST_TESTS
       "Build host-side unit tests"
       OFF)

add_subdirectory(control)
add_subdirectory(app)

if(RIP_TARGET STREQUAL "stm32f103")
    add_subdirectory(platform/stm32f103)
elseif(RIP_TARGET STREQUAL "none")
    message(STATUS "Firmware target disabled (RIP_TARGET=none)")
else()
    message(FATAL_ERROR
        "Unsupported RIP_TARGET='${RIP_TARGET}'. Supported values: stm32f103, none")
endif()

if(BUILD_HOST_TESTS)
    enable_testing()
    add_subdirectory(tests)
    add_subdirectory(tests/closed_loop_mode_manager)
endif()
