cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH must point to a Raspberry Pi Pico SDK checkout")
endif()

set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/boards" CACHE STRING "Custom Pico SDK board headers")
set(PICO_BOARD uno_rp2350 CACHE STRING "Pico SDK board")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rip_rp2350a C CXX ASM)
pico_sdk_init()

option(RIP_ENABLE_CDC_LOG "Enable unsolicited CDC debug logging" ON)

add_executable(rip_rp2350a
    src/main.cpp
    src/board.cpp
    src/runtime.cpp
    src/platform.cpp
    src/encoder_diag.cpp
    src/commissioning.cpp
    src/usb.cpp
    src/usb_descriptors.c
)

pico_generate_pio_header(rip_rp2350a
    ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio
)

target_include_directories(rip_rp2350a PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rip_rp2350a PRIVATE
    pico_stdlib
    pico_bootrom
    hardware_adc
    hardware_clocks
    hardware_gpio
    hardware_pio
    hardware_pwm
    hardware_watchdog
    tinyusb_device
)

target_compile_options(rip_rp2350a PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-fno-exceptions;-fno-rtti>
)

if(RIP_ENABLE_CDC_LOG)
    target_compile_definitions(rip_rp2350a PRIVATE RIP_ENABLE_CDC_LOG=1)
else()
    target_compile_definitions(rip_rp2350a PRIVATE RIP_ENABLE_CDC_LOG=0)
endif()

pico_enable_stdio_usb(rip_rp2350a 0)
pico_enable_stdio_uart(rip_rp2350a 0)

pico_add_extra_outputs(rip_rp2350a)
