set(TARGET_NAME rotary-inverted-pendulum)

set(PLATFORM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(LIBOPENCM3_DIR ${PROJECT_SOURCE_DIR}/third_party/libopencm3)
set(LIBOPENCM3_LIBRARY
    ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f1.a)
set(LINKER_SCRIPT
    ${PLATFORM_DIR}/linker/stm32f103c8t6.ld)

execute_process(
    COMMAND
        git -C ${PROJECT_SOURCE_DIR}
        describe --always --dirty --abbrev=8
    OUTPUT_VARIABLE FW_GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(FW_GIT_DESCRIBE STREQUAL "")
    set(FW_GIT_DESCRIBE "unknown")
endif()

string(
    TIMESTAMP
    FW_BUILD_UTC
    "%Y-%m-%dT%H:%M:%SZ"
    UTC
)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})
configure_file(
    ${PROJECT_SOURCE_DIR}/app/build_info.h.in
    ${GENERATED_DIR}/build_info.h
    @ONLY
)

find_program(HOST_MAKE_EXECUTABLE make REQUIRED)

add_custom_target(libopencm3_stm32f1_build
    COMMAND
        ${HOST_MAKE_EXECUTABLE}
        -C ${LIBOPENCM3_DIR}
        TARGETS=stm32/f1
    BYPRODUCTS
        ${LIBOPENCM3_LIBRARY}
    COMMENT "Building libopencm3 for STM32F1"
)

add_executable(${TARGET_NAME}
    ${PROJECT_SOURCE_DIR}/drivers/ssd1315/ssd1315.c
    ${PLATFORM_DIR}/board/board_adc.c
    ${PLATFORM_DIR}/board/board_button.c
    ${PLATFORM_DIR}/board/board_clock.c
    ${PLATFORM_DIR}/board/board_encoder.c
    ${PLATFORM_DIR}/board/board_led.c
    ${PLATFORM_DIR}/board/board_motor.c
    ${PLATFORM_DIR}/board/board_oled.c
    ${PLATFORM_DIR}/board/board_profile.c
    ${PLATFORM_DIR}/board/board_time.c
    ${PLATFORM_DIR}/board/board_uart.c
)

add_dependencies(
    ${TARGET_NAME}
    libopencm3_stm32f1_build
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${GENERATED_DIR}
    ${PROJECT_SOURCE_DIR}/platform/api
    ${PROJECT_SOURCE_DIR}/drivers/ssd1315
    ${PLATFORM_DIR}/board
    ${LIBOPENCM3_DIR}/include
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    STM32F1
)

target_compile_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -Os
    -g3
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Wshadow
    -Wundef
)

target_link_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -nostartfiles
    -T${LINKER_SCRIPT}
    -L${LIBOPENCM3_DIR}/lib
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map
)

target_link_libraries(${TARGET_NAME} PRIVATE
    rip_app
    ${LIBOPENCM3_LIBRARY}
)

set_target_properties(${TARGET_NAME} PROPERTIES
    SUFFIX ".elf"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_command(
    TARGET ${TARGET_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
        -O ihex
        $<TARGET_FILE:${TARGET_NAME}>
        ${CMAKE_BINARY_DIR}/${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY}
        -O binary
        $<TARGET_FILE:${TARGET_NAME}>
        ${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE}
        --format=berkeley
        $<TARGET_FILE:${TARGET_NAME}>
    BYPRODUCTS
        ${CMAKE_BINARY_DIR}/${TARGET_NAME}.hex
        ${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin
    COMMENT "Generating STM32F103 firmware images"
)
