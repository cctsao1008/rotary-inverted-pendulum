add_library(rip_app INTERFACE)

target_sources(rip_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/closed_loop_enable_gate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/closed_loop_gate_runtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/closed_loop_mode_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/command_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/control_profile.c
    ${CMAKE_CURRENT_SOURCE_DIR}/control_sensor_adapter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/key_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/local_ui.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motion_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_authority.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_test_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pendulum_angle.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime_parameters.c
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry_toggle.c
)

target_include_directories(rip_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/platform/api
    ${PROJECT_SOURCE_DIR}/drivers/ssd1315
)

target_link_libraries(rip_app INTERFACE
    control_core
)
