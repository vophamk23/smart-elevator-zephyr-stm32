# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smart_elevator_bkit_arm4 LANGUAGES C)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources(app PRIVATE
    src/main.c

    # Board Support Package (Per-peripheral BKIT ARM4 drivers)
    src/bsp/bsp_gpio_exti.c
    src/bsp/bsp_pwm_buzzer.c
    src/bsp/bsp_adc_sensors.c
    src/bsp/bsp_spi_hmi.c
    src/bsp/bsp_i2c_storage.c
    src/bsp/bsp_watchdog.c

    # Core Safety Control & Scheduling Logic
    src/core/elevator_scheduler.c
    src/core/elevator_hsm.c

    # Distributed Communications & HIL Digital Twin Stream
    src/comms/can_bus.c
    src/comms/twin_telemetry.c

    # Interactive Zephyr Shell Commands
    src/cli/cli_elev.c
    src/cli/cli_bkit_test.c
)
