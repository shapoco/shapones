# ShapoNES for the ClockworkPi PicoCalc (Pico 2 / Pico 2 W, RP2350)
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(APP_NAME shapones_picocalc)
project(${APP_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(${APP_NAME})
add_compile_definitions(SHAPONES_ENABLE_LOG=0)

set(SHAPONES_PATH ${CMAKE_CURRENT_LIST_DIR}/../../core)
set(XIAO_PATH ${CMAKE_CURRENT_LIST_DIR}/../xiao)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    ${XIAO_PATH}/common/src/fatfs/*.c
    ${SHAPONES_PATH}/src/*.cpp
)
target_sources(${APP_NAME} PRIVATE ${SRC_FILES}
    # unchanged helpers from the XIAO RP2350 port
    ${XIAO_PATH}/rp/src/lock.cpp
    ${XIAO_PATH}/rp/src/ram.cpp
    ${XIAO_PATH}/rp/src/timer.cpp
    ${XIAO_PATH}/rp/src/gpio.cpp
)
target_compile_options(${APP_NAME} PRIVATE -O2)

target_include_directories(${APP_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${XIAO_PATH}/common/src/fatfs
    ${SHAPONES_PATH}/include
    ${XIAO_PATH}/common/include
)

target_link_libraries(${APP_NAME} PRIVATE
    pico_stdlib pico_multicore
    hardware_spi hardware_i2c hardware_pwm hardware_dma hardware_irq
    hardware_sync hardware_watchdog hardware_clocks
)

pico_set_program_name(${APP_NAME} "ShapoNES for PicoCalc")
pico_enable_stdio_usb(${APP_NAME} 0)
pico_enable_stdio_uart(${APP_NAME} 0)
pico_add_extra_outputs(${APP_NAME})
