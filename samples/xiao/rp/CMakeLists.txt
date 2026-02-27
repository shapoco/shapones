cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
set(PICO_EXTRAS_PATH $ENV{PICO_EXTRAS_PATH})
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

set(APP_NAME shapones_xiao_rp)

project(${APP_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(${APP_NAME})
add_compile_options(-O2)
add_compile_definitions(SHAPONES_ENABLE_LOG=0)

set(SHAPONES_PATH ../../../core)
set(COMMON_PATH ../common)

file(GLOB CPP_FILES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    ${SHAPONES_PATH}/src/*.cpp
    ${COMMON_PATH}/src/*.cpp
    ${COMMON_PATH}/src/*.c
    ${COMMON_PATH}/src/fatfs/*.c
)

target_sources(
    ${APP_NAME} PRIVATE 
    ${CPP_FILES}
)

target_include_directories(
    ${APP_NAME} PRIVATE
    ./include
    ${SHAPONES_PATH}/include
    ${COMMON_PATH}/include
    ${COMMON_PATH}/src/fatfs
)

target_link_libraries(
    ${APP_NAME} PRIVATE
    hardware_pio
    hardware_i2c
    hardware_spi
    hardware_pwm
    hardware_dma
    hardware_irq
    hardware_flash
    hardware_sync
    hardware_sleep
    hardware_watchdog
    hardware_powman
    pico_stdlib
    pico_multicore
)

pico_add_extra_outputs(${APP_NAME})

pico_enable_stdio_usb(${APP_NAME} 1)
pico_enable_stdio_uart(${APP_NAME} 0)
