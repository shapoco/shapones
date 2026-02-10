cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

set(APP_NAME shapones_xiao_rp2350)

project(${APP_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(${APP_NAME})
add_compile_options(-O2 -fjump-tables)
#add_compile_definitions(SHAPONES_ENABLE_LOG=1)

set(SHAPONES_PATH ../../core)
set(FATFS_PATH ../fatfs/source)

file(GLOB CPP_FILES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    ${SHAPONES_PATH}/src/*.cpp
    ${FATFS_PATH}/ff.c
    ${FATFS_PATH}/ffsystem.c
    ${FATFS_PATH}/ffunicode.c
)

target_sources(
    ${APP_NAME} PRIVATE 
    ${CPP_FILES}
)

target_include_directories(
    ${APP_NAME} PRIVATE
    ./include
    ${SHAPONES_PATH}/include
    ${FATFS_PATH}
)

target_link_libraries(
    ${APP_NAME} PRIVATE
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_adc
    hardware_pwm
    hardware_dma
    hardware_irq
    hardware_flash
    hardware_sync
    pico_multicore
)

pico_add_extra_outputs(${APP_NAME})

pico_enable_stdio_usb(${APP_NAME} 1)
pico_enable_stdio_uart(${APP_NAME} 0)
