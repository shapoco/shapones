cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico_nes_ws19804 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(pico_nes_ws19804)

pico_generate_pio_header(pico_nes_ws19804 ${CMAKE_CURRENT_LIST_DIR}/ws19804.pio)

set(SHAPONES_PATH ../../core)
set(FATFS_PATH ../fatfs/source)

file(GLOB CPP_FILES
    ./*.cpp
    ${SHAPONES_PATH}/src/*.cpp
    ${FATFS_PATH}/ff.c
    ${FATFS_PATH}/ffsystem.c
    ${FATFS_PATH}/ffunicode.c
    ${FATFS_PATH}/mmc_pico_spi.c
    ../font/mono8x16/*.cpp
)

target_sources(
    pico_nes_ws19804 PRIVATE 
    ${CPP_FILES}
)

target_include_directories(
    pico_nes_ws19804 PRIVATE
    .
    ${SHAPONES_PATH}/include
    ${FATFS_PATH}
    ../font/mono8x16
)

target_link_libraries(
    pico_nes_ws19804 PRIVATE
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_pwm
    hardware_dma
    hardware_irq
    hardware_flash
    hardware_sync
    pico_multicore
)

pico_add_extra_outputs(pico_nes_ws19804)
