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

target_sources(
    pico_nes_ws19804 PRIVATE 
    common.cpp
    ws19804.cpp
    pico_nes_ws19804.cpp
    boot_menu.cpp
    ${SHAPONES_PATH}/src/cpu.cpp
    ${SHAPONES_PATH}/src/dma.cpp
    ${SHAPONES_PATH}/src/input.cpp
    ${SHAPONES_PATH}/src/interrupt.cpp
    ${SHAPONES_PATH}/src/memory.cpp
    ${SHAPONES_PATH}/src/ppu.cpp
    ${SHAPONES_PATH}/src/apu.cpp
    ${SHAPONES_PATH}/src/shapones.cpp
    ${SHAPONES_PATH}/src/utils_std.cpp
    ${FATFS_PATH}/ff.c
    ${FATFS_PATH}/ffsystem.c
    ${FATFS_PATH}/ffunicode.c
    ${FATFS_PATH}/mmc_pico_spi.c
    ../font/mono8x16/mono8x16.cpp
    ../font/mono8x16/mono8x16_bmp.cpp
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
    hardware_pwm
    hardware_dma
    hardware_irq
    pico_multicore
)

pico_add_extra_outputs(pico_nes_ws19804)
