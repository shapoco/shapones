cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico_nes_ws19579 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(pico_nes_ws19579)

set(SHAPONES_PATH ../../core)

target_sources(
    pico_nes_ws19579 PRIVATE 
    pico_nes_ws19579.cpp
    nes_rom.cpp
    ${SHAPONES_PATH}/src/cpu.cpp
    ${SHAPONES_PATH}/src/dma.cpp
    ${SHAPONES_PATH}/src/input.cpp
    ${SHAPONES_PATH}/src/interrupt.cpp
    ${SHAPONES_PATH}/src/memory.cpp
    ${SHAPONES_PATH}/src/ppu.cpp
    ${SHAPONES_PATH}/src/shapones.cpp
    ${SHAPONES_PATH}/src/utils_std.cpp
)

target_include_directories(
    pico_nes_ws19579 PRIVATE
    ${SHAPONES_PATH}/include
)

target_link_libraries(pico_nes_ws19579 PRIVATE pico_stdlib hardware_spi hardware_dma pico_multicore)
pico_add_extra_outputs(pico_nes_ws19579)
