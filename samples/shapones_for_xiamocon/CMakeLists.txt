cmake_minimum_required(VERSION 3.12)

set(APP_NAME shapones_for_xiamocon)
set(APP_INC_DIR ./include)
set(APP_SRC_DIR ./src)

if (DEFINED XMC_PLATFORM)
    message("Using XMC_PLATFORM from environment ('${XMC_PLATFORM}')")
else()
    set(XMC_PLATFORM rp2350_pico_sdk)
endif()

if ("${XMC_PLATFORM}" STREQUAL "rp2350_pico_sdk")
    set(PICO_BOARD seeed_xiao_rp2350)
    include($ENV{XMC_REPO_PATH}/cpp/pico_sdk_import.cmake)
    include($ENV{XMC_REPO_PATH}/cpp/pico_extras_import.cmake)
endif()

project(${APP_NAME} C CXX ASM)

if ("${XMC_PLATFORM}" STREQUAL "rp2350_pico_sdk")
    pico_sdk_init()
else()
    message(FATAL_ERROR "Unsupported XMC_PLATFORM: ${XMC_PLATFORM}")
endif()

add_executable(${APP_NAME})

include($ENV{XMC_REPO_PATH}/cpp/xmc_sdk_import.cmake)

add_subdirectory(../../core shapones_core)

target_compile_options(${APP_NAME} PUBLIC -O2 -Wall)
target_compile_features(${APP_NAME} PUBLIC cxx_std_17)

file(GLOB CPP_FILES
    ${APP_SRC_DIR}/*.cpp
    ${APP_SRC_DIR}/*.c
)

target_sources(${APP_NAME} PRIVATE 
    ${CPP_FILES}
)

target_include_directories(${APP_NAME} PRIVATE
    ${APP_INC_DIR}
)

target_link_libraries(${APP_NAME} PRIVATE
    xmc_lib
    xmc_pfm
    shapones_core
)

pico_add_extra_outputs(${APP_NAME})
