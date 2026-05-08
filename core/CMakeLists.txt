
set(LIB_NAME shapones_core)
set(LIB_SRC_DIR src)
set(LIB_INC_DIR include)

project(${LIB_NAME} C CXX ASM)

file(GLOB CPP_FILES
    ${LIB_SRC_DIR}/*.cpp
    ${LIB_SRC_DIR}/*.c
)

add_library(${LIB_NAME} STATIC
    ${CPP_FILES}
)

target_compile_options(${LIB_NAME} PUBLIC -O2 -Wall)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_17)

target_include_directories(${LIB_NAME} PUBLIC
    ${LIB_INC_DIR}
)
