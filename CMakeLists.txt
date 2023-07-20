cmake_minimum_required(VERSION 3.12)
set(VCPKG_TARGET_TRIPLET arm-none-eabi)
project(test_project
    LANGUAGES ASM C)

include(cmake/utils.cmake)

set(cpu_def STM32F072xB)
set(compiler_triple thumbv6m-unknown-none-eabi)
set(cpu_options -mcpu=cortex-m0 -mthumb)
set(float_abi -mfloat-abi=soft)
# Project specific CMake setup
set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_C_COMPILER_TARGET ${compiler_triple})
set(CMAKE_CXX_COMPILER_TARGET ${compiler_triple})
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_definitions(${cpu_def})
add_compile_options(${cpu_options} ${float_abi})
add_link_options(${cpu_options} ${float_abi})
add_compile_options(-ffreestanding -ffunction-sections -fdata-sections -fsigned-char -fmessage-length=0)

add_subdirectory(src/)
