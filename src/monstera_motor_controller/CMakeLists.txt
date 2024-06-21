cmake_minimum_required(VERSION 3.8)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(monstera_motor_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(monstera_motor_controller_MAJOR_VERSION 0)
set(monstera_motor_controller_MINOR_VERSION 0)
set(monstera_motor_controller_PATCH_VERSION 0)
set(monstera_motor_controller_VERSION
  ${monstera_motor_controller_MAJOR_VERSION}.${monstera_motor_controller_MINOR_VERSION}.${monstera_motor_controller_PATCH_VERSION})

# TODO:DS: What is this for?
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

pico_sdk_init()

add_subdirectory(src)