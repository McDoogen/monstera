cmake_minimum_required(VERSION 3.22)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(hello_micro C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(hello_micro_MAJOR_VERSION 0)
set(hello_micro_MINOR_VERSION 0)
set(hello_micro_PATCH_VERSION 0)
set(hello_micro_VERSION
${hello_micro_MAJOR_VERSION}.${hello_micro_MINOR_VERSION}.${hello_micro_PATCH_VERSION})

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

pico_sdk_init()

add_subdirectory(src/hello)

#TODO:DS: Include Micro-ROS
# [0.715s] WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'hello_micro' because it has no 'install' target
# How do I include an Install target for micro-ros?
#TODO:DS: Can I use ament_cmake for micro-ros?