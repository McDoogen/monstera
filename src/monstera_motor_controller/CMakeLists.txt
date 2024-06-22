#TODO:DS: Shoulrepod I just fetch pico-sdk as well? Since I am fetching micro-ros.... ?

cmake_minimum_required(VERSION 3.22)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(FetchContent)

project(monstera_motor_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(monstera_motor_controller_MAJOR_VERSION 0)
set(monstera_motor_controller_MINOR_VERSION 0)
set(monstera_motor_controller_PATCH_VERSION 0)
set(monstera_motor_controller_VERSION
  ${monstera_motor_controller_MAJOR_VERSION}.${monstera_motor_controller_MINOR_VERSION}.${monstera_motor_controller_PATCH_VERSION})

pico_sdk_init()

#TODO:DS: Can I add an 'IF' so we only log this if it is not available?
message("Fetching micro_ros repository...")
FetchContent_Declare(
  micro_ros
  GIT_REPOSITORY git@github.com:micro-ROS/micro_ros_raspberrypi_pico_sdk.git
  GIT_TAG        jazzy
)
FetchContent_MakeAvailable(micro_ros)

#TODO:DS: Should I change this location?
link_directories(${micro_ros_SOURCE_DIR}/libmicroros)

add_subdirectory(src)
