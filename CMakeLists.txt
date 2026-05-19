cmake_minimum_required(VERSION 3.16)
project(robot_dm_can_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(dm_can_core STATIC
    dm_can/canbus/can_socket.cpp
    dm_can/damiao_motor/dm_motor.cpp
    dm_can/damiao_motor/dm_motor_control.cpp
)

target_include_directories(dm_can_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/dm_can
)

target_link_libraries(dm_can_core PUBLIC Threads::Threads)

add_executable(robot_dm_driver
    robot_dm_can_driver.cpp
)

target_link_libraries(robot_dm_driver PRIVATE dm_can_core)
