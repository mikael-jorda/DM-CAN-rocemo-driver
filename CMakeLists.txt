cmake_minimum_required(VERSION 3.16)
project(robot_dm_can_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If a conda environment is active, use it to resolve dependencies by default.
if(DEFINED ENV{CONDA_PREFIX} AND EXISTS "$ENV{CONDA_PREFIX}")
  # Ensure find_package searches the active environment first
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  message(STATUS "Conda environment detected: $ENV{CONDA_PREFIX}")

  # Hint Boost resolution to the conda environment to avoid mixing system/Homebrew and conda Boost
  if(NOT DEFINED BOOST_ROOT)
    set(BOOST_ROOT "$ENV{CONDA_PREFIX}" CACHE PATH "Root of Boost installation" FORCE)
    set(BOOST_NO_SYSTEM_PATHS ON CACHE BOOL "Prefer non-system paths for Boost" FORCE)
  endif()
endif()

# Include glaze
include(FetchContent)
    FetchContent_Declare(
  glaze
  GIT_REPOSITORY https://github.com/stephenberry/glaze.git
  GIT_TAG v5.2.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glaze)

find_package(pinocchio 3.8.0 REQUIRED)
find_package(Threads REQUIRED)

find_library(HIREDIS_LIBRARY hiredis)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(redis_client STATIC
    redis/RedisClient.cpp
)
target_link_libraries(redis_client PUBLIC pinocchio::pinocchio glaze::glaze ${HIREDIS_LIBRARY})

add_library(dm_can_core STATIC
    dm_can/canbus/can_device_collection.cpp
    dm_can/canbus/can_socket.cpp
    dm_can/damiao_motor/dm_motor.cpp
    dm_can/damiao_motor/dm_motor_control.cpp
    dm_can/damiao_motor/dm_motor_device.cpp
    dm_can/damiao_motor/dm_motor_device_collection.cpp
)

target_include_directories(dm_can_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/dm_can
)

target_link_libraries(dm_can_core PUBLIC Threads::Threads)

add_executable(robot_dm_driver
    robot_dm_can_driver.cpp
)

target_link_libraries(robot_dm_driver PRIVATE dm_can_core redis_client)
