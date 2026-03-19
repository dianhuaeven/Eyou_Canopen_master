cmake_minimum_required(VERSION 3.10)
project(canopen_hw LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY REQUIRED liblely-coapp liblely-io2 liblely-ev liblely-co)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
include(GoogleTest)

include(FetchContent)
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.12.0
)
FetchContent_MakeAvailable(spdlog)

add_library(canopen_core
  src/cia402_state_machine.cpp
  src/shared_state.cpp
  src/axis_driver.cpp
  src/canopen_master.cpp
  src/canopen_robot_hw.cpp
  src/joints_config.cpp
  src/pdo_mapping.cpp
  src/lifecycle_manager.cpp
  src/sdo_accessor.cpp
)

target_include_directories(canopen_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  SYSTEM PUBLIC
    ${LELY_INCLUDE_DIRS}
)

target_link_libraries(canopen_core
  PUBLIC
    ${LELY_LIBRARIES}
    Threads::Threads
    yaml-cpp
    spdlog::spdlog
)

target_compile_options(canopen_core
  PRIVATE
    -Wall -Wextra -Werror
    ${LELY_CFLAGS_OTHER}
)

add_executable(canopen_hw_node
  src/main.cpp
)

target_link_libraries(canopen_hw_node
  PRIVATE
    canopen_core
)

# ── ROS1 适配层（可选） ──────────────────────────────────
option(WITH_ROS "Build ROS1 ros_control adapter" OFF)

if(WITH_ROS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    hardware_interface
    controller_manager
  )

  add_library(canopen_ros_adapter
    src/canopen_robot_hw_ros.cpp
  )
  target_include_directories(canopen_ros_adapter
    PUBLIC
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(canopen_ros_adapter
    PUBLIC
      canopen_core
      ${catkin_LIBRARIES}
  )
  target_compile_options(canopen_ros_adapter
    PRIVATE
      -Wall -Wextra -Werror
  )

  add_executable(canopen_hw_ros_node
    src/canopen_hw_ros_node.cpp
  )
  target_link_libraries(canopen_hw_ros_node
    PRIVATE
      canopen_ros_adapter
  )
endif()

# Tests
enable_testing()

add_executable(test_state_machine
  test/test_state_machine.cpp
)
target_link_libraries(test_state_machine PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_state_machine)

add_executable(test_shared_state
  test/test_shared_state.cpp
)
target_link_libraries(test_shared_state PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_shared_state)

add_executable(test_canopen_robot_hw
  test/test_canopen_robot_hw.cpp
)
target_link_libraries(test_canopen_robot_hw PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_canopen_robot_hw)

add_executable(test_unit_conversion
  test/test_unit_conversion.cpp
)
target_link_libraries(test_unit_conversion PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_unit_conversion)

add_executable(test_joints_config
  test/test_joints_config.cpp
)
target_link_libraries(test_joints_config PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_joints_config)

add_executable(test_canopen_master
  test/test_canopen_master.cpp
)
target_link_libraries(test_canopen_master PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_canopen_master)

add_executable(test_startup_integration
  test/test_startup_integration.cpp
)
target_link_libraries(test_startup_integration PRIVATE canopen_core GTest::gtest_main)
target_compile_definitions(test_startup_integration PRIVATE
  PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  PROJECT_BINARY_DIR="${CMAKE_BINARY_DIR}"
)
gtest_discover_tests(test_startup_integration)

add_executable(test_shared_state_concurrent
  test/test_shared_state_concurrent.cpp
)
target_link_libraries(test_shared_state_concurrent PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_shared_state_concurrent)

add_executable(test_boundary_cases
  test/test_boundary_cases.cpp
)
target_link_libraries(test_boundary_cases PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_boundary_cases)

add_executable(test_lifecycle_manager
  test/test_lifecycle_manager.cpp
)
target_link_libraries(test_lifecycle_manager PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_lifecycle_manager)

add_executable(test_manual_control
  test/test_manual_control.cpp
)
target_link_libraries(test_manual_control PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_manual_control)

add_executable(test_sdo_accessor
  test/test_sdo_accessor.cpp
)
target_link_libraries(test_sdo_accessor PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_sdo_accessor)
