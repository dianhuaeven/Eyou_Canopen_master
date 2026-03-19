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

add_library(canopen_core
  src/cia402_state_machine.cpp
  src/shared_state.cpp
  src/axis_driver.cpp
  src/canopen_master.cpp
  src/canopen_robot_hw.cpp
  src/joints_config.cpp
  src/pdo_mapping.cpp
)

target_include_directories(canopen_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LELY_INCLUDE_DIRS}
)

target_link_libraries(canopen_core
  PUBLIC
    ${LELY_LIBRARIES}
    Threads::Threads
    yaml-cpp
)

target_compile_options(canopen_core
  PRIVATE
    ${LELY_CFLAGS_OTHER}
)

add_executable(canopen_hw_node
  src/main.cpp
)

target_link_libraries(canopen_hw_node
  PRIVATE
    canopen_core
)

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
target_link_libraries(test_shared_state PRIVATE canopen_core)
add_test(NAME test_shared_state COMMAND test_shared_state)

add_executable(test_canopen_robot_hw
  test/test_canopen_robot_hw.cpp
)
target_link_libraries(test_canopen_robot_hw PRIVATE canopen_core)
add_test(NAME test_canopen_robot_hw COMMAND test_canopen_robot_hw)

add_executable(test_unit_conversion
  test/test_unit_conversion.cpp
)
target_link_libraries(test_unit_conversion PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_unit_conversion)

add_executable(test_joints_config
  test/test_joints_config.cpp
)
target_link_libraries(test_joints_config PRIVATE canopen_core)
add_test(NAME test_joints_config COMMAND test_joints_config)

add_executable(test_canopen_master
  test/test_canopen_master.cpp
)
target_link_libraries(test_canopen_master PRIVATE canopen_core)
add_test(NAME test_canopen_master COMMAND test_canopen_master)

add_executable(test_shared_state_gtest
  test/test_shared_state_gtest.cpp
)
target_link_libraries(test_shared_state_gtest PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_shared_state_gtest)

add_executable(test_shared_state_concurrent
  test/test_shared_state_concurrent.cpp
)
target_link_libraries(test_shared_state_concurrent PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_shared_state_concurrent)
