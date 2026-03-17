cmake_minimum_required(VERSION 3.10)
project(HardcoreRobotTest)

set(CMAKE_CXX_STANDARD 17)

# 强制告诉 CMake 去哪里找 Pinocchio (极其贴心的防报错设计)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

# 寻找三大神器
find_package(pinocchio REQUIRED)
find_package(ruckig REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY REQUIRED liblely-coapp liblely-io2 liblely-ev liblely-co)


add_executable(my_node main.cpp)

# 链接它们
target_include_directories(my_node PRIVATE ${LELY_INCLUDE_DIRS})
target_link_libraries(my_node PRIVATE 
    pinocchio::pinocchio
    ruckig::ruckig
    ${LELY_LIBRARIES}
)
