cmake_minimum_required(VERSION 3.13)

project(ekf-slam-example-tests)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

find_package(GTest REQUIRED)
enable_testing()

include_directories(../include/)

macro(make_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

make_test(ekf_slam_test)