cmake_minimum_required(VERSION 3.10)
project(ekf-slam-example)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(YAML-CPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(include)
include_directories(libs/rtl/include)
include_directories(libs/yaml/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${custom_interfaces_INCLUDE_DIRS})
include_directories(${SDL2_INCLUDE_DIRS})

file(GLOB SOURCES src/*.cpp)

add_executable(ekf-slam-example ${SOURCES})
target_include_directories(ekf-slam-example PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(ekf-slam-example yaml-cpp ${SDL2_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(ekf-slam-example rclcpp cv_bridge std_msgs sensor_msgs geometry_msgs visualization_msgs)

add_subdirectory(libs/rtl/)
add_subdirectory(tests/)