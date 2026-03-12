cmake_minimum_required(VERSION 3.16.3)
project(sp_vision_hero_pneumatic)

set(CMAKE_CXX_STANDARD 17)

# --- 基础依赖 ---
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# --- ROS2 依赖 (改为使用变量名模式) ---
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(fastcdr QUIET)

# 修复 fastcdr Generator Expression 错误
if(NOT TARGET fastcdr)
    add_library(fastcdr INTERFACE IMPORTED)
endif()

# --- 包含目录 ---
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
)

add_subdirectory(tools)
add_subdirectory(io)

# --- 编译目标 ---
add_executable(camera_test examples/camera_test.cpp)
target_link_libraries(camera_test PUBLIC ${OpenCV_LIBS} fmt::fmt tools io)

add_executable(capture examples/capture.cpp)
target_link_libraries(capture PUBLIC ${OpenCV_LIBS} fmt::fmt tools io)

add_executable(angle_test tests/shooting_angle_test.cpp tasks/hanging_shooter.cpp)
target_link_libraries(angle_test PUBLIC tools io ${OpenCV_LIBS} fmt::fmt)