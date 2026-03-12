cmake_minimum_required(VERSION 3.16)

add_library(io STATIC
        hikrobot/hikrobot.cpp
        mindvision/mindvision.cpp
        usbcamera/usbcamera.cpp
        camera.cpp
        cboard.cpp
)

# 架构判定
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH_DIR "amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "arm64")
else()
    message(FATAL_ERROR "Unsupported architecture!")
endif()

target_include_directories(io PUBLIC hikrobot/include mindvision/include)
target_link_directories(io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${ARCH_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/${ARCH_DIR}
)

# 基础链接
target_link_libraries(io PUBLIC
        MvCameraControl
        MVSDK
        usb-1.0
        yaml-cpp
)

# ROS2 链接 (使用变量而非 Target)
if(rclcpp_FOUND)
    message(STATUS "ROS2 FOUND - Linking using variables")
    target_sources(io PRIVATE ros2/ros2.cpp ros2/subscribe2location.cpp)

    # 使用 ${包名_LIBRARIES} 变量，这种方式最原始但也最有效
    target_link_libraries(io PUBLIC
            ${rclcpp_LIBRARIES}
            ${std_msgs_LIBRARIES}
            ${geometry_msgs_LIBRARIES}
            fastcdr
    )
    # 补充头文件
    target_include_directories(io PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS})
endif()