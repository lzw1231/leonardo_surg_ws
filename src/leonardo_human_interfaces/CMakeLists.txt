cmake_minimum_required(VERSION 3.28)
project(leonardo_human_interfaces)

# 使用 C++20 标准 (适配 ROS 2 Jazzy / Ubuntu 24.04)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra)
endif ()

# ==============================================================================
# 依赖查找
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# 查找系统级 libevdev 依赖
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

# ==============================================================================
# 1. 脚踏板节点 foot_pedal_node
# ==============================================================================
add_executable(foot_pedal_node
src/foot_pedal/foot_pedal_node.cpp
)

target_include_directories(foot_pedal_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(foot_pedal_node PRIVATE
rclcpp::rclcpp
rclcpp_lifecycle::rclcpp_lifecycle
std_msgs::std_msgs__rosidl_typesupport_cpp
${LIBEVDEV_LIBRARIES}
)
target_include_directories(foot_pedal_node PRIVATE ${LIBEVDEV_INCLUDE_DIRS})
target_link_directories(foot_pedal_node PRIVATE ${LIBEVDEV_LIBRARY_DIRS})

# ==============================================================================
# 后续新增设备模板（例如 hand_controller_node，以后直接复制这一段）
# ==============================================================================
# add_executable(hand_controller_node
# src/hand_controller/hand_controller_node.cpp
# )
# target_include_directories(hand_controller_node PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(hand_controller_node PRIVATE
# rclcpp::rclcpp
# rclcpp_lifecycle::rclcpp_lifecycle
# std_msgs::std_msgs__rosidl_typesupport_cpp
# ${LIBEVDEV_LIBRARIES}
# )
# target_include_directories(hand_controller_node PRIVATE ${LIBEVDEV_INCLUDE_DIRS})
# target_link_directories(hand_controller_node PRIVATE ${LIBEVDEV_LIBRARY_DIRS})

# ==============================================================================
# 安装规则：在这里追加所有设备target
# ==============================================================================
install(TARGETS
  foot_pedal_node
  # hand_controller_node 新增后加到这里
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include
)

ament_package()
