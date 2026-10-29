cmake_minimum_required(VERSION 3.28)
project(leonardo_teleop)

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
# 可执行文件构建
# ==============================================================================
add_executable(pedal_teleop_node 
    src/pedal/pedal_teleop_node.cpp
)

# 包含项目头文件
target_include_directories(pedal_teleop_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(pedal_teleop_node PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    std_msgs::std_msgs__rosidl_typesupport_cpp
    ${LIBEVDEV_LIBRARIES}
)

# 链接 libevdev 的头文件和库路径
target_include_directories(pedal_teleop_node PRIVATE ${LIBEVDEV_INCLUDE_DIRS})
target_link_directories(pedal_teleop_node PRIVATE ${LIBEVDEV_LIBRARY_DIRS})

# ==============================================================================
# 安装规则
# ==============================================================================
install(TARGETS pedal_teleop_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()
