# ==============================================================================
# 项目配置
# ==============================================================================
# 要求 CMake 版本（对齐 Jazzy + Clang-22）
cmake_minimum_required(VERSION 3.28)

# CMP0167: FindPython 策略（兼容 ament_cmake）
cmake_policy(SET CMP0167 OLD)

project(mtm_controllers)

# 强制 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# 依赖
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==============================================================================
# 构建
# ==============================================================================
# 左臂末端位姿广播器
# 未来扩展：右臂 fd_right_ee_pose_broadcaster 可另建一个库
add_library(fd_left_ee_pose_broadcaster SHARED
        src/fd_left/fd_left_ee_pose_broadcaster.cpp
)

# 头文件路径
# PUBLIC 保证依赖此库的目标也能访问 include 目录
target_include_directories(fd_left_ee_pose_broadcaster
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 依赖列表（一处定义，多处复用）
set(DEPENDENCIES
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        geometry_msgs
        std_msgs
)

# 定义 MTM_CONTROLLERS_BUILDING_DLL，供 visibility_control.hpp 区分导出/导入
target_compile_definitions(fd_left_ee_pose_broadcaster PRIVATE MTM_CONTROLLERS_BUILDING_DLL)

# 链接依赖
ament_target_dependencies(fd_left_ee_pose_broadcaster ${DEPENDENCIES})

# ==============================================================================
# 插件
# ==============================================================================
# 导出 controller_interface 插件描述
pluginlib_export_plugin_description_file(controller_interface mtm_controllers_plugin.xml)

# ==============================================================================
# 安装
# ==============================================================================
# 安装库并导出 target
install(TARGETS fd_left_ee_pose_broadcaster
        EXPORT export_fd_left_ee_pose_broadcaster
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# 安装头文件（包含 common/visibility_control.hpp）
install(DIRECTORY include/
        DESTINATION include
)

# 导出 CMake target，供其它包 find_package 使用
ament_export_targets(export_fd_left_ee_pose_broadcaster HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

# 由 package.xml 生成 ament 包信息
ament_package()