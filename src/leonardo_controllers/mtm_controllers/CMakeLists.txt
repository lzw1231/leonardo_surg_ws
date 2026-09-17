# ==============================================================================
# mtm_controllers 的 CMake 配置
# ==============================================================================
cmake_minimum_required(VERSION 3.28)
cmake_policy(SET CMP0167 OLD)
project(mtm_controllers)

# 强制 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GCC/Clang 通用警告
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif ()

# Clang 22 引入的 C2y 扩展警告，来自 pluginlib 的 __COUNTER__
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-c2y-extensions)
endif ()

# ==============================================================================
# 依赖查找
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ament 依赖：用于 ament_target_dependencies 与 ament_export_dependencies
set(AMENT_DEPENDENCIES
        builtin_interfaces
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        rcutils
        realtime_tools
        geometry_msgs
        std_msgs
        eigen3_cmake_module
        Eigen3
)

# ==============================================================================
# 左侧控制器
# ==============================================================================
add_library(mtm_fd_left_controllers SHARED
        src/fd_left/fd_left_ee_pose_broadcaster.cpp
        # src/fd_left/xxx_left_controller.cpp
)
target_include_directories(mtm_fd_left_controllers
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# 构建 DLL 时使用 dllexport
target_compile_definitions(mtm_fd_left_controllers PRIVATE MTM_CONTROLLERS_BUILDING_DLL
)
# 让 pluginlib 不再依赖 Boost 的 function/bind
target_compile_definitions(mtm_fd_left_controllers PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
ament_target_dependencies(mtm_fd_left_controllers ${AMENT_DEPENDENCIES})

# ==============================================================================
# 右侧控制器
# ==============================================================================
add_library(mtm_fd_right_controllers SHARED
        src/fd_right/fd_right_ee_pose_broadcaster.cpp
        # src/fd_right/xxx_right_controller.cpp
)
target_include_directories(mtm_fd_right_controllers
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_definitions(mtm_fd_right_controllers PRIVATE MTM_CONTROLLERS_BUILDING_DLL)
target_compile_definitions(mtm_fd_right_controllers PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
ament_target_dependencies(mtm_fd_right_controllers ${AMENT_DEPENDENCIES})

# ==============================================================================
# pluginlib 插件描述
# ==============================================================================
pluginlib_export_plugin_description_file(controller_interface mtm_controllers_plugin.xml)

# ==============================================================================
# 安装
# ==============================================================================
install(TARGETS
        mtm_fd_left_controllers
        mtm_fd_right_controllers
        EXPORT export_mtm_controllers
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)
install(DIRECTORY include/
        DESTINATION include
)

# ==============================================================================
# ament 导出（供下游包 find_package 使用）
# ==============================================================================
ament_export_targets(export_mtm_controllers HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()