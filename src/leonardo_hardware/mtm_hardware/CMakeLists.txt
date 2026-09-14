# ==============================================================================
# 项目配置
# ==============================================================================
# 要求 CMake 版本 (适配 Jazzy + Clang-22)
cmake_minimum_required(VERSION 3.28)

# CMP0167: FindPython 策略设置 (兼容 ament_cmake)
cmake_policy(SET CMP0167 OLD)

project(mtm_hardware)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# 依赖查找
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fd_vendor REQUIRED)

# ==============================================================================
# 库构建
# ==============================================================================
# 创建共享库
# 注意：如果后续添加了 PSM 或 Right MTM 的代码，请在此处追加源文件路径
add_library(${PROJECT_NAME} SHARED
        src/fd_base/fd_hardware_base.cpp
        src/fd_left/fd_left_hardware.cpp
        src/fd_right/fd_right_hardware.cpp
)

# 设置包含目录
# PUBLIC 意味着依赖此库的其他目标也会自动包含这些目录
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 定义依赖项
set(DEPENDENCIES
        fd_vendor
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

# 定义编译宏 MTM_HARDWARE_BUILDING_DLL，用于控制动态链接库符号的导出可见性
target_compile_definitions(${PROJECT_NAME} PRIVATE MTM_HARDWARE_BUILDING_DLL)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# ==============================================================================
# 插件导出
# ==============================================================================
# 导出 hardware_interface 插件描述文件
pluginlib_export_plugin_description_file(hardware_interface mtm_hardware_plugin.xml)

# ==============================================================================
# 安装规则
# ==============================================================================
# 安装库文件
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# 安装头文件 (递归安装 include 下的所有文件，包括 common/visibility_control.hpp)
install(DIRECTORY include/
        DESTINATION include
)

# 导出 CMake 目标和依赖，供其他包 find_package 使用
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

# 生成 package.xml 对应的 CMake 配置
ament_package()