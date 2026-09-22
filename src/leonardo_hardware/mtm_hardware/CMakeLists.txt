# ==============================================================================
# 项目配置
# ==============================================================================
cmake_minimum_required(VERSION 4.2)

cmake_policy(SET CMP0167 OLD)

project(mtm_hardware)

set(CMAKE_CXX_STANDARD 20)
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
add_library(${PROJECT_NAME} SHARED
        src/fd_base/fd_hardware_base.cpp
        src/fd_left/fd_left_hardware.cpp
        src/fd_right/fd_right_hardware.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 供 ament_export_dependencies 使用的包名列表
set(DEPENDENCIES
        fd_vendor
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

# 供 target_link_libraries 使用的目标列表
set(MTM_HARDWARE_TARGET_LIBS
        fd_vendor::fd_vendor
        hardware_interface::hardware_interface
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)

# 定义编译宏 MTM_HARDWARE_BUILDING_DLL，用于控制动态链接库符号的导出可见性
target_compile_definitions(${PROJECT_NAME} PRIVATE MTM_HARDWARE_BUILDING_DLL)

# 链接依赖
target_link_libraries(${PROJECT_NAME} PUBLIC ${MTM_HARDWARE_TARGET_LIBS})

# ==============================================================================
# 插件导出
# ==============================================================================
pluginlib_export_plugin_description_file(hardware_interface mtm_hardware_plugin.xml)

# ==============================================================================
# 安装规则
# ==============================================================================
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY include/
        DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()