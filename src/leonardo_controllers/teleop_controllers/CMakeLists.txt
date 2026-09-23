# ==============================================================================
# teleop_controllers 的 CMake 配置
# ==============================================================================
cmake_minimum_required(VERSION 4.2)
# cmake_policy(SET CMP0167 OLD)
project(teleop_controllers)

# 强制 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GCC/Clang 通用警告
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif ()

# Clang 引入的 C2y 扩展警告，来自 pluginlib 的 __COUNTER__
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-c2y-extensions)
endif ()

# ==============================================================================
# 依赖查找
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)


# 供 ament_export_dependencies 使用的包名列表
set(AMENT_DEPENDENCIES
        rclcpp
        rclcpp_lifecycle
        controller_interface
        pluginlib
        std_msgs
)

# 供 target_link_libraries 使用的目标列表
set(TELEOP_TARGET_LIBS
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        controller_interface::controller_interface
        pluginlib::pluginlib
        ${std_msgs_TARGETS}
)

# ==============================================================================
# 遥操作映射控制器
# ==============================================================================
add_library(teleop_controllers SHARED
        src/pedal_mapping/pedal_mapping_controller.cpp
)
target_include_directories(teleop_controllers
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# 构建 DLL 时使用 dllexport
# target_compile_definitions(teleop_controllers PRIVATE TELEOP_CONTROLLERS_BUILDING_DLL)

# 让 pluginlib 不再依赖 Boost 的 function/bind
target_compile_definitions(teleop_controllers PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
target_link_libraries(teleop_controllers PUBLIC ${TELEOP_TARGET_LIBS})

# ==============================================================================
# pluginlib 插件描述
# ==============================================================================
pluginlib_export_plugin_description_file(controller_interface teleop_controllers_plugin.xml)

# ==============================================================================
# 安装
# ==============================================================================
install(TARGETS
        teleop_controllers
        EXPORT export_teleop_controllers
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
ament_export_targets(export_teleop_controllers HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()