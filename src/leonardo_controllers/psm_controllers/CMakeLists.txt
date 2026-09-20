# ==============================================================================
# psm_controllers 的 CMake 配置
# ==============================================================================
cmake_minimum_required(VERSION 3.28)
cmake_policy(SET CMP0167 OLD)
project(psm_controllers)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# ament 依赖：用于 ament_target_dependencies 与 ament_export_dependencies
set(AMENT_DEPENDENCIES
        rclcpp
        rclcpp_lifecycle
        controller_interface
        example_interfaces
        pluginlib
)

# ==============================================================================
# psm从端控制器
# ==============================================================================
add_library(psm_controllers SHARED
        src/ft3215/ft3215_a_controller.cpp
        src/ft3215/ft3215_b_controller.cpp
)
target_include_directories(psm_controllers
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# 构建 DLL 时使用 dllexport
target_compile_definitions(psm_controllers PRIVATE PSM_CONTROLLERS_BUILDING_DLL
)
# 让 pluginlib 不再依赖 Boost 的 function/bind
target_compile_definitions(psm_controllers PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
ament_target_dependencies(psm_controllers ${AMENT_DEPENDENCIES})

# ==============================================================================
# pluginlib 插件描述
# ==============================================================================
pluginlib_export_plugin_description_file(controller_interface psm_controllers_plugin.xml)

# ==============================================================================
# 安装
# ==============================================================================
install(TARGETS
        psm_controllers
        EXPORT export_psm_controllers
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
ament_export_targets(export_psm_controllers HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()