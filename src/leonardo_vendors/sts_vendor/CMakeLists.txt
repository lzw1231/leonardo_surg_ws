cmake_minimum_required(VERSION 3.28)
project(sts_vendor)

# 启用 compile_commands.json 导出，以支持 IDE 代码索引
set(_dummy ${CMAKE_EXPORT_COMPILE_COMMANDS})

# 设定 C++ 标准（若未指定则默认为 C++17）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖 ROS2 ament 构建系统
find_package(ament_cmake REQUIRED)

# 定义第三方动态库 (.so) 源文件路径
set(ST_LIB_SRC lib/libSCServo.so)

# ==========================================
# 定义接口库 (Interface Library)
# ==========================================
# 接口库不生成二进制文件，仅用于传递头文件路径和链接属性
add_library(${PROJECT_NAME} INTERFACE)

# 配置头文件包含目录（区分构建期与安装期路径）
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 构建期：指向源码 include 目录
        $<INSTALL_INTERFACE:include>                            # 安装期：指向安装后的 include 目录
)

# 配置链接依赖（指向安装后的库文件路径）
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${CMAKE_INSTALL_PREFIX}/lib/libSCServo.so
)

# ==========================================
# 安装规则 (Install)
# ==========================================
# 安装头文件至安装根目录
install(DIRECTORY include DESTINATION .)

# 安装动态库至 lib 目录
install(FILES ${ST_LIB_SRC} DESTINATION lib)

# 安装接口库目标并生成导出集
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# ==========================================
# 导出配置 (Export)
# ==========================================
# 导出 CMake 目标，支持其他 ROS2 包通过 find_package(sts_vendor) 查找并使用
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 生成 ament 包元数据
ament_package()