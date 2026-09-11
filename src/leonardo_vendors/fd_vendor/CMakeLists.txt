cmake_minimum_required(VERSION 3.28)
project(fd_vendor)

# 启用 compile_commands.json 导出，以支持 IDE 代码索引
set(_dummy ${CMAKE_EXPORT_COMPILE_COMMANDS})

# 定义 SDK 版本号
set(FD_SDK_VERSION "3.17.8")

# 设定 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖 ROS2 ament 构建系统
find_package(ament_cmake REQUIRED)

# ==========================================
# 1. 版本解析与库文件路径定义
# ==========================================
# 解析版本号主版本字段（用于生成兼容性符号链接名称）
string(REPLACE "." ";" VERSION_SDK_LIST ${FD_SDK_VERSION})
list(GET VERSION_SDK_LIST 0 VERSION_SDK_MAJOR)

# 定义源动态库文件的完整路径
set(DHD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.${FD_SDK_VERSION})
set(DRD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.${FD_SDK_VERSION})


# ==========================================
# 2. 定义接口库 (Interface Library)
# ==========================================
add_library(${PROJECT_NAME} INTERFACE)

# 配置头文件包含目录（区分构建期与安装期路径）
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 构建期：指向源码 include 目录
        $<INSTALL_INTERFACE:include>                            # 安装期：指向安装后的 include 目录
)

# 配置链接依赖（指向安装后的库文件路径）
# 注意：此处使用安装前缀路径，假设调用方已正确配置环境或链接阶段可访问该路径
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${CMAKE_INSTALL_PREFIX}/lib/libdhd.so.${VERSION_SDK_MAJOR}
        ${CMAKE_INSTALL_PREFIX}/lib/libdrd.so.${VERSION_SDK_MAJOR}
)


# ==========================================
# 3. 安装规则 (Install)
# ==========================================
# 安装头文件至安装根目录
install(
        DIRECTORY include
        DESTINATION .
)

# 安装动态库并重命名为带主版本号的格式（如 libdhd.so.3），以符合 Linux 库命名规范
install(FILES ${DHD_LIB_SRC} DESTINATION lib RENAME libdhd.so.${VERSION_SDK_MAJOR})
install(FILES ${DRD_LIB_SRC} DESTINATION lib RENAME libdrd.so.${VERSION_SDK_MAJOR})

# 安装接口库目标并生成导出集
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

# ==========================================
# 4. 导出配置 (Export)
# ==========================================
# 导出 CMake 目标，支持其他包通过 find_package(fd_vendor) 查找并使用
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 生成 ament 包元数据
ament_package()