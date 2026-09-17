cmake_minimum_required(VERSION 3.28)
project(leonardo_bringup)
set(_dummy ${CMAKE_EXPORT_COMPILE_COMMANDS})

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# ---------------- 依赖 ----------------
find_package(ament_cmake REQUIRED)

# ---------------- 安装 ----------------
install(DIRECTORY
        config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()