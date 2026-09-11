cmake_minimum_required(VERSION 3.28)
project(mtm_description)

# 显式定义，避免未使用警告
set(_dummy ${CMAKE_EXPORT_COMPILE_COMMANDS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
        DIRECTORY urdf rviz bringup
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
