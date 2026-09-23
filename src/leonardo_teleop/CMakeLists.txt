cmake_minimum_required(VERSION 4.2)
project(leonardo_teleop)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif ()

# if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     add_compile_options(-Wno-c2y-extensions)
# endif ()

# ==============================================================================
# 依赖
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

# ==============================================================================
# pedal_publisher 可执行文件
# ==============================================================================
add_executable(pedal_publisher src/pedal_publisher.cpp)
target_include_directories(pedal_publisher
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(pedal_publisher PRIVATE ${LIBEVDEV_INCLUDE_DIRS})
target_link_directories(pedal_publisher PRIVATE ${LIBEVDEV_LIBRARY_DIRS})

target_link_libraries(pedal_publisher PRIVATE
        ${LIBEVDEV_LIBRARIES}
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
)

# ==============================================================================
# 安装
# ==============================================================================
install(TARGETS pedal_publisher
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)

ament_package()