cmake_minimum_required(VERSION 3.28) 
project(kinematics_test) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== 查找依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 

# ==================== 包含头文件目录 ====================
include_directories(include)

# ==================== 编译目标 ====================
add_executable(surgical_kinematics 
  src/surgical_kinematics.cpp
)

ament_target_dependencies(surgical_kinematics
  rclcpp
)

install(TARGETS
  surgical_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()