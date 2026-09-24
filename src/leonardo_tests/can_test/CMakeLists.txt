cmake_minimum_required(VERSION 4.2)
project(can_receiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# create executable
add_executable(can_receiver src/can_receiver.cpp)

# link libraries (replace the old ament_target_dependencies)
target_link_libraries(can_receiver
  ${rclcpp_TARGETS}
)

install(TARGETS
  can_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()