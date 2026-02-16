cmake_minimum_required(VERSION 3.8)
project(ur5e_contactile_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(ur5e_contactile_actions_node src/ur5e_contactile_actions.cpp)
ament_target_dependencies(ur5e_contactile_actions_node
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS ur5e_contactile_actions_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

