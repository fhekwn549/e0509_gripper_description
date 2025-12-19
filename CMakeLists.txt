cmake_minimum_required(VERSION 3.8)
project(e0509_gripper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/gripper_joint_publisher.py
  scripts/gripper.py
  scripts/gripper_service_node.py
  scripts/gazebo_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
