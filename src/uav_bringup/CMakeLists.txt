cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/home_visual_landing_test_node.py
  scripts/openvins_launch_shutdown_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
