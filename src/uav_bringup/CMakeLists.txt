cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_orbbec_openvins_bootstrap.py
  scripts/apply_openvins_calibration_result.py
  scripts/monitor_topic_position.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
