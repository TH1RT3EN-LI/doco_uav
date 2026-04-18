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
  scripts/monitor_topic_position.py
  scripts/sweep_openvins_orbbec.py
  scripts/uav_demo_search_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_openvins_ov_body_tf
    test/test_openvins_ov_body_tf.py
  )

  ament_add_pytest_test(
    test_demo_search_coordinator
    test/test_demo_search_coordinator.py
  )
endif()

ament_package()
