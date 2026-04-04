cmake_minimum_required(VERSION 3.8)
project(uav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)

add_library(${PROJECT_NAME}_px4_odom_adapter_logic
  src/px4_odom_adapter_logic.cpp
)
target_compile_features(${PROJECT_NAME}_px4_odom_adapter_logic PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_px4_odom_adapter_logic PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_px4_odom_adapter_logic
  px4_msgs
  tf2
)

add_library(${PROJECT_NAME}_uav_state_bridge_logic
  src/uav_state_bridge_logic.cpp
)
target_compile_features(${PROJECT_NAME}_uav_state_bridge_logic PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_uav_state_bridge_logic PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_uav_state_bridge_logic
  px4_msgs
  tf2
)

add_library(${PROJECT_NAME}_sim_vio_odometry_logic
  src/sim_vio_odometry_logic.cpp
)
target_compile_features(${PROJECT_NAME}_sim_vio_odometry_logic PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_sim_vio_odometry_logic PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_sim_vio_odometry_logic
  tf2
)

add_executable(mono_camera_source_node
  src/mono_camera_source_node.cpp
)
target_compile_features(mono_camera_source_node PUBLIC cxx_std_17)
target_include_directories(mono_camera_source_node PRIVATE include)
target_include_directories(mono_camera_source_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(mono_camera_source_node
  rclcpp
  sensor_msgs
)
target_link_libraries(mono_camera_source_node
  ${OpenCV_LIBS}
)

add_executable(mono_video_recorder_node
  src/mono_video_recorder_node.cpp
)
target_compile_features(mono_video_recorder_node PUBLIC cxx_std_17)
target_include_directories(mono_video_recorder_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(mono_video_recorder_node
  rclcpp
  sensor_msgs
)
target_link_libraries(mono_video_recorder_node
  ${OpenCV_LIBS}
)

add_executable(px4_planar_state_reader_node
  src/px4_planar_state_reader_node.cpp
)
target_compile_features(px4_planar_state_reader_node PUBLIC cxx_std_17)
ament_target_dependencies(px4_planar_state_reader_node
  rclcpp
  nav_msgs
  px4_msgs
)

add_executable(px4_odom_adapter_node
  src/px4_odom_adapter_node.cpp
)
target_compile_features(px4_odom_adapter_node PUBLIC cxx_std_17)
target_include_directories(px4_odom_adapter_node PRIVATE include)
ament_target_dependencies(px4_odom_adapter_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(px4_odom_adapter_node
  ${PROJECT_NAME}_px4_odom_adapter_logic
)

add_executable(uav_state_bridge_node
  src/uav_state_bridge_node.cpp
)
target_compile_features(uav_state_bridge_node PUBLIC cxx_std_17)
target_include_directories(uav_state_bridge_node PRIVATE include)
ament_target_dependencies(uav_state_bridge_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(uav_state_bridge_node
  ${PROJECT_NAME}_uav_state_bridge_logic
)

add_executable(trajectory_path_publisher_node
  src/trajectory_path_publisher_node.cpp
)
target_compile_features(trajectory_path_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(trajectory_path_publisher_node
  geometry_msgs
  nav_msgs
  rclcpp
)

add_executable(uav_control_node
  src/uav_control_node.cpp
  src/math_utils.cpp
)
target_compile_features(uav_control_node PUBLIC cxx_std_17)
target_include_directories(uav_control_node PRIVATE include)
ament_target_dependencies(uav_control_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_srvs
  quadrotor_msgs
  px4_msgs
  tf2
)

add_executable(height_measurement_bridge_node
  src/height_measurement_bridge_node.cpp
)
target_compile_features(height_measurement_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(height_measurement_bridge_node
  px4_msgs
  rclcpp
  sensor_msgs
)

add_executable(sim_vio_odometry_node
  src/sim_vio_odometry_node.cpp
)
target_compile_features(sim_vio_odometry_node PUBLIC cxx_std_17)
target_include_directories(sim_vio_odometry_node PRIVATE include)
ament_target_dependencies(sim_vio_odometry_node
  nav_msgs
  rclcpp
)
target_link_libraries(sim_vio_odometry_node
  ${PROJECT_NAME}_sim_vio_odometry_logic
)

add_executable(openvins_px4_vision_bridge_node
  src/openvins_px4_vision_bridge_node.cpp
  src/math_utils.cpp
)
target_compile_features(openvins_px4_vision_bridge_node PUBLIC cxx_std_17)
target_include_directories(openvins_px4_vision_bridge_node PRIVATE include)
ament_target_dependencies(openvins_px4_vision_bridge_node
  nav_msgs
  px4_msgs
  rclcpp
  std_msgs
  tf2
)

add_executable(openvins_state_bridge_node
  src/openvins_state_bridge_node.cpp
)
target_compile_features(openvins_state_bridge_node PUBLIC cxx_std_17)
target_include_directories(openvins_state_bridge_node PRIVATE include)
ament_target_dependencies(openvins_state_bridge_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(openvins_state_bridge_node
  ${PROJECT_NAME}_uav_state_bridge_logic
)

install(TARGETS
  ${PROJECT_NAME}_px4_odom_adapter_logic
  ${PROJECT_NAME}_sim_vio_odometry_logic
  mono_camera_source_node
  mono_video_recorder_node
  px4_planar_state_reader_node
  px4_odom_adapter_node
  uav_state_bridge_node
  trajectory_path_publisher_node
  uav_control_node
  height_measurement_bridge_node
  sim_vio_odometry_node
  openvins_px4_vision_bridge_node
  openvins_state_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sim_vio_odometry_logic
    test/test_sim_vio_odometry_logic.cpp
  )
  if(TARGET test_sim_vio_odometry_logic)
    target_link_libraries(test_sim_vio_odometry_logic ${PROJECT_NAME}_sim_vio_odometry_logic)
    ament_target_dependencies(test_sim_vio_odometry_logic tf2)
  endif()
endif()

ament_package()
