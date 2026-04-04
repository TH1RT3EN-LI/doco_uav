import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    visual_landing_share = get_package_share_directory("uav_visual_landing")

    camera_backend = LaunchConfiguration("camera_backend")
    camera_device = LaunchConfiguration("camera_device")
    camera_fourcc = LaunchConfiguration("camera_fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_fps = LaunchConfiguration("camera_fps")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_name = LaunchConfiguration("camera_name")
    base_frame_id = LaunchConfiguration("base_frame_id")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    fx = LaunchConfiguration("fx")
    fy = LaunchConfiguration("fy")
    cx = LaunchConfiguration("cx")
    cy = LaunchConfiguration("cy")
    camera_hfov_rad = LaunchConfiguration("camera_hfov_rad")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_openvins_stack = LaunchConfiguration("start_openvins_stack")
    openvins_start_rviz = LaunchConfiguration("openvins_start_rviz")
    openvins_config_path = LaunchConfiguration("openvins_config_path")
    height_measurement_transport = LaunchConfiguration("height_measurement_transport")
    height_measurement_topic = LaunchConfiguration("height_measurement_topic")
    height_measurement_mode = LaunchConfiguration("height_measurement_mode")
    height_measurement_frame_id = LaunchConfiguration("height_measurement_frame_id")
    motion_guard_enabled = LaunchConfiguration("motion_guard_enabled")
    motion_guard_soft_dwell_s = LaunchConfiguration("motion_guard_soft_dwell_s")
    motion_guard_pose_gap_reset_s = LaunchConfiguration("motion_guard_pose_gap_reset_s")
    motion_guard_soft_xy_mps = LaunchConfiguration("motion_guard_soft_xy_mps")
    motion_guard_soft_z_mps = LaunchConfiguration("motion_guard_soft_z_mps")
    motion_guard_soft_yaw_radps = LaunchConfiguration("motion_guard_soft_yaw_radps")
    motion_guard_hard_xy_mps = LaunchConfiguration("motion_guard_hard_xy_mps")
    motion_guard_hard_z_mps = LaunchConfiguration("motion_guard_hard_z_mps")
    motion_guard_hard_yaw_radps = LaunchConfiguration("motion_guard_hard_yaw_radps")
    motion_guard_feedback_hard_xy_mps = LaunchConfiguration("motion_guard_feedback_hard_xy_mps")
    motion_guard_feedback_hard_z_mps = LaunchConfiguration("motion_guard_feedback_hard_z_mps")
    motion_guard_pose_soft_xy_step_m = LaunchConfiguration("motion_guard_pose_soft_xy_step_m")
    motion_guard_pose_soft_z_step_m = LaunchConfiguration("motion_guard_pose_soft_z_step_m")
    motion_guard_pose_soft_yaw_step_rad = LaunchConfiguration("motion_guard_pose_soft_yaw_step_rad")
    motion_guard_pose_hard_xy_step_m = LaunchConfiguration("motion_guard_pose_hard_xy_step_m")
    motion_guard_pose_hard_z_step_m = LaunchConfiguration("motion_guard_pose_hard_z_step_m")
    motion_guard_pose_hard_yaw_step_rad = LaunchConfiguration("motion_guard_pose_hard_yaw_step_rad")
    tag_size_m = LaunchConfiguration("tag_size_m")
    publish_trajectory = LaunchConfiguration("publish_trajectory")
    trajectory_input_odom_topic = LaunchConfiguration("trajectory_input_odom_topic")
    trajectory_output_topic = LaunchConfiguration("trajectory_output_topic")
    trajectory_max_samples = LaunchConfiguration("trajectory_max_samples")
    trajectory_min_sample_distance_m = LaunchConfiguration("trajectory_min_sample_distance_m")
    trajectory_min_sample_period_s = LaunchConfiguration("trajectory_min_sample_period_s")

    visual_landing_config = os.path.join(visual_landing_share, "config", "visual_landing_stable.yaml")
    mono_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "mono_camera.launch.py")),
        launch_arguments={
            "backend": camera_backend,
            "device": camera_device,
            "fourcc": camera_fourcc,
            "image_width": image_width,
            "image_height": image_height,
            "fps": camera_fps,
            "camera_name": camera_name,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "base_frame_id": base_frame_id,
            "camera_frame_id": camera_frame_id,
            "camera_x": camera_x,
            "camera_y": camera_y,
            "camera_z": camera_z,
            "camera_roll": camera_roll,
            "camera_pitch": camera_pitch,
            "camera_yaw": camera_yaw,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "camera_hfov_rad": camera_hfov_rad,
            "camera_info_url": camera_info_url,
        }.items(),
    )

    vehicle_local_position_topic = "/fmu/out/vehicle_local_position"
    vehicle_odometry_topic = "/fmu/out/vehicle_odometry"
    vehicle_status_topic = "/fmu/out/vehicle_status"
    offboard_mode_topic = "/fmu/in/offboard_control_mode"
    trajectory_setpoint_topic = "/fmu/in/trajectory_setpoint"
    vehicle_command_topic = "/fmu/in/vehicle_command"
    distance_sensor_topic = "/fmu/out/distance_sensor"

    openvins_orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "openvins_orbbec.launch.py")
        ),
        condition=IfCondition(start_openvins_stack),
        launch_arguments={
            "start_px4_vision_bridge": "false",
            "start_rviz": openvins_start_rviz,
            "use_sim_time": use_sim_time,
            "openvins_config_path": openvins_config_path,
            "base_frame_id": base_frame_id,
        }.items(),
    )

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge_px4_debug",
        output="screen",
        parameters=[
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_odometry_topic": vehicle_odometry_topic},
            {"base_frame_id": base_frame_id},
            {"use_sim_time": use_sim_time},
            {"output_odometry_topic": "/uav/state/odometry_px4"},
            {"publish_tf": False},
            {"publish_map_to_odom_tf": False},
        ],
    )

    uav_control = Node(
        package="uav_bridge",
        executable="uav_control_node",
        name="uav_control",
        output="screen",
        parameters=[
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
            {"base_frame_id": base_frame_id},
            {"use_sim_time": use_sim_time},
            {"motion_guard_enabled": motion_guard_enabled},
            {"motion_guard_soft_dwell_s": motion_guard_soft_dwell_s},
            {"motion_guard_pose_gap_reset_s": motion_guard_pose_gap_reset_s},
            {"motion_guard_soft_xy_mps": motion_guard_soft_xy_mps},
            {"motion_guard_soft_z_mps": motion_guard_soft_z_mps},
            {"motion_guard_soft_yaw_radps": motion_guard_soft_yaw_radps},
            {"motion_guard_hard_xy_mps": motion_guard_hard_xy_mps},
            {"motion_guard_hard_z_mps": motion_guard_hard_z_mps},
            {"motion_guard_hard_yaw_radps": motion_guard_hard_yaw_radps},
            {"motion_guard_feedback_hard_xy_mps": motion_guard_feedback_hard_xy_mps},
            {"motion_guard_feedback_hard_z_mps": motion_guard_feedback_hard_z_mps},
            {"motion_guard_pose_soft_xy_step_m": motion_guard_pose_soft_xy_step_m},
            {"motion_guard_pose_soft_z_step_m": motion_guard_pose_soft_z_step_m},
            {"motion_guard_pose_soft_yaw_step_rad": motion_guard_pose_soft_yaw_step_rad},
            {"motion_guard_pose_hard_xy_step_m": motion_guard_pose_hard_xy_step_m},
            {"motion_guard_pose_hard_z_step_m": motion_guard_pose_hard_z_step_m},
            {"motion_guard_pose_hard_yaw_step_rad": motion_guard_pose_hard_yaw_step_rad},
        ],
    )

    trajectory_path_publisher = Node(
        package="uav_bridge",
        executable="trajectory_path_publisher_node",
        name="trajectory_path_publisher",
        output="screen",
        condition=IfCondition(publish_trajectory),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_odom_topic": trajectory_input_odom_topic},
            {"output_path_topic": trajectory_output_topic},
            {"max_samples": trajectory_max_samples},
            {"min_sample_distance_m": trajectory_min_sample_distance_m},
            {"min_sample_period_s": trajectory_min_sample_period_s},
        ],
    )

    height_measurement_bridge = Node(
        package="uav_bridge",
        executable="height_measurement_bridge_node",
        name="height_measurement_bridge_node",
        output="screen",
        parameters=[
            {"distance_sensor_topic": distance_sensor_topic},
            {"height_measurement_topic": height_measurement_topic},
            {"frame_id": height_measurement_frame_id},
            {"use_sim_time": use_sim_time},
        ],
    )

    aruco_detector = Node(
        package="uav_visual_landing",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"image_topic": image_topic},
            {"camera_info_topic": camera_info_topic},
            {"tag_size_m": tag_size_m},
        ],
    )

    visual_landing = Node(
        package="uav_visual_landing",
        executable="visual_landing_node",
        name="visual_landing_node",
        output="screen",
        parameters=[
            visual_landing_config,
            {"use_sim_time": use_sim_time},
            {"height_measurement_transport": height_measurement_transport},
            {"height_measurement_topic": height_measurement_topic},
            {"height_measurement_mode": height_measurement_mode},
            {"range_topic": distance_sensor_topic},
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
        ],
    )

    default_openvins_config = os.path.join(
        bringup_share, "config", "openvins", "orbbec_gemini336", "estimator_config.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_backend", default_value="legacy"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_fourcc", default_value="MJPG"),
        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("camera_fps", default_value="120.0"),
        DeclareLaunchArgument("camera_name", default_value="uav_mono_camera"),
        DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
        DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
        DeclareLaunchArgument("camera_frame_id", default_value="uav_camera_optical_frame"),
        DeclareLaunchArgument("camera_x", default_value="0.08"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.03"),
        DeclareLaunchArgument("camera_roll", default_value="-1.5707963267948966"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="-1.5707963267948966"),
        DeclareLaunchArgument("fx", default_value="762.7223004959626"),
        DeclareLaunchArgument("fy", default_value="762.7223004959626"),
        DeclareLaunchArgument("cx", default_value="640.0"),
        DeclareLaunchArgument("cy", default_value="360.0"),
        DeclareLaunchArgument("camera_hfov_rad", default_value="1.3962634"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("tag_size_m", default_value="0.20"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_openvins_stack", default_value="true"),
        DeclareLaunchArgument("openvins_start_rviz", default_value="false"),
        DeclareLaunchArgument("openvins_config_path", default_value=default_openvins_config),
        DeclareLaunchArgument("publish_trajectory", default_value="true"),
        DeclareLaunchArgument("trajectory_input_odom_topic", default_value="/uav/state/odometry"),
        DeclareLaunchArgument("trajectory_output_topic", default_value="/uav/state/trajectory"),
        DeclareLaunchArgument("trajectory_max_samples", default_value="5000"),
        DeclareLaunchArgument("trajectory_min_sample_distance_m", default_value="0.02"),
        DeclareLaunchArgument("trajectory_min_sample_period_s", default_value="0.10"),
        DeclareLaunchArgument("height_measurement_transport", default_value="stamped_range"),
        DeclareLaunchArgument("height_measurement_topic", default_value="/uav/sensors/downward_range"),
        DeclareLaunchArgument("height_measurement_mode", default_value=""),
        DeclareLaunchArgument(
            "height_measurement_frame_id", default_value="uav_optical_flow_range_frame"
        ),
        DeclareLaunchArgument("motion_guard_enabled", default_value="false"),
        DeclareLaunchArgument("motion_guard_soft_dwell_s", default_value="2.0"),
        DeclareLaunchArgument("motion_guard_pose_gap_reset_s", default_value="0.40"),
        DeclareLaunchArgument("motion_guard_soft_xy_mps", default_value="0.40"),
        DeclareLaunchArgument("motion_guard_soft_z_mps", default_value="0.45"),
        DeclareLaunchArgument("motion_guard_soft_yaw_radps", default_value="0.60"),
        DeclareLaunchArgument("motion_guard_hard_xy_mps", default_value="0.55"),
        DeclareLaunchArgument("motion_guard_hard_z_mps", default_value="0.55"),
        DeclareLaunchArgument("motion_guard_hard_yaw_radps", default_value="0.90"),
        DeclareLaunchArgument("motion_guard_feedback_hard_xy_mps", default_value="0.65"),
        DeclareLaunchArgument("motion_guard_feedback_hard_z_mps", default_value="0.55"),
        DeclareLaunchArgument("motion_guard_pose_soft_xy_step_m", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_pose_soft_z_step_m", default_value="0.12"),
        DeclareLaunchArgument("motion_guard_pose_soft_yaw_step_rad", default_value="0.35"),
        DeclareLaunchArgument("motion_guard_pose_hard_xy_step_m", default_value="0.50"),
        DeclareLaunchArgument("motion_guard_pose_hard_z_step_m", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_pose_hard_yaw_step_rad", default_value="0.70"),
        openvins_orbbec_launch,
        mono_camera_launch,
        uav_state_bridge,
        uav_control,
        trajectory_path_publisher,
        height_measurement_bridge,
        aruco_detector,
        visual_landing,
    ])
