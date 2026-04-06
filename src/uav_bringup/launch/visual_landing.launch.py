import math
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_bringup.profile_defaults import (
    DEFAULT_CAMERA_INTRINSICS,
    DEFAULT_CAMERA_TO_BODY,
    DEFAULT_ORBBEC_IR_EXPOSURE,
    DEFAULT_ORBBEC_IR_STREAM,
    DEFAULT_ORBBEC_STANDALONE_PROFILE,
    DEFAULT_STEREO_CAMERA_TO_BODY,
)


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    visual_landing_share = get_package_share_directory("uav_visual_landing")

    use_sim_time = LaunchConfiguration("use_sim_time")
    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    max_velocity_setpoint_mps = LaunchConfiguration("max_velocity_setpoint_mps")
    max_acceleration_setpoint_mps2 = LaunchConfiguration("max_acceleration_setpoint_mps2")
    control_state_topic = LaunchConfiguration("control_state_topic")
    execution_state_topic = LaunchConfiguration("execution_state_topic")
    position_command_frame_id = LaunchConfiguration("position_command_frame_id")
    ov_hold_kp_xy = LaunchConfiguration("ov_hold_kp_xy")
    ov_hold_kp_z = LaunchConfiguration("ov_hold_kp_z")
    ov_hold_kp_yaw = LaunchConfiguration("ov_hold_kp_yaw")
    ov_hold_max_vxy = LaunchConfiguration("ov_hold_max_vxy")
    ov_hold_max_vz = LaunchConfiguration("ov_hold_max_vz")
    ov_hold_max_yaw_rate = LaunchConfiguration("ov_hold_max_yaw_rate")
    ov_target_xy_tolerance_m = LaunchConfiguration("ov_target_xy_tolerance_m")
    ov_target_z_tolerance_m = LaunchConfiguration("ov_target_z_tolerance_m")
    ov_target_yaw_tolerance_rad = LaunchConfiguration("ov_target_yaw_tolerance_rad")
    ov_fault_pose_timeout_s = LaunchConfiguration("ov_fault_pose_timeout_s")
    ov_fault_max_xy_step_m = LaunchConfiguration("ov_fault_max_xy_step_m")
    ov_fault_max_z_step_m = LaunchConfiguration("ov_fault_max_z_step_m")
    ov_fault_max_yaw_step_rad = LaunchConfiguration("ov_fault_max_yaw_step_rad")
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

    start_mono_camera = LaunchConfiguration("start_mono_camera")
    camera_backend = LaunchConfiguration("camera_backend")
    camera_device = LaunchConfiguration("camera_device")
    camera_fourcc = LaunchConfiguration("camera_fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_fps = LaunchConfiguration("camera_fps")
    camera_name = LaunchConfiguration("camera_name")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
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

    record_mono_video = LaunchConfiguration("record_mono_video")
    mono_video_topic = LaunchConfiguration("mono_video_topic")
    mono_video_output_path = LaunchConfiguration("mono_video_output_path")
    mono_video_fps = LaunchConfiguration("mono_video_fps")
    mono_video_fourcc = LaunchConfiguration("mono_video_fourcc")

    publish_trajectory = LaunchConfiguration("publish_trajectory")
    trajectory_input_odom_topic = LaunchConfiguration("trajectory_input_odom_topic")
    trajectory_output_topic = LaunchConfiguration("trajectory_output_topic")
    trajectory_max_samples = LaunchConfiguration("trajectory_max_samples")
    trajectory_min_sample_distance_m = LaunchConfiguration("trajectory_min_sample_distance_m")
    trajectory_min_sample_period_s = LaunchConfiguration("trajectory_min_sample_period_s")
    start_rc_safety_mux = LaunchConfiguration("start_rc_safety_mux")
    rc_safety_config_path = LaunchConfiguration("rc_safety_config_path")

    start_orbbec_depth_camera = LaunchConfiguration("start_orbbec_depth_camera")
    orbbec_camera_name = LaunchConfiguration("orbbec_camera_name")
    orbbec_camera_frame_id = LaunchConfiguration("orbbec_camera_frame_id")
    orbbec_camera_x = LaunchConfiguration("orbbec_camera_x")
    orbbec_camera_y = LaunchConfiguration("orbbec_camera_y")
    orbbec_camera_z = LaunchConfiguration("orbbec_camera_z")
    orbbec_camera_roll = LaunchConfiguration("orbbec_camera_roll")
    orbbec_camera_pitch = LaunchConfiguration("orbbec_camera_pitch")
    orbbec_camera_yaw = LaunchConfiguration("orbbec_camera_yaw")
    orbbec_enable_depth = LaunchConfiguration("orbbec_enable_depth")
    orbbec_enable_color = LaunchConfiguration("orbbec_enable_color")
    orbbec_enable_left_ir = LaunchConfiguration("orbbec_enable_left_ir")
    orbbec_enable_right_ir = LaunchConfiguration("orbbec_enable_right_ir")
    orbbec_enable_point_cloud = LaunchConfiguration("orbbec_enable_point_cloud")
    orbbec_enable_colored_point_cloud = LaunchConfiguration("orbbec_enable_colored_point_cloud")
    orbbec_enable_sync_output_accel_gyro = LaunchConfiguration("orbbec_enable_sync_output_accel_gyro")
    orbbec_enable_publish_extrinsic = LaunchConfiguration("orbbec_enable_publish_extrinsic")
    orbbec_enable_accel = LaunchConfiguration("orbbec_enable_accel")
    orbbec_enable_gyro = LaunchConfiguration("orbbec_enable_gyro")
    orbbec_accel_rate = LaunchConfiguration("orbbec_accel_rate")
    orbbec_gyro_rate = LaunchConfiguration("orbbec_gyro_rate")
    orbbec_left_ir_width = LaunchConfiguration("orbbec_left_ir_width")
    orbbec_left_ir_height = LaunchConfiguration("orbbec_left_ir_height")
    orbbec_left_ir_fps = LaunchConfiguration("orbbec_left_ir_fps")
    orbbec_left_ir_format = LaunchConfiguration("orbbec_left_ir_format")
    orbbec_right_ir_width = LaunchConfiguration("orbbec_right_ir_width")
    orbbec_right_ir_height = LaunchConfiguration("orbbec_right_ir_height")
    orbbec_right_ir_fps = LaunchConfiguration("orbbec_right_ir_fps")
    orbbec_right_ir_format = LaunchConfiguration("orbbec_right_ir_format")
    orbbec_enable_ir_auto_exposure = LaunchConfiguration("orbbec_enable_ir_auto_exposure")
    orbbec_ir_exposure = LaunchConfiguration("orbbec_ir_exposure")
    orbbec_ir_gain = LaunchConfiguration("orbbec_ir_gain")
    orbbec_ir_ae_max_exposure = LaunchConfiguration("orbbec_ir_ae_max_exposure")
    orbbec_ir_brightness = LaunchConfiguration("orbbec_ir_brightness")
    orbbec_enable_laser = LaunchConfiguration("orbbec_enable_laser")
    orbbec_enable_ldp = LaunchConfiguration("orbbec_enable_ldp")

    search_height_m = LaunchConfiguration("search_height_m")
    terminal_entry_height_m = LaunchConfiguration("terminal_entry_height_m")
    max_vxy = LaunchConfiguration("max_vxy")
    align_enter_lateral_m = LaunchConfiguration("align_enter_lateral_m")
    align_exit_lateral_m = LaunchConfiguration("align_exit_lateral_m")
    tag_size_m = LaunchConfiguration("tag_size_m")
    height_measurement_transport = LaunchConfiguration("height_measurement_transport")
    height_measurement_topic = LaunchConfiguration("height_measurement_topic")
    height_measurement_mode = LaunchConfiguration("height_measurement_mode")
    height_measurement_frame_id = LaunchConfiguration("height_measurement_frame_id")

    default_recording_path = str(
        Path.home() / "uav_recordings" / f"mono_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    )

    vehicle_local_position_topic = "/fmu/out/vehicle_local_position"
    vehicle_status_topic = "/fmu/out/vehicle_status"
    distance_sensor_topic = "/fmu/out/distance_sensor"

    minimal_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "minimal_control.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "takeoff_height_m": takeoff_height_m,
            "max_velocity_setpoint_mps": max_velocity_setpoint_mps,
            "max_acceleration_setpoint_mps2": max_acceleration_setpoint_mps2,
            "control_state_topic": control_state_topic,
            "execution_state_topic": execution_state_topic,
            "position_command_frame_id": position_command_frame_id,
            "ov_hold_kp_xy": ov_hold_kp_xy,
            "ov_hold_kp_z": ov_hold_kp_z,
            "ov_hold_kp_yaw": ov_hold_kp_yaw,
            "ov_hold_max_vxy": ov_hold_max_vxy,
            "ov_hold_max_vz": ov_hold_max_vz,
            "ov_hold_max_yaw_rate": ov_hold_max_yaw_rate,
            "ov_target_xy_tolerance_m": ov_target_xy_tolerance_m,
            "ov_target_z_tolerance_m": ov_target_z_tolerance_m,
            "ov_target_yaw_tolerance_rad": ov_target_yaw_tolerance_rad,
            "ov_fault_pose_timeout_s": ov_fault_pose_timeout_s,
            "ov_fault_max_xy_step_m": ov_fault_max_xy_step_m,
            "ov_fault_max_z_step_m": ov_fault_max_z_step_m,
            "ov_fault_max_yaw_step_rad": ov_fault_max_yaw_step_rad,
            "motion_guard_enabled": motion_guard_enabled,
            "motion_guard_soft_dwell_s": motion_guard_soft_dwell_s,
            "motion_guard_pose_gap_reset_s": motion_guard_pose_gap_reset_s,
            "motion_guard_soft_xy_mps": motion_guard_soft_xy_mps,
            "motion_guard_soft_z_mps": motion_guard_soft_z_mps,
            "motion_guard_soft_yaw_radps": motion_guard_soft_yaw_radps,
            "motion_guard_hard_xy_mps": motion_guard_hard_xy_mps,
            "motion_guard_hard_z_mps": motion_guard_hard_z_mps,
            "motion_guard_hard_yaw_radps": motion_guard_hard_yaw_radps,
            "motion_guard_feedback_hard_xy_mps": motion_guard_feedback_hard_xy_mps,
            "motion_guard_feedback_hard_z_mps": motion_guard_feedback_hard_z_mps,
            "motion_guard_pose_soft_xy_step_m": motion_guard_pose_soft_xy_step_m,
            "motion_guard_pose_soft_z_step_m": motion_guard_pose_soft_z_step_m,
            "motion_guard_pose_soft_yaw_step_rad": motion_guard_pose_soft_yaw_step_rad,
            "motion_guard_pose_hard_xy_step_m": motion_guard_pose_hard_xy_step_m,
            "motion_guard_pose_hard_z_step_m": motion_guard_pose_hard_z_step_m,
            "motion_guard_pose_hard_yaw_step_rad": motion_guard_pose_hard_yaw_step_rad,
            "start_mono_camera": start_mono_camera,
            "camera_backend": camera_backend,
            "camera_device": camera_device,
            "camera_fourcc": camera_fourcc,
            "image_width": image_width,
            "image_height": image_height,
            "camera_fps": camera_fps,
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
            "record_mono_video": record_mono_video,
            "mono_video_topic": mono_video_topic,
            "mono_video_output_path": mono_video_output_path,
            "mono_video_fps": mono_video_fps,
            "mono_video_fourcc": mono_video_fourcc,
            "publish_trajectory": publish_trajectory,
            "trajectory_input_odom_topic": trajectory_input_odom_topic,
            "trajectory_output_topic": trajectory_output_topic,
            "trajectory_max_samples": trajectory_max_samples,
            "trajectory_min_sample_distance_m": trajectory_min_sample_distance_m,
            "trajectory_min_sample_period_s": trajectory_min_sample_period_s,
            "start_rc_safety_mux": start_rc_safety_mux,
            "rc_safety_config_path": rc_safety_config_path,
            "start_orbbec_depth_camera": start_orbbec_depth_camera,
            "orbbec_camera_name": orbbec_camera_name,
            "orbbec_camera_frame_id": orbbec_camera_frame_id,
            "orbbec_camera_x": orbbec_camera_x,
            "orbbec_camera_y": orbbec_camera_y,
            "orbbec_camera_z": orbbec_camera_z,
            "orbbec_camera_roll": orbbec_camera_roll,
            "orbbec_camera_pitch": orbbec_camera_pitch,
            "orbbec_camera_yaw": orbbec_camera_yaw,
            "orbbec_enable_depth": orbbec_enable_depth,
            "orbbec_enable_color": orbbec_enable_color,
            "orbbec_enable_left_ir": orbbec_enable_left_ir,
            "orbbec_enable_right_ir": orbbec_enable_right_ir,
            "orbbec_enable_point_cloud": orbbec_enable_point_cloud,
            "orbbec_enable_colored_point_cloud": orbbec_enable_colored_point_cloud,
            "orbbec_enable_sync_output_accel_gyro": orbbec_enable_sync_output_accel_gyro,
            "orbbec_enable_publish_extrinsic": orbbec_enable_publish_extrinsic,
            "orbbec_enable_accel": orbbec_enable_accel,
            "orbbec_enable_gyro": orbbec_enable_gyro,
            "orbbec_accel_rate": orbbec_accel_rate,
            "orbbec_gyro_rate": orbbec_gyro_rate,
            "orbbec_left_ir_width": orbbec_left_ir_width,
            "orbbec_left_ir_height": orbbec_left_ir_height,
            "orbbec_left_ir_fps": orbbec_left_ir_fps,
            "orbbec_left_ir_format": orbbec_left_ir_format,
            "orbbec_right_ir_width": orbbec_right_ir_width,
            "orbbec_right_ir_height": orbbec_right_ir_height,
            "orbbec_right_ir_fps": orbbec_right_ir_fps,
            "orbbec_right_ir_format": orbbec_right_ir_format,
            "orbbec_enable_ir_auto_exposure": orbbec_enable_ir_auto_exposure,
            "orbbec_ir_exposure": orbbec_ir_exposure,
            "orbbec_ir_gain": orbbec_ir_gain,
            "orbbec_ir_ae_max_exposure": orbbec_ir_ae_max_exposure,
            "orbbec_ir_brightness": orbbec_ir_brightness,
            "orbbec_enable_laser": orbbec_enable_laser,
            "orbbec_enable_ldp": orbbec_enable_ldp,
        }.items(),
    )

    visual_landing_config = os.path.join(
        visual_landing_share, "config", "visual_landing_stable.yaml"
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
            {"state_topic": execution_state_topic},
            {"search_height_m": search_height_m},
            {"terminal_entry_height_m": terminal_entry_height_m},
            {"max_vxy": max_vxy},
            {"align_enter_lateral_m": align_enter_lateral_m},
            {"align_exit_lateral_m": align_exit_lateral_m},
            {"height_measurement_transport": height_measurement_transport},
            {"height_measurement_topic": height_measurement_topic},
            {"height_measurement_mode": height_measurement_mode},
            {"range_topic": distance_sensor_topic},
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
        ],
    )

    default_rc_safety_config = os.path.join(
        bringup_share, "config", "safety", "rc_safety_mux.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("takeoff_height_m", default_value="0.70"),
        DeclareLaunchArgument("max_velocity_setpoint_mps", default_value="0.25"),
        DeclareLaunchArgument("max_acceleration_setpoint_mps2", default_value="0.35"),
        DeclareLaunchArgument("control_state_topic", default_value="/uav/state/odometry"),
        DeclareLaunchArgument("execution_state_topic", default_value="/uav/state/odometry_px4"),
        DeclareLaunchArgument("position_command_frame_id", default_value="global"),
        DeclareLaunchArgument("ov_hold_kp_xy", default_value="0.80"),
        DeclareLaunchArgument("ov_hold_kp_z", default_value="0.80"),
        DeclareLaunchArgument("ov_hold_kp_yaw", default_value="1.00"),
        DeclareLaunchArgument("ov_hold_max_vxy", default_value="0.20"),
        DeclareLaunchArgument("ov_hold_max_vz", default_value="0.10"),
        DeclareLaunchArgument("ov_hold_max_yaw_rate", default_value="0.20"),
        DeclareLaunchArgument("ov_target_xy_tolerance_m", default_value="0.05"),
        DeclareLaunchArgument("ov_target_z_tolerance_m", default_value="0.05"),
        DeclareLaunchArgument("ov_target_yaw_tolerance_rad", default_value="0.08"),
        DeclareLaunchArgument("ov_fault_pose_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("ov_fault_max_xy_step_m", default_value="0.30"),
        DeclareLaunchArgument("ov_fault_max_z_step_m", default_value="0.20"),
        DeclareLaunchArgument("ov_fault_max_yaw_step_rad", default_value="0.35"),
        DeclareLaunchArgument("motion_guard_enabled", default_value="false"),
        DeclareLaunchArgument("motion_guard_soft_dwell_s", default_value="2.0"),
        DeclareLaunchArgument("motion_guard_pose_gap_reset_s", default_value="0.40"),
        DeclareLaunchArgument("motion_guard_soft_xy_mps", default_value="0.40"),
        DeclareLaunchArgument("motion_guard_soft_z_mps", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_soft_yaw_radps", default_value="0.60"),
        DeclareLaunchArgument("motion_guard_hard_xy_mps", default_value="0.55"),
        DeclareLaunchArgument("motion_guard_hard_z_mps", default_value="0.35"),
        DeclareLaunchArgument("motion_guard_hard_yaw_radps", default_value="0.90"),
        DeclareLaunchArgument("motion_guard_feedback_hard_xy_mps", default_value="0.65"),
        DeclareLaunchArgument("motion_guard_feedback_hard_z_mps", default_value="0.45"),
        DeclareLaunchArgument("motion_guard_pose_soft_xy_step_m", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_pose_soft_z_step_m", default_value="0.12"),
        DeclareLaunchArgument("motion_guard_pose_soft_yaw_step_rad", default_value="0.35"),
        DeclareLaunchArgument("motion_guard_pose_hard_xy_step_m", default_value="0.50"),
        DeclareLaunchArgument("motion_guard_pose_hard_z_step_m", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_pose_hard_yaw_step_rad", default_value="0.70"),
        DeclareLaunchArgument("start_mono_camera", default_value="true"),
        DeclareLaunchArgument("camera_backend", default_value="legacy"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video1"),
        DeclareLaunchArgument("camera_fourcc", default_value="MJPG"),
        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("camera_fps", default_value="120.0"),
        DeclareLaunchArgument("camera_name", default_value="uav_mono_camera"),
        DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
        DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
        DeclareLaunchArgument("camera_frame_id", default_value="uav_camera_optical_frame"),
        DeclareLaunchArgument("camera_x", default_value=DEFAULT_CAMERA_TO_BODY["x"]),
        DeclareLaunchArgument("camera_y", default_value=DEFAULT_CAMERA_TO_BODY["y"]),
        DeclareLaunchArgument("camera_z", default_value=DEFAULT_CAMERA_TO_BODY["z"]),
        DeclareLaunchArgument("camera_roll", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("fx", default_value=DEFAULT_CAMERA_INTRINSICS["fx"]),
        DeclareLaunchArgument("fy", default_value=DEFAULT_CAMERA_INTRINSICS["fy"]),
        DeclareLaunchArgument("cx", default_value=DEFAULT_CAMERA_INTRINSICS["cx"]),
        DeclareLaunchArgument("cy", default_value=DEFAULT_CAMERA_INTRINSICS["cy"]),
        DeclareLaunchArgument("camera_hfov_rad", default_value="1.3962634"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("record_mono_video", default_value="false"),
        DeclareLaunchArgument("mono_video_topic", default_value=image_topic),
        DeclareLaunchArgument("mono_video_output_path", default_value=default_recording_path),
        DeclareLaunchArgument("mono_video_fps", default_value="120.0"),
        DeclareLaunchArgument("mono_video_fourcc", default_value="MJPG"),
        DeclareLaunchArgument("publish_trajectory", default_value="true"),
        DeclareLaunchArgument("trajectory_input_odom_topic", default_value="/uav/state/odometry"),
        DeclareLaunchArgument("trajectory_output_topic", default_value="/uav/state/trajectory"),
        DeclareLaunchArgument("trajectory_max_samples", default_value="5000"),
        DeclareLaunchArgument("trajectory_min_sample_distance_m", default_value="0.02"),
        DeclareLaunchArgument("trajectory_min_sample_period_s", default_value="0.10"),
        DeclareLaunchArgument("start_rc_safety_mux", default_value="true"),
        DeclareLaunchArgument("rc_safety_config_path", default_value=default_rc_safety_config),
        DeclareLaunchArgument("start_orbbec_depth_camera", default_value="false"),
        DeclareLaunchArgument("orbbec_camera_name", default_value="uav_depth_camera"),
        DeclareLaunchArgument("orbbec_camera_frame_id", default_value="uav_stereo_camera_optical_frame"),
        DeclareLaunchArgument("orbbec_camera_x", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["x"]),
        DeclareLaunchArgument("orbbec_camera_y", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["y"]),
        DeclareLaunchArgument("orbbec_camera_z", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["z"]),
        DeclareLaunchArgument("orbbec_camera_roll", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("orbbec_camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("orbbec_camera_yaw", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("orbbec_enable_depth", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_depth"]),
        DeclareLaunchArgument("orbbec_enable_color", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_color"]),
        DeclareLaunchArgument("orbbec_enable_left_ir", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_left_ir"]),
        DeclareLaunchArgument("orbbec_enable_right_ir", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_right_ir"]),
        DeclareLaunchArgument("orbbec_enable_point_cloud", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_point_cloud"]),
        DeclareLaunchArgument("orbbec_enable_colored_point_cloud", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_colored_point_cloud"]),
        DeclareLaunchArgument("orbbec_enable_sync_output_accel_gyro", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_sync_output_accel_gyro"]),
        DeclareLaunchArgument("orbbec_enable_publish_extrinsic", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_publish_extrinsic"]),
        DeclareLaunchArgument("orbbec_enable_accel", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_accel"]),
        DeclareLaunchArgument("orbbec_enable_gyro", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_gyro"]),
        DeclareLaunchArgument("orbbec_accel_rate", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["accel_rate"]),
        DeclareLaunchArgument("orbbec_gyro_rate", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["gyro_rate"]),
        DeclareLaunchArgument("orbbec_left_ir_width", default_value=DEFAULT_ORBBEC_IR_STREAM["width"]),
        DeclareLaunchArgument("orbbec_left_ir_height", default_value=DEFAULT_ORBBEC_IR_STREAM["height"]),
        DeclareLaunchArgument("orbbec_left_ir_fps", default_value=DEFAULT_ORBBEC_IR_STREAM["fps"]),
        DeclareLaunchArgument("orbbec_left_ir_format", default_value=DEFAULT_ORBBEC_IR_STREAM["format"]),
        DeclareLaunchArgument("orbbec_right_ir_width", default_value=DEFAULT_ORBBEC_IR_STREAM["width"]),
        DeclareLaunchArgument("orbbec_right_ir_height", default_value=DEFAULT_ORBBEC_IR_STREAM["height"]),
        DeclareLaunchArgument("orbbec_right_ir_fps", default_value=DEFAULT_ORBBEC_IR_STREAM["fps"]),
        DeclareLaunchArgument("orbbec_right_ir_format", default_value=DEFAULT_ORBBEC_IR_STREAM["format"]),
        DeclareLaunchArgument("orbbec_enable_ir_auto_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["enable_auto_exposure"]),
        DeclareLaunchArgument("orbbec_ir_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["exposure"]),
        DeclareLaunchArgument("orbbec_ir_gain", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["gain"]),
        DeclareLaunchArgument("orbbec_ir_ae_max_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["ae_max_exposure"]),
        DeclareLaunchArgument("orbbec_ir_brightness", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["brightness"]),
        DeclareLaunchArgument("orbbec_enable_laser", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_laser"]),
        DeclareLaunchArgument("orbbec_enable_ldp", default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_ldp"]),
        DeclareLaunchArgument("search_height_m", default_value="0.60"),
        DeclareLaunchArgument("terminal_entry_height_m", default_value="0.40"),
        DeclareLaunchArgument("max_vxy", default_value="0.10"),
        DeclareLaunchArgument("align_enter_lateral_m", default_value="0.08"),
        DeclareLaunchArgument("align_exit_lateral_m", default_value="0.05"),
        DeclareLaunchArgument("tag_size_m", default_value="0.20"),
        DeclareLaunchArgument("height_measurement_transport", default_value="stamped_range"),
        DeclareLaunchArgument("height_measurement_topic", default_value="/uav/sensors/downward_range"),
        DeclareLaunchArgument("height_measurement_mode", default_value=""),
        DeclareLaunchArgument(
            "height_measurement_frame_id", default_value="uav_optical_flow_range_frame"
        ),
        minimal_control_launch,
        height_measurement_bridge,
        aruco_detector,
        visual_landing,
    ])
