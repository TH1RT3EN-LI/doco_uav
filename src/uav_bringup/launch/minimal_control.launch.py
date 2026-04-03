import math
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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

    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    max_velocity_setpoint_mps = LaunchConfiguration("max_velocity_setpoint_mps")
    max_acceleration_setpoint_mps2 = LaunchConfiguration("max_acceleration_setpoint_mps2")
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

    default_recording_path = str(
        Path.home() / "uav_recordings" / f"mono_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    )

    vehicle_local_position_topic = "/fmu/out/vehicle_local_position"
    vehicle_odometry_topic = "/fmu/out/vehicle_odometry"
    vehicle_status_topic = "/fmu/out/vehicle_status"
    offboard_mode_topic = "/fmu/in/offboard_control_mode"
    trajectory_setpoint_topic = "/fmu/in/trajectory_setpoint"
    vehicle_command_topic = "/fmu/in/vehicle_command"

    mono_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "mono_camera.launch.py")),
        condition=IfCondition(start_mono_camera),
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

    orbbec_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "orbbec_depth_camera.launch.py")),
        condition=IfCondition(start_orbbec_depth_camera),
        launch_arguments={
            "start_camera": start_orbbec_depth_camera,
            "camera_name": orbbec_camera_name,
            "base_frame_id": base_frame_id,
            "camera_frame_id": orbbec_camera_frame_id,
            "camera_x": orbbec_camera_x,
            "camera_y": orbbec_camera_y,
            "camera_z": orbbec_camera_z,
            "camera_roll": orbbec_camera_roll,
            "camera_pitch": orbbec_camera_pitch,
            "camera_yaw": orbbec_camera_yaw,
            "enable_depth": orbbec_enable_depth,
            "enable_color": orbbec_enable_color,
            "enable_left_ir": orbbec_enable_left_ir,
            "enable_right_ir": orbbec_enable_right_ir,
            "enable_point_cloud": orbbec_enable_point_cloud,
            "enable_colored_point_cloud": orbbec_enable_colored_point_cloud,
            "enable_sync_output_accel_gyro": orbbec_enable_sync_output_accel_gyro,
            "enable_publish_extrinsic": orbbec_enable_publish_extrinsic,
            "enable_accel": orbbec_enable_accel,
            "enable_gyro": orbbec_enable_gyro,
            "accel_rate": orbbec_accel_rate,
            "gyro_rate": orbbec_gyro_rate,
            "left_ir_width": orbbec_left_ir_width,
            "left_ir_height": orbbec_left_ir_height,
            "left_ir_fps": orbbec_left_ir_fps,
            "left_ir_format": orbbec_left_ir_format,
            "right_ir_width": orbbec_right_ir_width,
            "right_ir_height": orbbec_right_ir_height,
            "right_ir_fps": orbbec_right_ir_fps,
            "right_ir_format": orbbec_right_ir_format,
            "enable_ir_auto_exposure": orbbec_enable_ir_auto_exposure,
            "ir_exposure": orbbec_ir_exposure,
            "ir_gain": orbbec_ir_gain,
            "ir_ae_max_exposure": orbbec_ir_ae_max_exposure,
            "ir_brightness": orbbec_ir_brightness,
            "enable_laser": orbbec_enable_laser,
            "enable_ldp": orbbec_enable_ldp,
        }.items(),
    )

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge",
        output="screen",
        parameters=[
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_odometry_topic": vehicle_odometry_topic},
            {"base_frame_id": base_frame_id},
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
            {"takeoff_height_m": takeoff_height_m},
            {"max_velocity_setpoint_mps": max_velocity_setpoint_mps},
            {"max_acceleration_setpoint_mps2": max_acceleration_setpoint_mps2},
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
            {"input_odom_topic": trajectory_input_odom_topic},
            {"output_path_topic": trajectory_output_topic},
            {"max_samples": trajectory_max_samples},
            {"min_sample_distance_m": trajectory_min_sample_distance_m},
            {"min_sample_period_s": trajectory_min_sample_period_s},
        ],
    )

    mono_video_recorder = Node(
        package="uav_bridge",
        executable="mono_video_recorder_node",
        name="mono_video_recorder",
        output="screen",
        condition=IfCondition(record_mono_video),
        parameters=[
            {"image_topic": mono_video_topic},
            {"output_path": mono_video_output_path},
            {"fps": mono_video_fps},
            {"fourcc": mono_video_fourcc},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("takeoff_height_m", default_value="0.25"),
        DeclareLaunchArgument("max_velocity_setpoint_mps", default_value="0.40"),
        DeclareLaunchArgument("max_acceleration_setpoint_mps2", default_value="0.60"),
        DeclareLaunchArgument("motion_guard_enabled", default_value="true"),
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
        DeclareLaunchArgument("start_orbbec_depth_camera", default_value="false"),
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
        DeclareLaunchArgument("record_mono_video", default_value="true"),
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
        mono_camera_launch,
        orbbec_depth_camera_launch,
        uav_state_bridge,
        uav_control,
        trajectory_path_publisher,
        mono_video_recorder,
    ])
