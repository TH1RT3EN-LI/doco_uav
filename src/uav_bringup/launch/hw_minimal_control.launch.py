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
    DEFAULT_STEREO_CAMERA_TO_BODY,
)


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    fmu_namespace = LaunchConfiguration("fmu_namespace")
    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    max_velocity_setpoint_mps = LaunchConfiguration("max_velocity_setpoint_mps")
    max_acceleration_setpoint_mps2 = LaunchConfiguration("max_acceleration_setpoint_mps2")
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
    start_orbbec_depth_camera = LaunchConfiguration("start_orbbec_depth_camera")
    orbbec_camera_launch_file = LaunchConfiguration("orbbec_camera_launch_file")
    orbbec_camera_name = LaunchConfiguration("orbbec_camera_name")
    orbbec_camera_serial_number = LaunchConfiguration("orbbec_camera_serial_number")
    orbbec_camera_usb_port = LaunchConfiguration("orbbec_camera_usb_port")
    orbbec_camera_frame_id = LaunchConfiguration("orbbec_camera_frame_id")
    orbbec_camera_x = LaunchConfiguration("orbbec_camera_x")
    orbbec_camera_y = LaunchConfiguration("orbbec_camera_y")
    orbbec_camera_z = LaunchConfiguration("orbbec_camera_z")
    orbbec_camera_roll = LaunchConfiguration("orbbec_camera_roll")
    orbbec_camera_pitch = LaunchConfiguration("orbbec_camera_pitch")
    orbbec_camera_yaw = LaunchConfiguration("orbbec_camera_yaw")

    default_recording_path = str(
        Path.home() / "uav_recordings" / f"mono_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    )

    vehicle_local_position_topic = [fmu_namespace, "/out/vehicle_local_position"]
    vehicle_odometry_topic = [fmu_namespace, "/out/vehicle_odometry"]
    vehicle_status_topic = [fmu_namespace, "/out/vehicle_status"]
    offboard_mode_topic = [fmu_namespace, "/in/offboard_control_mode"]
    trajectory_setpoint_topic = [fmu_namespace, "/in/trajectory_setpoint"]
    vehicle_command_topic = [fmu_namespace, "/in/vehicle_command"]

    hw_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "hw_mono_camera.launch.py")),
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

    hw_orbbec_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "hw_orbbec_depth_camera.launch.py")),
        condition=IfCondition(start_orbbec_depth_camera),
        launch_arguments={
            "start_camera": start_orbbec_depth_camera,
            "camera_launch_file": orbbec_camera_launch_file,
            "camera_name": orbbec_camera_name,
            "serial_number": orbbec_camera_serial_number,
            "usb_port": orbbec_camera_usb_port,
            "base_frame_id": base_frame_id,
            "camera_frame_id": orbbec_camera_frame_id,
            "camera_x": orbbec_camera_x,
            "camera_y": orbbec_camera_y,
            "camera_z": orbbec_camera_z,
            "camera_roll": orbbec_camera_roll,
            "camera_pitch": orbbec_camera_pitch,
            "camera_yaw": orbbec_camera_yaw,
        }.items(),
    )

    fmu_topic_namespace_bridge = Node(
        package="uav_bridge",
        executable="fmu_topic_namespace_bridge_node",
        name="fmu_topic_namespace_bridge",
        output="screen",
        parameters=[
            {"namespaced_fmu_prefix": fmu_namespace},
            {"global_fmu_prefix": "/fmu"},
        ],
    )

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge",
        output="screen",
        parameters=[
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_odometry_topic": vehicle_odometry_topic},
            {"output_odometry_topic": "/uav/state/odometry"},
            {"map_frame_id": "uav_map"},
            {"odom_frame_id": "uav_odom"},
            {"base_frame_id": base_frame_id},
            {"publish_rate_hz": 50.0},
            {"publish_odometry": True},
            {"publish_tf": True},
            {"publish_map_to_odom_tf": True},
        ],
    )

    uav_control = Node(
        package="uav_bridge",
        executable="uav_control_node",
        name="uav_control",
        output="screen",
        parameters=[
            {"state_topic": "/uav/state/odometry"},
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
            {"takeoff_service": "/uav/control/command/takeoff"},
            {"hold_service": "/uav/control/command/hold"},
            {"position_mode_service": "/uav/control/command/position_mode"},
            {"land_service": "/uav/control/command/land"},
            {"abort_service": "/uav/control/command/abort"},
            {"disarm_service": "/uav/control/command/disarm"},
            {"takeoff_height_m": takeoff_height_m},
            {"max_velocity_setpoint_mps": max_velocity_setpoint_mps},
            {"max_acceleration_setpoint_mps2": max_acceleration_setpoint_mps2},
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
        DeclareLaunchArgument("fmu_namespace", default_value="/fmu"),
        DeclareLaunchArgument("takeoff_height_m", default_value="0.25"),
        DeclareLaunchArgument("max_velocity_setpoint_mps", default_value="0.40"),
        DeclareLaunchArgument("max_acceleration_setpoint_mps2", default_value="0.60"),
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
        DeclareLaunchArgument("orbbec_camera_launch_file", default_value="gemini_330_series.launch.py"),
        DeclareLaunchArgument("orbbec_camera_name", default_value="uav/depth_camera"),
        DeclareLaunchArgument("orbbec_camera_serial_number", default_value=""),
        DeclareLaunchArgument("orbbec_camera_usb_port", default_value=""),
        DeclareLaunchArgument("orbbec_camera_frame_id", default_value="uav_stereo_camera_optical_frame"),
        DeclareLaunchArgument("orbbec_camera_x", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["x"]),
        DeclareLaunchArgument("orbbec_camera_y", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["y"]),
        DeclareLaunchArgument("orbbec_camera_z", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["z"]),
        DeclareLaunchArgument("orbbec_camera_roll", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("orbbec_camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("orbbec_camera_yaw", default_value=str(-math.pi / 2.0)),
        hw_camera_launch,
        hw_orbbec_depth_camera_launch,
        fmu_topic_namespace_bridge,
        uav_state_bridge,
        uav_control,
        mono_video_recorder,
    ])
