import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "orbbec_depth_camera.launch.py")
        ),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
            "camera_name": LaunchConfiguration("camera_name"),
            "base_frame_id": LaunchConfiguration("base_frame_id"),
            "camera_frame_id": LaunchConfiguration("camera_frame_id"),
            "camera_x": LaunchConfiguration("camera_x"),
            "camera_y": LaunchConfiguration("camera_y"),
            "camera_z": LaunchConfiguration("camera_z"),
            "camera_roll": LaunchConfiguration("camera_roll"),
            "camera_pitch": LaunchConfiguration("camera_pitch"),
            "camera_yaw": LaunchConfiguration("camera_yaw"),
            "publish_mount_tf": LaunchConfiguration("publish_mount_tf"),
            "enable_depth": "true",
            "left_ir_width": LaunchConfiguration("left_ir_width"),
            "left_ir_height": LaunchConfiguration("left_ir_height"),
            "left_ir_fps": LaunchConfiguration("left_ir_fps"),
            "left_ir_format": LaunchConfiguration("left_ir_format"),
            "right_ir_width": LaunchConfiguration("right_ir_width"),
            "right_ir_height": LaunchConfiguration("right_ir_height"),
            "right_ir_fps": LaunchConfiguration("right_ir_fps"),
            "right_ir_format": LaunchConfiguration("right_ir_format"),
            "enable_ir_auto_exposure": LaunchConfiguration("enable_ir_auto_exposure"),
            "ir_exposure": LaunchConfiguration("ir_exposure"),
            "ir_gain": LaunchConfiguration("ir_gain"),
            "ir_ae_max_exposure": LaunchConfiguration("ir_ae_max_exposure"),
            "ir_brightness": LaunchConfiguration("ir_brightness"),
            "enable_laser": LaunchConfiguration("enable_laser"),
            "enable_ldp": LaunchConfiguration("enable_ldp"),
            "accel_rate": LaunchConfiguration("accel_rate"),
            "gyro_rate": LaunchConfiguration("gyro_rate"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument(
                "camera_frame_id", default_value="uav_stereo_camera_optical_frame"
            ),
            DeclareLaunchArgument("camera_x", default_value="0.0503"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.1043"),
            DeclareLaunchArgument("camera_roll", default_value=str(-3.141592653589793 / 2.0)),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value=str(-3.141592653589793 / 2.0)),
            DeclareLaunchArgument("publish_mount_tf", default_value="true"),
            DeclareLaunchArgument("left_ir_width", default_value="0"),
            DeclareLaunchArgument("left_ir_height", default_value="0"),
            DeclareLaunchArgument("left_ir_fps", default_value="0"),
            DeclareLaunchArgument("left_ir_format", default_value="ANY"),
            DeclareLaunchArgument("right_ir_width", default_value="0"),
            DeclareLaunchArgument("right_ir_height", default_value="0"),
            DeclareLaunchArgument("right_ir_fps", default_value="0"),
            DeclareLaunchArgument("right_ir_format", default_value="ANY"),
            DeclareLaunchArgument("enable_ir_auto_exposure", default_value="true"),
            DeclareLaunchArgument("ir_exposure", default_value="-1"),
            DeclareLaunchArgument("ir_gain", default_value="-1"),
            DeclareLaunchArgument("ir_ae_max_exposure", default_value="-1"),
            DeclareLaunchArgument("ir_brightness", default_value="-1"),
            DeclareLaunchArgument("enable_laser", default_value="true"),
            DeclareLaunchArgument("enable_ldp", default_value="true"),
            DeclareLaunchArgument("accel_rate", default_value="200hz"),
            DeclareLaunchArgument("gyro_rate", default_value="200hz"),
            DeclareLaunchArgument("log_level", default_value="none"),
            depth_camera_launch,
        ]
    )
