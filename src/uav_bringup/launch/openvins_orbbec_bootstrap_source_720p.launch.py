import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from uav_bringup.profile_defaults import (
    DEFAULT_ORBBEC_BOOTSTRAP_PROFILE,
    DEFAULT_ORBBEC_IR_EXPOSURE,
    DEFAULT_ORBBEC_IR_STREAM_720P,
)


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    wrapped_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share, "launch", "openvins_orbbec_bootstrap_source.launch.py"
            )
        ),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
            "camera_name": LaunchConfiguration("camera_name"),
            "accel_rate": LaunchConfiguration("accel_rate"),
            "gyro_rate": LaunchConfiguration("gyro_rate"),
            "left_ir_width": DEFAULT_ORBBEC_IR_STREAM_720P["width"],
            "left_ir_height": DEFAULT_ORBBEC_IR_STREAM_720P["height"],
            "left_ir_fps": DEFAULT_ORBBEC_IR_STREAM_720P["fps"],
            "left_ir_format": DEFAULT_ORBBEC_IR_STREAM_720P["format"],
            "right_ir_width": DEFAULT_ORBBEC_IR_STREAM_720P["width"],
            "right_ir_height": DEFAULT_ORBBEC_IR_STREAM_720P["height"],
            "right_ir_fps": DEFAULT_ORBBEC_IR_STREAM_720P["fps"],
            "right_ir_format": DEFAULT_ORBBEC_IR_STREAM_720P["format"],
            "enable_ir_auto_exposure": LaunchConfiguration("enable_ir_auto_exposure"),
            "ir_exposure": LaunchConfiguration("ir_exposure"),
            "ir_gain": LaunchConfiguration("ir_gain"),
            "ir_ae_max_exposure": LaunchConfiguration("ir_ae_max_exposure"),
            "ir_brightness": LaunchConfiguration("ir_brightness"),
            "enable_laser": LaunchConfiguration("enable_laser"),
            "enable_ldp": LaunchConfiguration("enable_ldp"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument(
                "accel_rate", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["accel_rate"]
            ),
            DeclareLaunchArgument(
                "gyro_rate", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["gyro_rate"]
            ),
            DeclareLaunchArgument(
                "enable_ir_auto_exposure",
                default_value=DEFAULT_ORBBEC_IR_EXPOSURE["enable_auto_exposure"],
            ),
            DeclareLaunchArgument(
                "ir_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["exposure"]
            ),
            DeclareLaunchArgument(
                "ir_gain", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["gain"]
            ),
            DeclareLaunchArgument(
                "ir_ae_max_exposure",
                default_value=DEFAULT_ORBBEC_IR_EXPOSURE["ae_max_exposure"],
            ),
            DeclareLaunchArgument(
                "ir_brightness", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["brightness"]
            ),
            DeclareLaunchArgument(
                "enable_laser", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["enable_laser"]
            ),
            DeclareLaunchArgument(
                "enable_ldp", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["enable_ldp"]
            ),
            DeclareLaunchArgument("log_level", default_value="none"),
            wrapped_launch,
        ]
    )
