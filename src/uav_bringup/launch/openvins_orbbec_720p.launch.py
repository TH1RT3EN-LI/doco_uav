import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from uav_bringup.profile_defaults import (
    DEFAULT_ORBBEC_IR_EXPOSURE,
    DEFAULT_ORBBEC_IR_STREAM_720P,
)


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    default_config_path = os.path.join(
        bringup_share,
        "config",
        "openvins",
        "orbbec_stereo_imu",
        "frozen_final",
        "estimator_config.flight.yaml",
    )

    wrapped_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "openvins_orbbec.launch.py")
        ),
        launch_arguments={
            "start_openvins": LaunchConfiguration("start_openvins"),
            "start_orbbec_camera": LaunchConfiguration("start_orbbec_camera"),
            "camera_name": LaunchConfiguration("camera_name"),
            "openvins_namespace": LaunchConfiguration("openvins_namespace"),
            "openvins_config_path": LaunchConfiguration("openvins_config_path"),
            "openvins_verbosity": LaunchConfiguration("openvins_verbosity"),
            "publish_px4_external_vision": LaunchConfiguration(
                "publish_px4_external_vision"
            ),
            "fmu_namespace": LaunchConfiguration("fmu_namespace"),
            "sensor_roll_in_body_rad": LaunchConfiguration("sensor_roll_in_body_rad"),
            "sensor_pitch_in_body_rad": LaunchConfiguration("sensor_pitch_in_body_rad"),
            "sensor_yaw_in_body_rad": LaunchConfiguration("sensor_yaw_in_body_rad"),
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
            DeclareLaunchArgument("start_openvins", default_value="true"),
            DeclareLaunchArgument("start_orbbec_camera", default_value="true"),
            DeclareLaunchArgument("camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("openvins_config_path", default_value=default_config_path),
            DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
            DeclareLaunchArgument("publish_px4_external_vision", default_value="true"),
            DeclareLaunchArgument("fmu_namespace", default_value="/fmu"),
            DeclareLaunchArgument("sensor_roll_in_body_rad", default_value="0.0"),
            DeclareLaunchArgument("sensor_pitch_in_body_rad", default_value="0.0"),
            DeclareLaunchArgument("sensor_yaw_in_body_rad", default_value="0.0"),
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
            DeclareLaunchArgument("enable_laser", default_value="false"),
            DeclareLaunchArgument("enable_ldp", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="none"),
            wrapped_launch,
        ]
    )
