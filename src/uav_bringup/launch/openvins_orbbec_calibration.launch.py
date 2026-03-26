import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from uav_bringup.profile_defaults import (
    DEFAULT_ORBBEC_BOOTSTRAP_PROFILE,
    DEFAULT_ORBBEC_IR_EXPOSURE,
    DEFAULT_ORBBEC_IR_STREAM,
)


_OPENVINS_TUNING_PRESETS = {
    "default": "estimator_config.calibration.yaml",
    "balanced": "estimator_config.calibration.autoexposure_balanced.yaml",
    "msckf_bias": "estimator_config.calibration.autoexposure_msckf_bias.yaml",
    "slam_bias": "estimator_config.calibration.autoexposure_slam_bias.yaml",
}


def _resolve_openvins_config_path(bringup_share: str, profile: str, explicit_path: str) -> Path:
    if explicit_path:
        return Path(explicit_path)

    if profile not in _OPENVINS_TUNING_PRESETS:
        available_profiles = ", ".join(sorted(_OPENVINS_TUNING_PRESETS.keys()))
        raise RuntimeError(
            f"Unknown OpenVINS tuning profile '{profile}'. Available profiles: {available_profiles}."
        )

    return Path(bringup_share) / "config" / "openvins" / "orbbec_stereo_imu" / "bootstrap" / _OPENVINS_TUNING_PRESETS[profile]


def _prepare_calibration(context, *_args, **_kwargs):
    bringup_share = get_package_share_directory("uav_bringup")

    state_output_dir = Path(LaunchConfiguration("state_output_dir").perform(context))
    state_output_dir.mkdir(parents=True, exist_ok=True)

    profile = LaunchConfiguration("openvins_tuning_profile").perform(context).strip()
    explicit_config_path = LaunchConfiguration("openvins_config_path").perform(context).strip()
    config_path = _resolve_openvins_config_path(bringup_share, profile, explicit_config_path)
    if not config_path.exists():
        raise RuntimeError(
            "OpenVINS calibration config not found: "
            f"{config_path}. Run generate_orbbec_openvins_bootstrap.py first."
        )

    return [SetLaunchConfiguration("resolved_openvins_config_path", str(config_path))]


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    state_output_dir = LaunchConfiguration("state_output_dir")

    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "openvins_orbbec.launch.py")
        ),
        launch_arguments={
            "start_openvins": LaunchConfiguration("start_openvins"),
            "start_orbbec_camera": LaunchConfiguration("start_orbbec_camera"),
            "camera_name": LaunchConfiguration("camera_name"),
            "base_frame_id": LaunchConfiguration("base_frame_id"),
            "camera_frame_id": LaunchConfiguration("camera_frame_id"),
            "camera_x": LaunchConfiguration("camera_x"),
            "camera_y": LaunchConfiguration("camera_y"),
            "camera_z": LaunchConfiguration("camera_z"),
            "camera_roll": LaunchConfiguration("camera_roll"),
            "camera_pitch": LaunchConfiguration("camera_pitch"),
            "camera_yaw": LaunchConfiguration("camera_yaw"),
            "enable_depth": "true",
            "enable_publish_extrinsic": "true",
            "accel_rate": LaunchConfiguration("accel_rate"),
            "gyro_rate": LaunchConfiguration("gyro_rate"),
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
            "enable_frame_sync": LaunchConfiguration("enable_frame_sync"),
            "time_domain": LaunchConfiguration("time_domain"),
            "frame_aggregate_mode": LaunchConfiguration("frame_aggregate_mode"),
            "log_level": LaunchConfiguration("log_level"),
            "openvins_namespace": LaunchConfiguration("openvins_namespace"),
            "openvins_config_path": LaunchConfiguration("resolved_openvins_config_path"),
            "openvins_verbosity": LaunchConfiguration("openvins_verbosity"),
            "openvins_save_total_state": "true",
            "openvins_filepath_est": [state_output_dir, "/ov_estimate.txt"],
            "openvins_filepath_std": [state_output_dir, "/ov_estimate_std.txt"],
            "openvins_filepath_gt": [state_output_dir, "/ov_groundtruth.txt"],
            "publish_px4_external_vision": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_openvins", default_value="true"),
            DeclareLaunchArgument("start_orbbec_camera", default_value="true"),
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
            DeclareLaunchArgument("accel_rate", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["accel_rate"]),
            DeclareLaunchArgument("gyro_rate", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["gyro_rate"]),
            DeclareLaunchArgument("left_ir_width", default_value=DEFAULT_ORBBEC_IR_STREAM["width"]),
            DeclareLaunchArgument("left_ir_height", default_value=DEFAULT_ORBBEC_IR_STREAM["height"]),
            DeclareLaunchArgument("left_ir_fps", default_value=DEFAULT_ORBBEC_IR_STREAM["fps"]),
            DeclareLaunchArgument("left_ir_format", default_value="Y8"),
            DeclareLaunchArgument("right_ir_width", default_value=DEFAULT_ORBBEC_IR_STREAM["width"]),
            DeclareLaunchArgument("right_ir_height", default_value=DEFAULT_ORBBEC_IR_STREAM["height"]),
            DeclareLaunchArgument("right_ir_fps", default_value=DEFAULT_ORBBEC_IR_STREAM["fps"]),
            DeclareLaunchArgument("right_ir_format", default_value="Y8"),
            DeclareLaunchArgument("enable_ir_auto_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["enable_auto_exposure"]),
            DeclareLaunchArgument("ir_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["exposure"]),
            DeclareLaunchArgument("ir_gain", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["gain"]),
            DeclareLaunchArgument("ir_ae_max_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["ae_max_exposure"]),
            DeclareLaunchArgument("ir_brightness", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["brightness"]),
            DeclareLaunchArgument("enable_laser", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["enable_laser"]),
            DeclareLaunchArgument("enable_ldp", default_value=DEFAULT_ORBBEC_BOOTSTRAP_PROFILE["enable_ldp"]),
            DeclareLaunchArgument("enable_frame_sync", default_value="true"),
            DeclareLaunchArgument("time_domain", default_value="global"),
            DeclareLaunchArgument("frame_aggregate_mode", default_value="full_frame"),
            DeclareLaunchArgument("log_level", default_value="none"),
            DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("openvins_tuning_profile", default_value="default"),
            DeclareLaunchArgument("openvins_config_path", default_value=""),
            DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
            DeclareLaunchArgument(
                "state_output_dir", default_value="/tmp/openvins_orbbec_calibration"
            ),
            OpaqueFunction(function=_prepare_calibration),
            calibration_launch,
        ]
    )
