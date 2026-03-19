import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def _prepare_calibration(context, *_args, **_kwargs):
    state_output_dir = Path(LaunchConfiguration("state_output_dir").perform(context))
    state_output_dir.mkdir(parents=True, exist_ok=True)

    config_path = Path(LaunchConfiguration("openvins_config_path").perform(context))
    if not config_path.exists():
        raise RuntimeError(
            "OpenVINS calibration config not found: "
            f"{config_path}. Run generate_orbbec_openvins_bootstrap.py first "
            "or override openvins_config_path."
        )

    return []



def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    default_config_path = os.path.join(
        bringup_share,
        "config",
        "openvins",
        "orbbec_stereo_imu",
        "bootstrap",
        "estimator_config.calibration.yaml",
    )

    state_output_dir = LaunchConfiguration("state_output_dir")
    config_path = LaunchConfiguration("openvins_config_path")

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
            "enable_depth": LaunchConfiguration("enable_depth"),
            "enable_color": LaunchConfiguration("enable_color"),
            "enable_left_ir": LaunchConfiguration("enable_left_ir"),
            "enable_right_ir": LaunchConfiguration("enable_right_ir"),
            "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
            "enable_colored_point_cloud": LaunchConfiguration("enable_colored_point_cloud"),
            "enable_sync_output_accel_gyro": LaunchConfiguration("enable_sync_output_accel_gyro"),
            "enable_publish_extrinsic": LaunchConfiguration("enable_publish_extrinsic"),
            "enable_accel": LaunchConfiguration("enable_accel"),
            "enable_gyro": LaunchConfiguration("enable_gyro"),
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
            "enable_laser": LaunchConfiguration("enable_laser"),
            "enable_ldp": LaunchConfiguration("enable_ldp"),
            "log_level": LaunchConfiguration("log_level"),
            "openvins_namespace": LaunchConfiguration("openvins_namespace"),
            "openvins_config_path": config_path,
            "openvins_verbosity": LaunchConfiguration("openvins_verbosity"),
            "openvins_save_total_state": "true",
            "openvins_publish_calibration_tf": "true",
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
            DeclareLaunchArgument("camera_frame_id", default_value="uav_stereo_camera_optical_frame"),
            DeclareLaunchArgument("camera_x", default_value="0.0503"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.1043"),
            DeclareLaunchArgument("camera_roll", default_value=str(-3.141592653589793 / 2.0)),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value=str(-3.141592653589793 / 2.0)),
            DeclareLaunchArgument("enable_depth", default_value="false"),
            DeclareLaunchArgument("enable_color", default_value="false"),
            DeclareLaunchArgument("enable_left_ir", default_value="true"),
            DeclareLaunchArgument("enable_right_ir", default_value="true"),
            DeclareLaunchArgument("enable_point_cloud", default_value="false"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="true"),
            DeclareLaunchArgument("enable_publish_extrinsic", default_value="true"),
            DeclareLaunchArgument("enable_accel", default_value="true"),
            DeclareLaunchArgument("enable_gyro", default_value="true"),
            DeclareLaunchArgument("accel_rate", default_value="200hz"),
            DeclareLaunchArgument("gyro_rate", default_value="200hz"),
            DeclareLaunchArgument("left_ir_width", default_value="0"),
            DeclareLaunchArgument("left_ir_height", default_value="0"),
            DeclareLaunchArgument("left_ir_fps", default_value="0"),
            DeclareLaunchArgument("left_ir_format", default_value="ANY"),
            DeclareLaunchArgument("right_ir_width", default_value="0"),
            DeclareLaunchArgument("right_ir_height", default_value="0"),
            DeclareLaunchArgument("right_ir_fps", default_value="0"),
            DeclareLaunchArgument("right_ir_format", default_value="ANY"),
            DeclareLaunchArgument("enable_ir_auto_exposure", default_value="true"),
            DeclareLaunchArgument("enable_laser", default_value="false"),
            DeclareLaunchArgument("enable_ldp", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("openvins_config_path", default_value=default_config_path),
            DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
            DeclareLaunchArgument(
                "state_output_dir",
                default_value="/tmp/openvins_orbbec_calibration",
            ),
            OpaqueFunction(function=_prepare_calibration),
            calibration_launch,
        ]
    )
