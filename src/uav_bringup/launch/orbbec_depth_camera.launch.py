import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from uav_bringup.profile_defaults import DEFAULT_STEREO_CAMERA_TO_BODY


GEMINI_336_LAUNCH_FILE = "gemini_330_series.launch.py"


def generate_launch_description():
    orbbec_camera_share = get_package_share_directory("orbbec_camera")

    start_camera = LaunchConfiguration("start_camera")
    camera_name = LaunchConfiguration("camera_name")
    base_frame_id = LaunchConfiguration("base_frame_id")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    publish_mount_tf = LaunchConfiguration("publish_mount_tf")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_color = LaunchConfiguration("enable_color")
    enable_left_ir = LaunchConfiguration("enable_left_ir")
    enable_right_ir = LaunchConfiguration("enable_right_ir")
    left_ir_width = LaunchConfiguration("left_ir_width")
    left_ir_height = LaunchConfiguration("left_ir_height")
    left_ir_fps = LaunchConfiguration("left_ir_fps")
    left_ir_format = LaunchConfiguration("left_ir_format")
    right_ir_width = LaunchConfiguration("right_ir_width")
    right_ir_height = LaunchConfiguration("right_ir_height")
    right_ir_fps = LaunchConfiguration("right_ir_fps")
    right_ir_format = LaunchConfiguration("right_ir_format")
    enable_ir_auto_exposure = LaunchConfiguration("enable_ir_auto_exposure")
    enable_laser = LaunchConfiguration("enable_laser")
    enable_ldp = LaunchConfiguration("enable_ldp")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_colored_point_cloud = LaunchConfiguration("enable_colored_point_cloud")
    enable_sync_output_accel_gyro = LaunchConfiguration("enable_sync_output_accel_gyro")
    enable_publish_extrinsic = LaunchConfiguration("enable_publish_extrinsic")
    enable_accel = LaunchConfiguration("enable_accel")
    accel_rate = LaunchConfiguration("accel_rate")
    enable_gyro = LaunchConfiguration("enable_gyro")
    gyro_rate = LaunchConfiguration("gyro_rate")
    log_level = LaunchConfiguration("log_level")

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([orbbec_camera_share, "launch", GEMINI_336_LAUNCH_FILE])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "camera_name": camera_name,
            "enable_depth": enable_depth,
            "enable_color": enable_color,
            "enable_left_ir": enable_left_ir,
            "enable_right_ir": enable_right_ir,
            "left_ir_width": left_ir_width,
            "left_ir_height": left_ir_height,
            "left_ir_fps": left_ir_fps,
            "left_ir_format": left_ir_format,
            "right_ir_width": right_ir_width,
            "right_ir_height": right_ir_height,
            "right_ir_fps": right_ir_fps,
            "right_ir_format": right_ir_format,
            "enable_ir_auto_exposure": enable_ir_auto_exposure,
            "enable_laser": enable_laser,
            "enable_ldp": enable_ldp,
            "enable_point_cloud": enable_point_cloud,
            "enable_colored_point_cloud": enable_colored_point_cloud,
            "enable_sync_output_accel_gyro": enable_sync_output_accel_gyro,
            "enable_publish_extrinsic": enable_publish_extrinsic,
            "enable_accel": enable_accel,
            "accel_rate": accel_rate,
            "enable_gyro": enable_gyro,
            "gyro_rate": gyro_rate,
            "publish_tf": enable_publish_extrinsic,
            "depth_optical_frame_id": camera_frame_id,
            "cloud_frame_id": camera_frame_id,
            "log_level": log_level,
        }.items(),
    )

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uav_orbbec_depth_camera_static_tf",
        output="screen",
        condition=IfCondition(publish_mount_tf),
        arguments=[
            "--x",
            camera_x,
            "--y",
            camera_y,
            "--z",
            camera_z,
            "--roll",
            camera_roll,
            "--pitch",
            camera_pitch,
            "--yaw",
            camera_yaw,
            "--frame-id",
            base_frame_id,
            "--child-frame-id",
            camera_frame_id,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument("camera_frame_id", default_value="uav_stereo_camera_optical_frame"),
            DeclareLaunchArgument("camera_x", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["x"]),
            DeclareLaunchArgument("camera_y", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["y"]),
            DeclareLaunchArgument("camera_z", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["z"]),
            DeclareLaunchArgument("camera_roll", default_value=str(-math.pi / 2.0)),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value=str(-math.pi / 2.0)),
            DeclareLaunchArgument("publish_mount_tf", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_color", default_value="false"),
            DeclareLaunchArgument("enable_left_ir", default_value="false"),
            DeclareLaunchArgument("enable_right_ir", default_value="false"),
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
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="false"),
            DeclareLaunchArgument("enable_publish_extrinsic", default_value="false"),
            DeclareLaunchArgument("enable_accel", default_value="false"),
            DeclareLaunchArgument("accel_rate", default_value="200hz"),
            DeclareLaunchArgument("enable_gyro", default_value="false"),
            DeclareLaunchArgument("gyro_rate", default_value="200hz"),
            DeclareLaunchArgument("log_level", default_value="info"),
            camera_mount_tf,
            depth_camera_launch,
        ]
    )
