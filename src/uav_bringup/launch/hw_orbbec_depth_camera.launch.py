import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from uav_bringup.profile_defaults import DEFAULT_STEREO_CAMERA_TO_BODY


def generate_launch_description():
    orbbec_camera_share = get_package_share_directory("orbbec_camera")

    start_camera = LaunchConfiguration("start_camera")
    camera_launch_file = LaunchConfiguration("camera_launch_file")
    camera_name = LaunchConfiguration("camera_name")
    serial_number = LaunchConfiguration("serial_number")
    usb_port = LaunchConfiguration("usb_port")
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
    enable_ir = LaunchConfiguration("enable_ir")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_colored_point_cloud = LaunchConfiguration("enable_colored_point_cloud")
    log_level = LaunchConfiguration("log_level")

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([orbbec_camera_share, "launch", camera_launch_file])),
        condition=IfCondition(start_camera),
        launch_arguments={
            "camera_name": camera_name,
            "serial_number": serial_number,
            "usb_port": usb_port,
            "enable_depth": enable_depth,
            "enable_color": enable_color,
            "enable_ir": enable_ir,
            "enable_point_cloud": enable_point_cloud,
            "enable_colored_point_cloud": enable_colored_point_cloud,
            "publish_tf": "false",
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
            DeclareLaunchArgument("camera_launch_file", default_value="gemini_330_series.launch.py"),
            DeclareLaunchArgument("camera_name", default_value="uav/depth_camera"),
            DeclareLaunchArgument("serial_number", default_value=""),
            DeclareLaunchArgument("usb_port", default_value=""),
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
            DeclareLaunchArgument("enable_ir", default_value="false"),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            camera_mount_tf,
            depth_camera_launch,
        ]
    )
