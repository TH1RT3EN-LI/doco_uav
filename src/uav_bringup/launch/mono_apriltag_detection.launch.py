import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_bringup.profile_defaults import DEFAULT_CAMERA_INTRINSICS, DEFAULT_CAMERA_TO_BODY


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    backend = LaunchConfiguration("backend")
    device = LaunchConfiguration("device")
    fourcc = LaunchConfiguration("fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    fps = LaunchConfiguration("fps")
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

    target_observation_topic = LaunchConfiguration("target_observation_topic")
    tag_detection_topic = LaunchConfiguration("tag_detection_topic")
    tag_pose_topic = LaunchConfiguration("tag_pose_topic")
    tag_marker_topic = LaunchConfiguration("tag_marker_topic")
    tag_output_frame_mode = LaunchConfiguration("tag_output_frame_mode")
    controller_state_topic = LaunchConfiguration("controller_state_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    target_marker_id = LaunchConfiguration("target_marker_id")
    tag_family = LaunchConfiguration("tag_family")
    tag_size_m = LaunchConfiguration("tag_size_m")
    tag_frame_yaw_offset_rad = LaunchConfiguration("tag_frame_yaw_offset_rad")
    odometry_topic = LaunchConfiguration("odometry_topic")
    odometry_timeout_s = LaunchConfiguration("odometry_timeout_s")
    mono_dx = LaunchConfiguration("mono_dx")
    mono_dy = LaunchConfiguration("mono_dy")
    mono_dz = LaunchConfiguration("mono_dz")
    mono_droll = LaunchConfiguration("mono_droll")
    mono_dpitch = LaunchConfiguration("mono_dpitch")
    mono_dyaw = LaunchConfiguration("mono_dyaw")
    publish_tag_base_tf = LaunchConfiguration("publish_tag_base_tf")
    publish_tag_odom_tf = LaunchConfiguration("publish_tag_odom_tf")
    tag_tf_frame_prefix = LaunchConfiguration("tag_tf_frame_prefix")

    mono_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "mono_camera.launch.py")
        ),
        launch_arguments={
            "backend": backend,
            "device": device,
            "fourcc": fourcc,
            "image_width": image_width,
            "image_height": image_height,
            "fps": fps,
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

    aruco_detector = Node(
        package="uav_visual_landing",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"image_topic": image_topic},
            {"camera_info_topic": camera_info_topic},
            {"target_observation_topic": target_observation_topic},
            {"tag_detection_topic": tag_detection_topic},
            {"tag_pose_topic": tag_pose_topic},
            {"tag_marker_topic": tag_marker_topic},
            {"tag_output_frame_mode": tag_output_frame_mode},
            {"controller_state_topic": controller_state_topic},
            {"debug_image_topic": debug_image_topic},
            {"camera_frame_id": camera_frame_id},
            {"base_frame_id": base_frame_id},
            {"target_marker_id": target_marker_id},
            {"tag_family": tag_family},
            {"tag_size_m": tag_size_m},
            {"tag_frame_yaw_offset_rad": tag_frame_yaw_offset_rad},
            {"odometry_topic": odometry_topic},
            {"odometry_timeout_s": odometry_timeout_s},
            {"mono_in_ov_x_m": mono_dx},
            {"mono_in_ov_y_m": mono_dy},
            {"mono_in_ov_z_m": mono_dz},
            {"mono_in_ov_roll_rad": mono_droll},
            {"mono_in_ov_pitch_rad": mono_dpitch},
            {"mono_in_ov_yaw_rad": mono_dyaw},
            {"publish_tag_base_tf": publish_tag_base_tf},
            {"publish_tag_odom_tf": publish_tag_odom_tf},
            {"tag_tf_frame_prefix": tag_tf_frame_prefix},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("backend", default_value="legacy"),
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("fourcc", default_value="MJPG"),
        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("fps", default_value="120.0"),
        DeclareLaunchArgument("camera_name", default_value="uav_mono_camera"),
        DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
        DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
        DeclareLaunchArgument("camera_frame_id", default_value="uav_camera_optical_frame"),
        DeclareLaunchArgument("camera_x", default_value=DEFAULT_CAMERA_TO_BODY["x"]),
        DeclareLaunchArgument("camera_y", default_value=DEFAULT_CAMERA_TO_BODY["y"]),
        DeclareLaunchArgument("camera_z", default_value=DEFAULT_CAMERA_TO_BODY["z"]),
        DeclareLaunchArgument("camera_roll", default_value=str(math.pi)),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("fx", default_value=DEFAULT_CAMERA_INTRINSICS["fx"]),
        DeclareLaunchArgument("fy", default_value=DEFAULT_CAMERA_INTRINSICS["fy"]),
        DeclareLaunchArgument("cx", default_value=DEFAULT_CAMERA_INTRINSICS["cx"]),
        DeclareLaunchArgument("cy", default_value=DEFAULT_CAMERA_INTRINSICS["cy"]),
        DeclareLaunchArgument("camera_hfov_rad", default_value="1.3962634"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument(
            "target_observation_topic",
            default_value="/uav/visual_landing/target_observation",
        ),
        DeclareLaunchArgument(
            "tag_detection_topic",
            default_value="/uav/visual_landing/apriltag_detection",
        ),
        DeclareLaunchArgument(
            "tag_pose_topic",
            default_value="/uav/visual_landing/apriltag_pose",
        ),
        DeclareLaunchArgument(
            "tag_marker_topic",
            default_value="/uav/visual_landing/apriltag_marker",
        ),
        DeclareLaunchArgument("tag_output_frame_mode", default_value="base"),
        DeclareLaunchArgument("controller_state_topic", default_value=""),
        DeclareLaunchArgument(
            "debug_image_topic",
            default_value="/uav/visual_landing/debug_image",
        ),
        DeclareLaunchArgument("target_marker_id", default_value="0"),
        DeclareLaunchArgument("tag_family", default_value="36h11"),
        DeclareLaunchArgument("tag_size_m", default_value="0.20"),
        DeclareLaunchArgument("tag_frame_yaw_offset_rad", default_value="0.0"),
        DeclareLaunchArgument("odometry_topic", default_value="/ov_msckf/odomimu"),
        DeclareLaunchArgument("odometry_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("mono_dx", default_value="0.0"),
        DeclareLaunchArgument("mono_dy", default_value="0.0"),
        DeclareLaunchArgument("mono_dz", default_value="0.0"),
        DeclareLaunchArgument("mono_droll", default_value="0.0"),
        DeclareLaunchArgument("mono_dpitch", default_value="0.0"),
        DeclareLaunchArgument("mono_dyaw", default_value="0.0"),
        DeclareLaunchArgument("publish_tag_base_tf", default_value="true"),
        DeclareLaunchArgument("publish_tag_odom_tf", default_value="true"),
        DeclareLaunchArgument("tag_tf_frame_prefix", default_value="apriltag"),
        mono_camera,
        aruco_detector,
    ])
