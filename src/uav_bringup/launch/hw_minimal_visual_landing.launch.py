import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


DEBUG_IMAGE_TOPIC = "/uav/visual_landing/debug_image"
MONO_IMAGE_TOPIC = "/uav/camera/image_raw"
CAMERA_INFO_TOPIC = "/uav/camera/camera_info"
TARGET_OBSERVATION_TOPIC = "/uav/visual_landing/target_observation"
CONTROLLER_STATE_TOPIC = "/uav/visual_landing/controller_state"


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    visual_landing_share = get_package_share_directory("uav_visual_landing")

    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    search_height_m = LaunchConfiguration("search_height_m")
    terminal_entry_height_m = LaunchConfiguration("terminal_entry_height_m")
    max_vxy = LaunchConfiguration("max_vxy")
    align_enter_lateral_m = LaunchConfiguration("align_enter_lateral_m")
    align_exit_lateral_m = LaunchConfiguration("align_exit_lateral_m")
    tag_size_m = LaunchConfiguration("tag_size_m")
    record_video_source = LaunchConfiguration("record_video_source")
    record_video_output_path = LaunchConfiguration("record_video_output_path")
    record_video_fps = LaunchConfiguration("record_video_fps")
    record_video_fourcc = LaunchConfiguration("record_video_fourcc")

    fmu_namespace = "/fmu"
    vehicle_local_position_topic = f"{fmu_namespace}/out/vehicle_local_position"
    distance_sensor_topic = f"{fmu_namespace}/out/distance_sensor"
    default_recording_path = str(
        Path.home() / "uav_recordings" / f"visual_landing_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    )

    minimal_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "hw_minimal_control.launch.py")
        ),
        launch_arguments={
            "fmu_namespace": fmu_namespace,
            "takeoff_height_m": takeoff_height_m,
            "image_topic": MONO_IMAGE_TOPIC,
            "camera_info_topic": CAMERA_INFO_TOPIC,
            "record_mono_video": "false",
        }.items(),
    )

    visual_landing_config = os.path.join(
        visual_landing_share, "config", "visual_landing_stable.yaml"
    )

    aruco_detector = Node(
        package="uav_visual_landing",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[
            {"target_marker_id": 0},
            {"tag_size_m": tag_size_m},
            {"image_topic": MONO_IMAGE_TOPIC},
            {"camera_info_topic": CAMERA_INFO_TOPIC},
            {"target_observation_topic": TARGET_OBSERVATION_TOPIC},
            {"controller_state_topic": CONTROLLER_STATE_TOPIC},
            {"debug_image_topic": DEBUG_IMAGE_TOPIC},
        ],
    )

    visual_landing = Node(
        package="uav_visual_landing",
        executable="visual_landing_node",
        name="visual_landing_node",
        output="screen",
        parameters=[
            visual_landing_config,
            {"search_height_m": search_height_m},
            {"terminal_entry_height_m": terminal_entry_height_m},
            {"max_vxy": max_vxy},
            {"align_enter_lateral_m": align_enter_lateral_m},
            {"align_exit_lateral_m": align_exit_lateral_m},
            {"target_observation_topic": TARGET_OBSERVATION_TOPIC},
            {"state_topic": "/uav/state/odometry"},
            {"velocity_body_topic": "/uav/control/setpoint/velocity_body"},
            {"controller_state_topic": CONTROLLER_STATE_TOPIC},
            {"hold_service": "/uav/control/command/hold"},
            {"land_service": "/uav/control/command/land"},
            {"height_measurement_mode": "distance_sensor"},
            {"range_topic": distance_sensor_topic},
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"start_service": "/uav/visual_landing/command/start"},
            {"stop_service": "/uav/visual_landing/command/stop"},
        ],
    )

    debug_video_recorder = Node(
        package="uav_bridge",
        executable="mono_video_recorder_node",
        name="debug_video_recorder",
        output="screen",
        condition=IfCondition(PythonExpression(["'", record_video_source, "' == 'debug'"])),
        parameters=[
            {"image_topic": DEBUG_IMAGE_TOPIC},
            {"output_path": record_video_output_path},
            {"fps": record_video_fps},
            {"fourcc": record_video_fourcc},
        ],
    )

    mono_video_recorder = Node(
        package="uav_bridge",
        executable="mono_video_recorder_node",
        name="mono_video_recorder",
        output="screen",
        condition=IfCondition(PythonExpression(["'", record_video_source, "' == 'mono'"])),
        parameters=[
            {"image_topic": MONO_IMAGE_TOPIC},
            {"output_path": record_video_output_path},
            {"fps": record_video_fps},
            {"fourcc": record_video_fourcc},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("takeoff_height_m", default_value="0.60"),
        DeclareLaunchArgument("search_height_m", default_value="0.60"),
        DeclareLaunchArgument("terminal_entry_height_m", default_value="0.40"),
        DeclareLaunchArgument("max_vxy", default_value="0.10"),
        DeclareLaunchArgument("align_enter_lateral_m", default_value="0.08"),
        DeclareLaunchArgument("align_exit_lateral_m", default_value="0.05"),
        DeclareLaunchArgument("tag_size_m", default_value="0.13"),
        DeclareLaunchArgument("record_video_source", default_value="debug"),
        DeclareLaunchArgument("record_video_output_path", default_value=default_recording_path),
        DeclareLaunchArgument("record_video_fps", default_value="120.0"),
        DeclareLaunchArgument("record_video_fourcc", default_value="MJPG"),
        minimal_control_launch,
        aruco_detector,
        visual_landing,
        debug_video_recorder,
        mono_video_recorder,
    ])
