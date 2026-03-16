import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    visual_landing_share = get_package_share_directory("uav_visual_landing")

    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    search_height_m = LaunchConfiguration("search_height_m")
    terminal_entry_height_m = LaunchConfiguration("terminal_entry_height_m")

    image_topic = "/uav/camera/image_raw"
    camera_info_topic = "/uav/camera/camera_info"
    fmu_namespace = "/fmu"
    vehicle_local_position_topic = f"{fmu_namespace}/out/vehicle_local_position"
    distance_sensor_topic = f"{fmu_namespace}/out/distance_sensor"

    minimal_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "hw_minimal_control.launch.py")
        ),
        launch_arguments={
            "fmu_namespace": fmu_namespace,
            "takeoff_height_m": takeoff_height_m,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
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
            {"tag_size_m": 0.1625},
            {"image_topic": image_topic},
            {"camera_info_topic": camera_info_topic},
            {"target_observation_topic": "/uav/visual_landing/target_observation"},
            {"controller_state_topic": "/uav/visual_landing/controller_state"},
            {"debug_image_topic": "/uav/visual_landing/debug_image"},
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
            {"target_observation_topic": "/uav/visual_landing/target_observation"},
            {"state_topic": "/uav/state/odometry"},
            {"velocity_body_topic": "/uav/control/setpoint/velocity_body"},
            {"controller_state_topic": "/uav/visual_landing/controller_state"},
            {"hold_service": "/uav/control/command/hold"},
            {"land_service": "/uav/control/command/land"},
            {"height_measurement_mode": "distance_sensor"},
            {"range_topic": distance_sensor_topic},
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"start_service": "/uav/visual_landing/command/start"},
            {"stop_service": "/uav/visual_landing/command/stop"},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("takeoff_height_m", default_value="0.60"),
        DeclareLaunchArgument("search_height_m", default_value="0.60"),
        DeclareLaunchArgument("terminal_entry_height_m", default_value="0.40"),
        minimal_control_launch,
        aruco_detector,
        visual_landing,
    ])
