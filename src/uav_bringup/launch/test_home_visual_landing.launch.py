import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONTROLLER_STATE_TOPIC = "/uav/visual_landing/controller_state"


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    visual_landing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "visual_landing.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    home_visual_landing_test = Node(
        package="uav_bringup",
        executable="home_visual_landing_test_node.py",
        name="home_visual_landing_test_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"state_topic": "/uav/state/odometry"},
            {"position_topic": "/uav/control/pose"},
            {"takeoff_service": "/uav/control/command/takeoff"},
            {"hold_service": "/uav/control/command/hold"},
            {"visual_landing_start_service": "/uav/visual_landing/command/start"},
            {"visual_landing_state_topic": CONTROLLER_STATE_TOPIC},
            {"home_takeoff_service": "/uav/test/home_takeoff"},
            {"home_visual_land_service": "/uav/test/home_visual_land"},
            {"arrival_xy_tolerance_m": 0.15},
            {"state_timeout_s": 0.20},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        visual_landing_launch,
        home_visual_landing_test,
    ])
