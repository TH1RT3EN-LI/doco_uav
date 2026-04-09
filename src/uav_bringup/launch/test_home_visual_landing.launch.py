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
    start_micro_xrce_agent = LaunchConfiguration("start_micro_xrce_agent")
    micro_xrce_agent_executable = LaunchConfiguration("micro_xrce_agent_executable")
    micro_xrce_agent_transport = LaunchConfiguration("micro_xrce_agent_transport")
    micro_xrce_agent_port = LaunchConfiguration("micro_xrce_agent_port")
    micro_xrce_agent_device = LaunchConfiguration("micro_xrce_agent_device")
    micro_xrce_agent_baudrate = LaunchConfiguration("micro_xrce_agent_baudrate")
    micro_xrce_agent_middleware = LaunchConfiguration("micro_xrce_agent_middleware")
    micro_xrce_agent_refs_file = LaunchConfiguration("micro_xrce_agent_refs_file")
    micro_xrce_agent_verbose_level = LaunchConfiguration("micro_xrce_agent_verbose_level")

    visual_landing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "visual_landing.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_micro_xrce_agent": start_micro_xrce_agent,
            "micro_xrce_agent_executable": micro_xrce_agent_executable,
            "micro_xrce_agent_transport": micro_xrce_agent_transport,
            "micro_xrce_agent_port": micro_xrce_agent_port,
            "micro_xrce_agent_device": micro_xrce_agent_device,
            "micro_xrce_agent_baudrate": micro_xrce_agent_baudrate,
            "micro_xrce_agent_middleware": micro_xrce_agent_middleware,
            "micro_xrce_agent_refs_file": micro_xrce_agent_refs_file,
            "micro_xrce_agent_verbose_level": micro_xrce_agent_verbose_level,
        }.items(),
    )

    home_visual_landing_test = Node(
        package="uav_bringup",
        executable="home_visual_landing_test_node.py",
        name="home_visual_landing_test_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"state_topic": "/uav/state/odometry_px4"},
            {"position_topic": "/uav/control/position_yaw"},
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
        DeclareLaunchArgument("start_micro_xrce_agent", default_value="false"),
        DeclareLaunchArgument("micro_xrce_agent_executable", default_value="MicroXRCEAgent"),
        DeclareLaunchArgument("micro_xrce_agent_transport", default_value="udp4"),
        DeclareLaunchArgument("micro_xrce_agent_port", default_value="8888"),
        DeclareLaunchArgument("micro_xrce_agent_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("micro_xrce_agent_baudrate", default_value="921600"),
        DeclareLaunchArgument("micro_xrce_agent_middleware", default_value="dds"),
        DeclareLaunchArgument("micro_xrce_agent_refs_file", default_value=""),
        DeclareLaunchArgument("micro_xrce_agent_verbose_level", default_value=""),
        visual_landing_launch,
        home_visual_landing_test,
    ])
