from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fmu_namespace = LaunchConfiguration("fmu_namespace")
    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    max_velocity_setpoint_mps = LaunchConfiguration("max_velocity_setpoint_mps")
    max_acceleration_setpoint_mps2 = LaunchConfiguration("max_acceleration_setpoint_mps2")

    vehicle_local_position_topic = [fmu_namespace, "/out/vehicle_local_position"]
    vehicle_odometry_topic = [fmu_namespace, "/out/vehicle_odometry"]
    vehicle_status_topic = [fmu_namespace, "/out/vehicle_status"]
    offboard_mode_topic = [fmu_namespace, "/in/offboard_control_mode"]
    trajectory_setpoint_topic = [fmu_namespace, "/in/trajectory_setpoint"]
    vehicle_command_topic = [fmu_namespace, "/in/vehicle_command"]

    fmu_topic_namespace_bridge = Node(
        package="uav_bridge",
        executable="fmu_topic_namespace_bridge_node",
        name="fmu_topic_namespace_bridge",
        output="screen",
        parameters=[
            {"namespaced_fmu_prefix": fmu_namespace},
            {"global_fmu_prefix": "/fmu"},
        ],
    )

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge",
        output="screen",
        parameters=[
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_odometry_topic": vehicle_odometry_topic},
            {"output_odometry_topic": "/uav/state/odometry"},
            {"map_frame_id": "uav_map"},
            {"odom_frame_id": "uav_odom"},
            {"base_frame_id": "uav_base_link"},
            {"publish_rate_hz": 50.0},
            {"publish_odometry": True},
            {"publish_tf": True},
            {"publish_map_to_odom_tf": True},
        ],
    )

    uav_control = Node(
        package="uav_bridge",
        executable="uav_control_node",
        name="uav_control",
        output="screen",
        parameters=[
            {"state_topic": "/uav/state/odometry"},
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
            {"takeoff_service": "/uav/control/command/takeoff"},
            {"hold_service": "/uav/control/command/hold"},
            {"position_mode_service": "/uav/control/command/position_mode"},
            {"land_service": "/uav/control/command/land"},
            {"abort_service": "/uav/control/command/abort"},
            {"disarm_service": "/uav/control/command/disarm"},
            {"takeoff_height_m": takeoff_height_m},
            {"max_velocity_setpoint_mps": max_velocity_setpoint_mps},
            {"max_acceleration_setpoint_mps2": max_acceleration_setpoint_mps2},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("fmu_namespace", default_value="/fmu"),
        DeclareLaunchArgument("takeoff_height_m", default_value="0.30"),
        DeclareLaunchArgument("max_velocity_setpoint_mps", default_value="0.40"),
        DeclareLaunchArgument("max_acceleration_setpoint_mps2", default_value="0.60"),
        fmu_topic_namespace_bridge,
        uav_state_bridge,
        uav_control,
    ])
