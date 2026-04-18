import math
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from uav_bringup.launch_utils import namespaced_path


def next_bag_run_id(base_dir: Path, prefix: str) -> int:
    if not base_dir.exists():
        return 1

    max_id = 0
    stem = f"{prefix}_"
    for entry in base_dir.iterdir():
        if not entry.name.startswith(stem):
            continue

        suffix = entry.name[len(stem) :]
        run_id, _, _ = suffix.partition("_")
        if run_id.isdigit():
            max_id = max(max_id, int(run_id))

    return max_id + 1


def _launch_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _parse_six_dof(value: str, arg_name: str):
    parts = [part.strip() for part in value.replace(",", " ").split() if part.strip()]
    if len(parts) != 6:
        raise RuntimeError(
            f"{arg_name} must contain 6 numeric values: x y z roll pitch yaw"
        )

    try:
        return tuple(float(part) for part in parts)
    except ValueError as exc:
        raise RuntimeError(
            f"{arg_name} must contain numeric values: x y z roll pitch yaw"
        ) from exc


def _parse_xy(value: str, arg_name: str):
    parts = [part.strip() for part in value.replace(",", " ").split() if part.strip()]
    if len(parts) != 2:
        raise RuntimeError(f"{arg_name} must contain 2 numeric values: x y")

    try:
        return tuple(float(part) for part in parts)
    except ValueError as exc:
        raise RuntimeError(f"{arg_name} must contain numeric values: x y") from exc


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    sin_r, cos_r = math.sin(half_roll), math.cos(half_roll)
    sin_p, cos_p = math.sin(half_pitch), math.cos(half_pitch)
    sin_y, cos_y = math.sin(half_yaw), math.cos(half_yaw)
    return (
        (sin_r * cos_p * cos_y) - (cos_r * sin_p * sin_y),
        (cos_r * sin_p * cos_y) + (sin_r * cos_p * sin_y),
        (cos_r * cos_p * sin_y) - (sin_r * sin_p * cos_y),
        (cos_r * cos_p * cos_y) + (sin_r * sin_p * sin_y),
    )


def _resolve_global_to_uav_map(context, *args, **kwargs):
    if not _launch_bool(
        LaunchConfiguration("compute_uav_map_from_ugv_relative").perform(context)
    ):
        return [
            SetLaunchConfiguration(
                "resolved_global_to_uav_map",
                LaunchConfiguration("global_to_uav_map").perform(context),
            )
        ]

    ugv_x, ugv_y, ugv_z, ugv_roll, ugv_pitch, ugv_yaw = _parse_six_dof(
        LaunchConfiguration("global_to_ugv_map").perform(context),
        "global_to_ugv_map",
    )
    relative_x, relative_y = _parse_xy(
        LaunchConfiguration("uav_initial_relative_xy_to_ugv").perform(context),
        "uav_initial_relative_xy_to_ugv",
    )
    z_offset = float(LaunchConfiguration("uav_map_z_offset").perform(context))
    yaw_offset = float(LaunchConfiguration("uav_map_yaw_offset").perform(context))

    cos_yaw = math.cos(ugv_yaw)
    sin_yaw = math.sin(ugv_yaw)

    resolved_x = ugv_x + (cos_yaw * relative_x) - (sin_yaw * relative_y)
    resolved_y = ugv_y + (sin_yaw * relative_x) + (cos_yaw * relative_y)
    resolved_z = ugv_z + z_offset
    resolved_yaw = ugv_yaw + yaw_offset
    resolved_value = (
        f"{resolved_x:.15g},{resolved_y:.15g},{resolved_z:.15g},"
        f"{ugv_roll:.15g},{ugv_pitch:.15g},{resolved_yaw:.15g}"
    )
    return [SetLaunchConfiguration("resolved_global_to_uav_map", resolved_value)]


def _build_global_to_uav_map_static_tf(context, *args, **kwargs):
    global_frame_name = LaunchConfiguration("global_frame").perform(context).strip()
    uav_map_frame_name = LaunchConfiguration("uav_map_frame").perform(context).strip()
    if (
        not global_frame_name
        or not uav_map_frame_name
        or global_frame_name == uav_map_frame_name
    ):
        return []

    x, y, z, roll, pitch, yaw = _parse_six_dof(
        LaunchConfiguration("resolved_global_to_uav_map").perform(context),
        "resolved_global_to_uav_map",
    )
    qx, qy, qz, qw = _quat_from_rpy(roll, pitch, yaw)
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="uav_demo_global_to_map_static_tf",
            output="screen",
            arguments=[
                "--x",
                f"{x:.15g}",
                "--y",
                f"{y:.15g}",
                "--z",
                f"{z:.15g}",
                "--qx",
                f"{qx:.15g}",
                "--qy",
                f"{qy:.15g}",
                "--qz",
                f"{qz:.15g}",
                "--qw",
                f"{qw:.15g}",
                "--frame-id",
                global_frame_name,
                "--child-frame-id",
                uav_map_frame_name,
            ],
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    description_share = get_package_share_directory("uav_description")
    relative_position_fusion_share = get_package_share_directory("relative_position_fusion")

    default_openvins_config = os.path.join(
        bringup_share, "config", "openvins", "orbbec_gemini336", "estimator_config.flight.yaml"
    )
    default_rviz_config = os.path.join(
        bringup_share, "config", "rviz", "openvins_orbbec_mono_apriltag.rviz"
    )
    default_relative_fusion_config = os.path.join(
        bringup_share, "config", "relative_position_fusion_uav_demo.yaml"
    )
    default_rc_safety_config = os.path.join(
        bringup_share, "config", "safety", "rc_safety_mux.yaml"
    )
    model_xacro = os.path.join(description_share, "urdf", "uav.urdf.xacro")

    bag_root_dir = Path.home() / "uav_bags"
    default_bag_id = next_bag_run_id(bag_root_dir, "uav_demo_search")
    default_bag_output_dir = str(
        bag_root_dir
        / f"uav_demo_search_{default_bag_id:03d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    start_perception_stack = LaunchConfiguration("start_perception_stack")
    start_orbbec_camera = LaunchConfiguration("start_orbbec_camera")
    start_openvins = LaunchConfiguration("start_openvins")
    start_px4_vision_bridge = LaunchConfiguration("start_px4_vision_bridge")
    start_uav_state_bridge = LaunchConfiguration("start_uav_state_bridge")
    guard_enable = LaunchConfiguration("guard_enable")
    openvins_namespace = LaunchConfiguration("openvins_namespace")
    openvins_config_path = LaunchConfiguration("openvins_config_path")
    openvins_verbosity = LaunchConfiguration("openvins_verbosity")
    orbbec_camera_name = LaunchConfiguration("orbbec_camera_name")
    camera_backend = LaunchConfiguration("camera_backend")
    camera_device = LaunchConfiguration("camera_device")
    camera_fourcc = LaunchConfiguration("camera_fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_fps = LaunchConfiguration("camera_fps")
    camera_name = LaunchConfiguration("camera_name")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_info_url = LaunchConfiguration("camera_info_url")
    target_marker_id = LaunchConfiguration("target_marker_id")
    tag_family = LaunchConfiguration("tag_family")
    tag_size_m = LaunchConfiguration("tag_size_m")
    tag_detection_topic = LaunchConfiguration("tag_detection_topic")
    tag_pose_topic = LaunchConfiguration("tag_pose_topic")
    tag_marker_topic = LaunchConfiguration("tag_marker_topic")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    uav_map_frame = LaunchConfiguration("uav_map_frame")
    uav_odom_frame = LaunchConfiguration("uav_odom_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    global_to_uav_map = LaunchConfiguration("global_to_uav_map")
    compute_uav_map_from_ugv_relative = LaunchConfiguration(
        "compute_uav_map_from_ugv_relative"
    )
    uav_initial_relative_xy_to_ugv = LaunchConfiguration(
        "uav_initial_relative_xy_to_ugv"
    )
    uav_map_z_offset = LaunchConfiguration("uav_map_z_offset")
    uav_map_yaw_offset = LaunchConfiguration("uav_map_yaw_offset")
    resolved_global_to_uav_map = LaunchConfiguration("resolved_global_to_uav_map")
    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    max_velocity_setpoint_mps = LaunchConfiguration("max_velocity_setpoint_mps")
    max_acceleration_setpoint_mps2 = LaunchConfiguration(
        "max_acceleration_setpoint_mps2"
    )
    control_state_topic = LaunchConfiguration("control_state_topic")
    execution_state_topic = LaunchConfiguration("execution_state_topic")
    position_topic = LaunchConfiguration("position_topic")
    position_keep_yaw_topic = LaunchConfiguration("position_keep_yaw_topic")
    velocity_body_topic = LaunchConfiguration("velocity_body_topic")
    position_delta_service = LaunchConfiguration("position_delta_service")
    position_command_frame_id = LaunchConfiguration("position_command_frame_id")
    px4_timestamp_source = LaunchConfiguration("px4_timestamp_source")
    timesync_status_topic = LaunchConfiguration("timesync_status_topic")
    state_bridge_output_odometry_topic = LaunchConfiguration(
        "state_bridge_output_odometry_topic"
    )
    trajectory_output_topic = LaunchConfiguration("trajectory_output_topic")
    trajectory_max_samples = LaunchConfiguration("trajectory_max_samples")
    trajectory_min_sample_distance_m = LaunchConfiguration(
        "trajectory_min_sample_distance_m"
    )
    trajectory_min_sample_period_s = LaunchConfiguration("trajectory_min_sample_period_s")
    start_rc_safety_mux = LaunchConfiguration("start_rc_safety_mux")
    rc_safety_config_path = LaunchConfiguration("rc_safety_config_path")
    start_demo_search_coordinator = LaunchConfiguration("start_demo_search_coordinator")
    demo_search_auto_start = LaunchConfiguration("demo_search_auto_start")
    demo_search_auto_start_delay_s = LaunchConfiguration(
        "demo_search_auto_start_delay_s"
    )
    demo_search_waypoints_body = LaunchConfiguration("demo_search_waypoints_body")
    demo_search_waypoint_reach_tolerance_m = LaunchConfiguration(
        "demo_search_waypoint_reach_tolerance_m"
    )
    demo_search_waypoint_timeout_s = LaunchConfiguration(
        "demo_search_waypoint_timeout_s"
    )
    demo_search_hold_on_finish = LaunchConfiguration("demo_search_hold_on_finish")
    demo_search_hold_on_stop = LaunchConfiguration("demo_search_hold_on_stop")
    demo_search_start_service = LaunchConfiguration("demo_search_start_service")
    demo_search_stop_service = LaunchConfiguration("demo_search_stop_service")
    demo_search_manual_waypoint_service = LaunchConfiguration(
        "demo_search_manual_waypoint_service"
    )
    demo_search_hold_service = LaunchConfiguration("demo_search_hold_service")
    demo_search_ugv_state_topic = LaunchConfiguration("demo_search_ugv_state_topic")
    demo_search_ugv_go_relative_service = LaunchConfiguration(
        "demo_search_ugv_go_relative_service"
    )
    demo_search_tag_confidence_threshold = LaunchConfiguration(
        "demo_search_tag_confidence_threshold"
    )
    demo_search_relative_pose_global_topic = LaunchConfiguration(
        "demo_search_relative_pose_global_topic"
    )
    demo_search_uav_state_timeout_s = LaunchConfiguration(
        "demo_search_uav_state_timeout_s"
    )
    demo_search_ugv_state_timeout_s = LaunchConfiguration(
        "demo_search_ugv_state_timeout_s"
    )
    demo_search_relative_pose_timeout_s = LaunchConfiguration(
        "demo_search_relative_pose_timeout_s"
    )
    demo_search_transform_timeout_s = LaunchConfiguration(
        "demo_search_transform_timeout_s"
    )
    demo_search_service_wait_timeout_s = LaunchConfiguration(
        "demo_search_service_wait_timeout_s"
    )
    demo_search_ugv_goal_offset_x_m = LaunchConfiguration(
        "demo_search_ugv_goal_offset_x_m"
    )
    demo_search_ugv_goal_offset_y_m = LaunchConfiguration(
        "demo_search_ugv_goal_offset_y_m"
    )
    enable_relative_position_fusion = LaunchConfiguration(
        "enable_relative_position_fusion"
    )
    enable_relative_tracking = LaunchConfiguration("enable_relative_tracking")
    relative_position_fusion_config = LaunchConfiguration(
        "relative_position_fusion_config"
    )
    publish_model = LaunchConfiguration("publish_model")
    use_record_bag = LaunchConfiguration("use_record_bag")
    bag_output_dir = LaunchConfiguration("bag_output_dir")
    base_frame_id = LaunchConfiguration("base_frame_id")
    camera_frame_id = LaunchConfiguration("camera_frame_id")

    actual_ov_trackhist_topic = namespaced_path(openvins_namespace, "trackhist")

    model_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                model_xacro,
                " prefix:=uav_model_",
                " base_frame:=base_link",
            ]
        ),
        value_type=str,
    )

    resolve_global_to_uav_map = OpaqueFunction(function=_resolve_global_to_uav_map)
    global_to_uav_map_static_tf = OpaqueFunction(
        function=_build_global_to_uav_map_static_tf
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "openvins_orbbec_mono_apriltag.launch.py")
        ),
        condition=IfCondition(start_perception_stack),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
            "start_orbbec_camera": start_orbbec_camera,
            "start_openvins": start_openvins,
            "start_px4_vision_bridge": start_px4_vision_bridge,
            "start_uav_state_bridge": "false",
            "guard_enable": guard_enable,
            "enable_relative_position_fusion": "false",
            "enable_relative_tracking": "false",
            "openvins_namespace": openvins_namespace,
            "openvins_config_path": openvins_config_path,
            "openvins_verbosity": openvins_verbosity,
            "state_bridge_output_odometry_topic": state_bridge_output_odometry_topic,
            "state_bridge_publish_tf": "false",
            "state_bridge_publish_map_to_odom_tf": "false",
            "global_frame": global_frame,
            "uav_map_frame": uav_map_frame,
            "uav_odom_frame": uav_odom_frame,
            "global_to_uav_map": resolved_global_to_uav_map,
            "publish_global_map_tf": "false",
            "orbbec_camera_name": orbbec_camera_name,
            "backend": camera_backend,
            "device": camera_device,
            "fourcc": camera_fourcc,
            "image_width": image_width,
            "image_height": image_height,
            "fps": camera_fps,
            "camera_name": camera_name,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "camera_info_url": camera_info_url,
            "base_frame_id": base_frame_id,
            "camera_frame_id": camera_frame_id,
            "tag_detection_topic": tag_detection_topic,
            "tag_pose_topic": tag_pose_topic,
            "tag_marker_topic": tag_marker_topic,
            "target_marker_id": target_marker_id,
            "tag_family": tag_family,
            "tag_size_m": tag_size_m,
        }.items(),
    )

    relative_position_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                relative_position_fusion_share,
                "launch",
                "relative_position_fusion.launch.py",
            )
        ),
        condition=IfCondition(enable_relative_position_fusion),
        launch_arguments={
            "preset": "",
            "use_sim_time": use_sim_time,
            "global_frame": global_frame,
            "uav_body_frame": base_frame_id,
            "uav_odom_topic": state_bridge_output_odometry_topic,
            "uav_twist_in_child_frame": "true",
            "enable_relative_tracking": enable_relative_tracking,
            "config_overlay": relative_position_fusion_config,
        }.items(),
    )

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge",
        output="screen",
        condition=IfCondition(start_uav_state_bridge),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"vehicle_local_position_topic": "/fmu/out/vehicle_local_position"},
            {"vehicle_odometry_topic": "/fmu/out/vehicle_odometry"},
            {"output_odometry_topic": state_bridge_output_odometry_topic},
            {"map_frame_id": uav_map_frame},
            {"odom_frame_id": uav_odom_frame},
            {"base_frame_id": base_frame_id},
            {"px4_timestamp_source": px4_timestamp_source},
            {"timesync_status_topic": timesync_status_topic},
            {"publish_tf": True},
            {"publish_map_to_odom_tf": True},
        ],
    )

    model_root_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uav_model_root_tf",
        output="screen",
        condition=IfCondition(publish_model),
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            base_frame_id,
            "--child-frame-id",
            "uav_model_base_link",
        ],
    )

    model_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="uav_model_state_publisher",
        output="screen",
        condition=IfCondition(publish_model),
        parameters=[
            {
                "robot_description": model_description,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("/robot_description", "/uav/model/robot_description")],
    )

    uav_control = Node(
        package="uav_bridge",
        executable="uav_control_node",
        name="uav_control",
        output="screen",
        parameters=[
            {"position_topic": position_topic},
            {"position_keep_yaw_topic": position_keep_yaw_topic},
            {"offboard_mode_topic": "/fmu/in/offboard_control_mode"},
            {"trajectory_setpoint_topic": "/fmu/in/trajectory_setpoint"},
            {"vehicle_command_topic": "/fmu/in/vehicle_command"},
            {"vehicle_status_topic": "/fmu/out/vehicle_status"},
            {"state_topic": control_state_topic},
            {"execution_state_topic": execution_state_topic},
            {"vehicle_local_position_topic": "/fmu/out/vehicle_local_position"},
            {"base_frame_id": base_frame_id},
            {"position_command_frame_id": position_command_frame_id},
            {"px4_timestamp_source": px4_timestamp_source},
            {"timesync_status_topic": timesync_status_topic},
            {"use_sim_time": use_sim_time},
            {"takeoff_height_m": takeoff_height_m},
            {"max_velocity_setpoint_mps": max_velocity_setpoint_mps},
            {"max_acceleration_setpoint_mps2": max_acceleration_setpoint_mps2},
            {"velocity_mode_max_acc_xy_mps2": max_acceleration_setpoint_mps2},
            {"velocity_mode_max_acc_z_mps2": max_acceleration_setpoint_mps2},
        ],
    )

    position_delta = Node(
        package="uav_bridge",
        executable="position_delta_node",
        name="position_delta",
        output="screen",
        parameters=[
            {"state_topic": control_state_topic},
            {"delta_service": position_delta_service},
            {"output_position_topic": position_keep_yaw_topic},
            {"odom_frame_id": position_command_frame_id},
        ],
    )

    trajectory_path_publisher = Node(
        package="uav_bridge",
        executable="trajectory_path_publisher_node",
        name="trajectory_path_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_odom_topic": control_state_topic},
            {"output_path_topic": trajectory_output_topic},
            {"max_samples": trajectory_max_samples},
            {"min_sample_distance_m": trajectory_min_sample_distance_m},
            {"min_sample_period_s": trajectory_min_sample_period_s},
        ],
    )

    rc_safety_mux = Node(
        package="uav_bridge",
        executable="rc_safety_mux_node",
        name="rc_safety_mux",
        output="screen",
        condition=IfCondition(start_rc_safety_mux),
        parameters=[rc_safety_config_path, {"use_sim_time": use_sim_time}],
    )

    demo_search_coordinator = Node(
        package="uav_bringup",
        executable="uav_demo_search_coordinator.py",
        name="uav_demo_search_coordinator",
        output="screen",
        condition=IfCondition(start_demo_search_coordinator),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"global_frame": global_frame},
            {"uav_local_frame": position_command_frame_id},
            {"uav_state_topic": control_state_topic},
            {"position_keep_yaw_topic": position_keep_yaw_topic},
            {"ugv_state_topic": demo_search_ugv_state_topic},
            {"relative_pose_global_topic": demo_search_relative_pose_global_topic},
            {"tag_detection_topic": tag_detection_topic},
            {"position_delta_service": position_delta_service},
            {"hold_service": demo_search_hold_service},
            {"ugv_go_relative_service": demo_search_ugv_go_relative_service},
            {"start_service": demo_search_start_service},
            {"stop_service": demo_search_stop_service},
            {"manual_waypoint_service": demo_search_manual_waypoint_service},
            {"auto_start": demo_search_auto_start},
            {"auto_start_delay_s": demo_search_auto_start_delay_s},
            {"waypoints_body": demo_search_waypoints_body},
            {"waypoint_reach_tolerance_m": demo_search_waypoint_reach_tolerance_m},
            {"waypoint_timeout_s": demo_search_waypoint_timeout_s},
            {"hold_on_finish": demo_search_hold_on_finish},
            {"hold_on_stop": demo_search_hold_on_stop},
            {"tag_confidence_threshold": demo_search_tag_confidence_threshold},
            {"uav_state_timeout_s": demo_search_uav_state_timeout_s},
            {"ugv_state_timeout_s": demo_search_ugv_state_timeout_s},
            {"relative_pose_timeout_s": demo_search_relative_pose_timeout_s},
            {"transform_timeout_s": demo_search_transform_timeout_s},
            {"service_wait_timeout_s": demo_search_service_wait_timeout_s},
            {"ugv_goal_offset_x_m": demo_search_ugv_goal_offset_x_m},
            {"ugv_goal_offset_y_m": demo_search_ugv_goal_offset_y_m},
        ],
    )

    bag_record_topics = [
        image_topic,
        # actual_ov_trackhist_topic,
        state_bridge_output_odometry_topic,
        trajectory_output_topic,
        # position_topic,
        # position_keep_yaw_topic,
        velocity_body_topic,
        tag_detection_topic,
        tag_pose_topic,
        tag_marker_topic,
        "/uav/model/robot_description",
        "/ugv/odom",
        "/ugv/odometry/filtered",
        "/relative_position/estimate/global",
        "/relative_position/estimate/uav_body",
        "/relative_position/debug/relative_velocity",
        "/relative_position/relocalize_requested",
        "/relative_position/diagnostics",
        "/tf",
        "/tf_static",
    ]

    record_bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--output", bag_output_dir, *bag_record_topics],
        output="screen",
        condition=IfCondition(use_record_bag),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("start_perception_stack", default_value="false"),
            DeclareLaunchArgument("start_orbbec_camera", default_value="false"),
            DeclareLaunchArgument("start_openvins", default_value="false"),
            DeclareLaunchArgument("start_px4_vision_bridge", default_value="false"),
            DeclareLaunchArgument("start_uav_state_bridge", default_value="true"),
            DeclareLaunchArgument("guard_enable", default_value="true"),
            DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("openvins_config_path", default_value=default_openvins_config),
            DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
            DeclareLaunchArgument("orbbec_camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument("camera_backend", default_value="legacy"),
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_fourcc", default_value="MJPG"),
            DeclareLaunchArgument("image_width", default_value="1280"),
            DeclareLaunchArgument("image_height", default_value="720"),
            DeclareLaunchArgument("camera_fps", default_value="120.0"),
            DeclareLaunchArgument("camera_name", default_value="uav_mono_camera"),
            DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="uav_ov_mono_camera_optical_frame",
            ),
            DeclareLaunchArgument("target_marker_id", default_value="0"),
            DeclareLaunchArgument("tag_family", default_value="36h11"),
            DeclareLaunchArgument("tag_size_m", default_value="0.20"),
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
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("ugv_map_frame", default_value="ugv_map"),
            DeclareLaunchArgument("uav_map_frame", default_value="uav_map"),
            DeclareLaunchArgument("uav_odom_frame", default_value="uav_odom"),
            DeclareLaunchArgument(
                "global_to_ugv_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to ugv_map_frame",
            ),
            DeclareLaunchArgument(
                "global_to_uav_map",
                default_value="0,0,0,0,0,0",
                description="Explicit global_frame -> uav_map_frame transform used when compute_uav_map_from_ugv_relative=false",
            ),
            DeclareLaunchArgument(
                "compute_uav_map_from_ugv_relative",
                default_value="true",
                description="Resolve global_to_uav_map from global_to_ugv_map plus uav_initial_relative_xy_to_ugv",
            ),
            DeclareLaunchArgument(
                "uav_initial_relative_xy_to_ugv",
                default_value="0.0,0.0",
                description="Initial UAV planar offset x,y expressed in ugv_map_frame before takeoff alignment",
            ),
            DeclareLaunchArgument("uav_map_z_offset", default_value="0.0"),
            DeclareLaunchArgument("uav_map_yaw_offset", default_value="0.0"),
            DeclareLaunchArgument("resolved_global_to_uav_map", default_value="0,0,0,0,0,0"),
            DeclareLaunchArgument("takeoff_height_m", default_value="0.70"),
            DeclareLaunchArgument("max_velocity_setpoint_mps", default_value="0.25"),
            DeclareLaunchArgument("max_acceleration_setpoint_mps2", default_value="0.35"),
            DeclareLaunchArgument(
                "control_state_topic", default_value="/uav/state/odometry_px4"
            ),
            DeclareLaunchArgument(
                "execution_state_topic", default_value="/uav/state/odometry_px4"
            ),
            DeclareLaunchArgument("position_topic", default_value="/uav/control/position_yaw"),
            DeclareLaunchArgument(
                "position_keep_yaw_topic", default_value="/uav/control/position_keep_yaw"
            ),
            DeclareLaunchArgument(
                "velocity_body_topic",
                default_value="/uav/control/setpoint/velocity_body",
            ),
            DeclareLaunchArgument(
                "position_delta_service",
                default_value="/uav/control/command/position_delta",
            ),
            DeclareLaunchArgument(
                "position_command_frame_id", default_value="uav_odom"
            ),
            DeclareLaunchArgument(
                "px4_timestamp_source", default_value="px4_timesync"
            ),
            DeclareLaunchArgument(
                "timesync_status_topic",
                default_value="/fmu/out/timesync_status",
            ),
            DeclareLaunchArgument(
                "state_bridge_output_odometry_topic",
                default_value="/uav/state/odometry_px4",
            ),
            DeclareLaunchArgument(
                "trajectory_output_topic", default_value="/uav/state/trajectory"
            ),
            DeclareLaunchArgument("trajectory_max_samples", default_value="5000"),
            DeclareLaunchArgument(
                "trajectory_min_sample_distance_m", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "trajectory_min_sample_period_s", default_value="0.10"
            ),
            DeclareLaunchArgument("start_rc_safety_mux", default_value="true"),
            DeclareLaunchArgument(
                "rc_safety_config_path", default_value=default_rc_safety_config
            ),
            DeclareLaunchArgument(
                "start_demo_search_coordinator", default_value="true"
            ),
            DeclareLaunchArgument("demo_search_auto_start", default_value="false"),
            DeclareLaunchArgument(
                "demo_search_auto_start_delay_s", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "demo_search_waypoints_body",
                default_value="",
                description="Semicolon separated body-frame waypoint deltas x,y[,z], for example '1.0,0.0; 0.0,1.0'",
            ),
            DeclareLaunchArgument(
                "demo_search_waypoint_reach_tolerance_m", default_value="0.12"
            ),
            DeclareLaunchArgument(
                "demo_search_waypoint_timeout_s", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "demo_search_hold_on_finish", default_value="true"
            ),
            DeclareLaunchArgument("demo_search_hold_on_stop", default_value="true"),
            DeclareLaunchArgument(
                "demo_search_start_service",
                default_value="/uav/demo_search/command/start",
            ),
            DeclareLaunchArgument(
                "demo_search_stop_service",
                default_value="/uav/demo_search/command/stop",
            ),
            DeclareLaunchArgument(
                "demo_search_manual_waypoint_service",
                default_value="/uav/demo_search/command/waypoint_delta",
            ),
            DeclareLaunchArgument(
                "demo_search_hold_service",
                default_value="/uav/control/command/hold",
            ),
            DeclareLaunchArgument(
                "demo_search_ugv_state_topic",
                default_value="/ugv/odometry/filtered",
            ),
            DeclareLaunchArgument(
                "demo_search_ugv_go_relative_service",
                default_value="/ugv/navigation/go_relative_xy",
            ),
            DeclareLaunchArgument(
                "demo_search_relative_pose_global_topic",
                default_value="/relative_position/estimate/global",
            ),
            DeclareLaunchArgument(
                "demo_search_tag_confidence_threshold", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "demo_search_uav_state_timeout_s", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "demo_search_ugv_state_timeout_s", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "demo_search_relative_pose_timeout_s", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "demo_search_transform_timeout_s", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "demo_search_service_wait_timeout_s", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "demo_search_ugv_goal_offset_x_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "demo_search_ugv_goal_offset_y_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "enable_relative_position_fusion", default_value="true"
            ),
            DeclareLaunchArgument("enable_relative_tracking", default_value="false"),
            DeclareLaunchArgument(
                "relative_position_fusion_config",
                default_value=default_relative_fusion_config,
            ),
            DeclareLaunchArgument("publish_model", default_value="true"),
            DeclareLaunchArgument("use_record_bag", default_value="true"),
            DeclareLaunchArgument(
                "bag_output_dir", default_value=default_bag_output_dir
            ),
            resolve_global_to_uav_map,
            global_to_uav_map_static_tf,
            perception_launch,
            relative_position_fusion_launch,
            uav_state_bridge,
            model_root_tf,
            model_publisher,
            uav_control,
            position_delta,
            rc_safety_mux,
            demo_search_coordinator,
            trajectory_path_publisher,
            record_bag,
        ]
    )
