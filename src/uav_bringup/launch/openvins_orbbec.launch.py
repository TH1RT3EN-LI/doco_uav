import math
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from uav_bringup.launch_utils import namespaced_path
from uav_bringup.profile_defaults import (
    DEFAULT_ORBBEC_IR_EXPOSURE,
    DEFAULT_ORBBEC_IR_STREAM,
    DEFAULT_ORBBEC_STANDALONE_PROFILE,
    DEFAULT_STEREO_CAMERA_TO_BODY,
)


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


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    start_orbbec_camera = LaunchConfiguration("start_orbbec_camera")
    start_openvins = LaunchConfiguration("start_openvins")
    start_px4_vision_bridge = LaunchConfiguration("start_px4_vision_bridge")
    start_uav_state_bridge = LaunchConfiguration("start_uav_state_bridge")
    use_rviz = LaunchConfiguration("use_rviz")
    use_record_bag = LaunchConfiguration("use_record_bag")
    bag_output_dir = LaunchConfiguration("bag_output_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")

    openvins_namespace = LaunchConfiguration("openvins_namespace")
    openvins_config_path = LaunchConfiguration("openvins_config_path")
    openvins_verbosity = LaunchConfiguration("openvins_verbosity")
    openvins_use_stereo = LaunchConfiguration("openvins_use_stereo")
    openvins_use_mask = LaunchConfiguration("openvins_use_mask")
    openvins_max_cameras = LaunchConfiguration("openvins_max_cameras")
    openvins_save_total_state = LaunchConfiguration("openvins_save_total_state")
    openvins_yaml_left_ir_topic = LaunchConfiguration("openvins_yaml_left_ir_topic")
    openvins_yaml_right_ir_topic = LaunchConfiguration("openvins_yaml_right_ir_topic")
    openvins_yaml_imu_topic = LaunchConfiguration("openvins_yaml_imu_topic")
    px4_visual_odometry_topic = LaunchConfiguration("px4_visual_odometry_topic")
    px4_bridge_expected_odom_frame_id = LaunchConfiguration(
        "px4_bridge_expected_odom_frame_id"
    )
    px4_bridge_expected_child_frame_id = LaunchConfiguration(
        "px4_bridge_expected_child_frame_id"
    )
    px4_bridge_sensor_x_in_body_m = LaunchConfiguration("px4_bridge_sensor_x_in_body_m")
    px4_bridge_sensor_y_in_body_m = LaunchConfiguration("px4_bridge_sensor_y_in_body_m")
    px4_bridge_sensor_z_in_body_m = LaunchConfiguration("px4_bridge_sensor_z_in_body_m")
    px4_bridge_sensor_roll_in_body_rad = LaunchConfiguration(
        "px4_bridge_sensor_roll_in_body_rad"
    )
    px4_bridge_sensor_pitch_in_body_rad = LaunchConfiguration(
        "px4_bridge_sensor_pitch_in_body_rad"
    )
    px4_bridge_sensor_yaw_in_body_rad = LaunchConfiguration(
        "px4_bridge_sensor_yaw_in_body_rad"
    )
    px4_bridge_timestamp_source = LaunchConfiguration("px4_bridge_timestamp_source")
    px4_bridge_timesync_status_topic = LaunchConfiguration(
        "px4_bridge_timesync_status_topic"
    )
    px4_bridge_timesync_timeout_s = LaunchConfiguration("px4_bridge_timesync_timeout_s")
    px4_bridge_log_debug = LaunchConfiguration("px4_bridge_log_debug")
    state_bridge_vehicle_local_position_topic = LaunchConfiguration(
        "state_bridge_vehicle_local_position_topic"
    )
    state_bridge_vehicle_odometry_topic = LaunchConfiguration(
        "state_bridge_vehicle_odometry_topic"
    )
    state_bridge_output_odometry_topic = LaunchConfiguration(
        "state_bridge_output_odometry_topic"
    )
    state_bridge_timestamp_source = LaunchConfiguration("state_bridge_timestamp_source")
    state_bridge_timesync_status_topic = LaunchConfiguration(
        "state_bridge_timesync_status_topic"
    )
    state_bridge_timesync_timeout_s = LaunchConfiguration("state_bridge_timesync_timeout_s")
    state_bridge_publish_tf = LaunchConfiguration("state_bridge_publish_tf")
    state_bridge_publish_map_to_odom_tf = LaunchConfiguration(
        "state_bridge_publish_map_to_odom_tf"
    )
    state_bridge_log_state = LaunchConfiguration("state_bridge_log_state")

    orbbec_camera_name = LaunchConfiguration("orbbec_camera_name")
    base_frame_id = LaunchConfiguration("base_frame_id")
    orbbec_camera_frame_id = LaunchConfiguration("orbbec_camera_frame_id")
    orbbec_camera_x = LaunchConfiguration("orbbec_camera_x")
    orbbec_camera_y = LaunchConfiguration("orbbec_camera_y")
    orbbec_camera_z = LaunchConfiguration("orbbec_camera_z")
    orbbec_camera_roll = LaunchConfiguration("orbbec_camera_roll")
    orbbec_camera_pitch = LaunchConfiguration("orbbec_camera_pitch")
    orbbec_camera_yaw = LaunchConfiguration("orbbec_camera_yaw")
    orbbec_publish_mount_tf = LaunchConfiguration("orbbec_publish_mount_tf")
    orbbec_enable_depth = LaunchConfiguration("orbbec_enable_depth")
    orbbec_enable_color = LaunchConfiguration("orbbec_enable_color")
    orbbec_enable_left_ir = LaunchConfiguration("orbbec_enable_left_ir")
    orbbec_enable_right_ir = LaunchConfiguration("orbbec_enable_right_ir")
    orbbec_enable_point_cloud = LaunchConfiguration("orbbec_enable_point_cloud")
    orbbec_enable_colored_point_cloud = LaunchConfiguration("orbbec_enable_colored_point_cloud")
    orbbec_enable_sync_output_accel_gyro = LaunchConfiguration("orbbec_enable_sync_output_accel_gyro")
    orbbec_enable_publish_extrinsic = LaunchConfiguration("orbbec_enable_publish_extrinsic")
    orbbec_enable_accel = LaunchConfiguration("orbbec_enable_accel")
    orbbec_enable_gyro = LaunchConfiguration("orbbec_enable_gyro")
    orbbec_accel_rate = LaunchConfiguration("orbbec_accel_rate")
    orbbec_gyro_rate = LaunchConfiguration("orbbec_gyro_rate")
    orbbec_left_ir_width = LaunchConfiguration("orbbec_left_ir_width")
    orbbec_left_ir_height = LaunchConfiguration("orbbec_left_ir_height")
    orbbec_left_ir_fps = LaunchConfiguration("orbbec_left_ir_fps")
    orbbec_left_ir_format = LaunchConfiguration("orbbec_left_ir_format")
    orbbec_right_ir_width = LaunchConfiguration("orbbec_right_ir_width")
    orbbec_right_ir_height = LaunchConfiguration("orbbec_right_ir_height")
    orbbec_right_ir_fps = LaunchConfiguration("orbbec_right_ir_fps")
    orbbec_right_ir_format = LaunchConfiguration("orbbec_right_ir_format")
    orbbec_enable_ir_auto_exposure = LaunchConfiguration("orbbec_enable_ir_auto_exposure")
    orbbec_ir_exposure = LaunchConfiguration("orbbec_ir_exposure")
    orbbec_ir_gain = LaunchConfiguration("orbbec_ir_gain")
    orbbec_ir_ae_max_exposure = LaunchConfiguration("orbbec_ir_ae_max_exposure")
    orbbec_ir_brightness = LaunchConfiguration("orbbec_ir_brightness")
    orbbec_enable_laser = LaunchConfiguration("orbbec_enable_laser")
    orbbec_enable_ldp = LaunchConfiguration("orbbec_enable_ldp")

    rviz_config = LaunchConfiguration("rviz_config")

    openvins_default_odom_topic = "/ov_msckf/odomimu"
    openvins_default_path_topic = "/ov_msckf/pathimu"
    openvins_default_trackhist_topic = "/ov_msckf/trackhist"
    openvins_default_loop_depth_topic = "/ov_msckf/loop_depth_colored"
    openvins_default_msckf_points_topic = "/ov_msckf/points_msckf"
    openvins_default_slam_points_topic = "/ov_msckf/points_slam"
    openvins_default_aruco_points_topic = "/ov_msckf/points_aruco"
    openvins_default_loop_feats_topic = "/ov_msckf/loop_feats"

    actual_ov_odom_topic = namespaced_path(openvins_namespace, "odomimu")
    actual_ov_path_topic = namespaced_path(openvins_namespace, "pathimu")
    actual_ov_trackhist_topic = namespaced_path(openvins_namespace, "trackhist")
    actual_ov_loop_depth_topic = namespaced_path(openvins_namespace, "loop_depth_colored")
    actual_ov_msckf_points_topic = namespaced_path(openvins_namespace, "points_msckf")
    actual_ov_slam_points_topic = namespaced_path(openvins_namespace, "points_slam")
    actual_ov_aruco_points_topic = namespaced_path(openvins_namespace, "points_aruco")
    actual_ov_loop_feats_topic = namespaced_path(openvins_namespace, "loop_feats")

    default_left_ir_topic = "/uav_depth_camera/left_ir/image_raw"
    default_right_ir_topic = "/uav_depth_camera/right_ir/image_raw"
    default_imu_topic = "/uav_depth_camera/gyro_accel/sample"

    actual_left_ir_topic = namespaced_path(orbbec_camera_name, "left_ir/image_raw")
    actual_right_ir_topic = namespaced_path(orbbec_camera_name, "right_ir/image_raw")
    actual_imu_topic = namespaced_path(orbbec_camera_name, "gyro_accel/sample")
    bag_record_topics = [
        actual_ov_odom_topic,
        "/tf",
        "/tf_static",
    ]

    orbbec_depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "orbbec_depth_camera.launch.py")
        ),
        condition=IfCondition(start_orbbec_camera),
        launch_arguments={
            "start_camera": start_orbbec_camera,
            "camera_name": orbbec_camera_name,
            "base_frame_id": base_frame_id,
            "camera_frame_id": orbbec_camera_frame_id,
            "camera_x": orbbec_camera_x,
            "camera_y": orbbec_camera_y,
            "camera_z": orbbec_camera_z,
            "camera_roll": orbbec_camera_roll,
            "camera_pitch": orbbec_camera_pitch,
            "camera_yaw": orbbec_camera_yaw,
            "publish_mount_tf": orbbec_publish_mount_tf,
            "enable_depth": orbbec_enable_depth,
            "enable_color": orbbec_enable_color,
            "enable_left_ir": orbbec_enable_left_ir,
            "enable_right_ir": orbbec_enable_right_ir,
            "enable_point_cloud": orbbec_enable_point_cloud,
            "enable_colored_point_cloud": orbbec_enable_colored_point_cloud,
            "enable_sync_output_accel_gyro": orbbec_enable_sync_output_accel_gyro,
            "enable_publish_extrinsic": orbbec_enable_publish_extrinsic,
            "enable_accel": orbbec_enable_accel,
            "enable_gyro": orbbec_enable_gyro,
            "accel_rate": orbbec_accel_rate,
            "gyro_rate": orbbec_gyro_rate,
            "left_ir_width": orbbec_left_ir_width,
            "left_ir_height": orbbec_left_ir_height,
            "left_ir_fps": orbbec_left_ir_fps,
            "left_ir_format": orbbec_left_ir_format,
            "right_ir_width": orbbec_right_ir_width,
            "right_ir_height": orbbec_right_ir_height,
            "right_ir_fps": orbbec_right_ir_fps,
            "right_ir_format": orbbec_right_ir_format,
            "enable_ir_auto_exposure": orbbec_enable_ir_auto_exposure,
            "ir_exposure": orbbec_ir_exposure,
            "ir_gain": orbbec_ir_gain,
            "ir_ae_max_exposure": orbbec_ir_ae_max_exposure,
            "ir_brightness": orbbec_ir_brightness,
            "enable_laser": orbbec_enable_laser,
            "enable_ldp": orbbec_enable_ldp,
        }.items(),
    )

    openvins = GroupAction(
        condition=IfCondition(start_openvins),
        actions=[
            SetRemap(src=openvins_yaml_left_ir_topic, dst=actual_left_ir_topic),
            SetRemap(src=openvins_yaml_right_ir_topic, dst=actual_right_ir_topic),
            SetRemap(src=openvins_yaml_imu_topic, dst=actual_imu_topic),
            Node(
                package="ov_msckf",
                executable="run_subscribe_msckf",
                namespace=openvins_namespace,
                output="screen",
                parameters=[
                    {"verbosity": openvins_verbosity},
                    {"use_stereo": openvins_use_stereo},
                    {"use_mask": openvins_use_mask},
                    {"max_cameras": openvins_max_cameras},
                    {"save_total_state": openvins_save_total_state},
                    {"config_path": openvins_config_path},
                ],
            ),
        ],
    )

    openvins_px4_vision_bridge = Node(
        package="uav_bridge",
        executable="openvins_px4_vision_bridge_node",
        name="openvins_px4_vision_bridge",
        output="screen",
        condition=IfCondition(start_px4_vision_bridge),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"odometry_topic": actual_ov_odom_topic},
            {"visual_odometry_topic": px4_visual_odometry_topic},
            {"expected_odom_frame_id": px4_bridge_expected_odom_frame_id},
            {"expected_child_frame_id": px4_bridge_expected_child_frame_id},
            {"sensor_x_in_body_m": px4_bridge_sensor_x_in_body_m},
            {"sensor_y_in_body_m": px4_bridge_sensor_y_in_body_m},
            {"sensor_z_in_body_m": px4_bridge_sensor_z_in_body_m},
            {"sensor_roll_in_body_rad": px4_bridge_sensor_roll_in_body_rad},
            {"sensor_pitch_in_body_rad": px4_bridge_sensor_pitch_in_body_rad},
            {"sensor_yaw_in_body_rad": px4_bridge_sensor_yaw_in_body_rad},
            {"px4_timestamp_source": px4_bridge_timestamp_source},
            {"timesync_status_topic": px4_bridge_timesync_status_topic},
            {"timesync_timeout_s": px4_bridge_timesync_timeout_s},
            {"log_debug": px4_bridge_log_debug},
            {"health_ok_topic": "/uav/ov/bridge/health_ok"},
            {"fault_reason_topic": "/uav/ov/bridge/fault_reason"},
            {"guard_enable": True},
            {"guard_auto_recover": False},
            {"guard_nominal_rate_hz": 30.0},
            {"guard_hold_last_budget_s": 0.12},
            {"guard_recovery_good_frames": 5},
            {"guard_max_source_gap_s": 0.20},
            {"guard_max_position_step_m": 0.20},
            {"guard_max_implied_speed_mps": 1.0},
            {"guard_max_reported_speed_mps": 1.0},
            {"guard_max_accel_mps2": 2.5},
            {"guard_max_yaw_rate_radps": 2.0},
        ],
    )

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge",
        output="screen",
        condition=IfCondition(start_uav_state_bridge),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"vehicle_local_position_topic": state_bridge_vehicle_local_position_topic},
            {"vehicle_odometry_topic": state_bridge_vehicle_odometry_topic},
            {"output_odometry_topic": state_bridge_output_odometry_topic},
            {"base_frame_id": base_frame_id},
            {"px4_timestamp_source": state_bridge_timestamp_source},
            {"timesync_status_topic": state_bridge_timesync_status_topic},
            {"timesync_timeout_s": state_bridge_timesync_timeout_s},
            {"publish_tf": state_bridge_publish_tf},
            {"publish_map_to_odom_tf": state_bridge_publish_map_to_odom_tf},
            {"log_state": state_bridge_log_state},
        ],
    )

    record_bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag_output_dir,
            *bag_record_topics,
        ],
        output="screen",
        condition=IfCondition(use_record_bag),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="openvins_rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            (openvins_default_odom_topic, actual_ov_odom_topic),
            (openvins_default_path_topic, actual_ov_path_topic),
            (openvins_default_trackhist_topic, actual_ov_trackhist_topic),
            (openvins_default_loop_depth_topic, actual_ov_loop_depth_topic),
            (openvins_default_msckf_points_topic, actual_ov_msckf_points_topic),
            (openvins_default_slam_points_topic, actual_ov_slam_points_topic),
            (openvins_default_aruco_points_topic, actual_ov_aruco_points_topic),
            (openvins_default_loop_feats_topic, actual_ov_loop_feats_topic),
            (default_left_ir_topic, actual_left_ir_topic),
            (default_right_ir_topic, actual_right_ir_topic),
        ],
    )

    default_openvins_config = os.path.join(
        bringup_share, "config", "openvins", "orbbec_gemini336", "estimator_config.yaml"
    )
    default_rviz_config = os.path.join(
        bringup_share, "config", "rviz", "openvins_orbbec.rviz"
    )
    bag_root_dir = Path.home() / "uav_bags"
    default_bag_id = next_bag_run_id(bag_root_dir, "openvins_orbbec")
    default_bag_output_dir = str(
        bag_root_dir
        / f"openvins_orbbec_{default_bag_id:03d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_orbbec_camera", default_value="true"),
            DeclareLaunchArgument("start_openvins", default_value="true"),
            DeclareLaunchArgument("start_px4_vision_bridge", default_value="true"),
            DeclareLaunchArgument("start_uav_state_bridge", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_record_bag", default_value="false"),
            DeclareLaunchArgument("bag_output_dir", default_value=default_bag_output_dir),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("openvins_config_path", default_value=default_openvins_config),
            DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
            DeclareLaunchArgument("openvins_use_stereo", default_value="true"),
            DeclareLaunchArgument("openvins_use_mask", default_value="true"),
            DeclareLaunchArgument("openvins_max_cameras", default_value="2"),
            DeclareLaunchArgument("openvins_save_total_state", default_value="false"),
            DeclareLaunchArgument(
                "openvins_yaml_left_ir_topic", default_value=default_left_ir_topic
            ),
            DeclareLaunchArgument(
                "openvins_yaml_right_ir_topic", default_value=default_right_ir_topic
            ),
            DeclareLaunchArgument(
                "openvins_yaml_imu_topic", default_value=default_imu_topic
            ),
            DeclareLaunchArgument(
                "px4_visual_odometry_topic",
                default_value="/fmu/in/vehicle_visual_odometry",
            ),
            DeclareLaunchArgument(
                "px4_bridge_expected_odom_frame_id", default_value="global"
            ),
            DeclareLaunchArgument(
                "px4_bridge_expected_child_frame_id", default_value="imu"
            ),
            DeclareLaunchArgument(
                "px4_bridge_sensor_x_in_body_m",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "px4_bridge_sensor_y_in_body_m",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "px4_bridge_sensor_z_in_body_m",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "px4_bridge_sensor_roll_in_body_rad", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "px4_bridge_sensor_pitch_in_body_rad", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "px4_bridge_sensor_yaw_in_body_rad", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "px4_bridge_timestamp_source", default_value="px4_timesync"
            ),
            DeclareLaunchArgument(
                "px4_bridge_timesync_status_topic",
                default_value="/fmu/out/timesync_status",
            ),
            DeclareLaunchArgument(
                "px4_bridge_timesync_timeout_s", default_value="2.0"
            ),
            DeclareLaunchArgument("px4_bridge_log_debug", default_value="false"),
            DeclareLaunchArgument(
                "state_bridge_vehicle_local_position_topic",
                default_value="/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "state_bridge_vehicle_odometry_topic",
                default_value="/fmu/out/vehicle_odometry",
            ),
            DeclareLaunchArgument(
                "state_bridge_output_odometry_topic",
                default_value="/uav/state/odometry_px4",
            ),
            DeclareLaunchArgument(
                "state_bridge_timestamp_source", default_value="px4_timesync"
            ),
            DeclareLaunchArgument(
                "state_bridge_timesync_status_topic",
                default_value="/fmu/out/timesync_status",
            ),
            DeclareLaunchArgument(
                "state_bridge_timesync_timeout_s", default_value="2.0"
            ),
            DeclareLaunchArgument("state_bridge_publish_tf", default_value="false"),
            DeclareLaunchArgument(
                "state_bridge_publish_map_to_odom_tf", default_value="false"
            ),
            DeclareLaunchArgument("state_bridge_log_state", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("orbbec_camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument(
                "orbbec_camera_frame_id",
                default_value="uav_stereo_camera_optical_frame",
            ),
            DeclareLaunchArgument(
                "orbbec_camera_x", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["x"]
            ),
            DeclareLaunchArgument(
                "orbbec_camera_y", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["y"]
            ),
            DeclareLaunchArgument(
                "orbbec_camera_z", default_value=DEFAULT_STEREO_CAMERA_TO_BODY["z"]
            ),
            DeclareLaunchArgument(
                "orbbec_camera_roll", default_value=str(-math.pi / 2.0)
            ),
            DeclareLaunchArgument("orbbec_camera_pitch", default_value="0.0"),
            DeclareLaunchArgument(
                "orbbec_camera_yaw", default_value=str(-math.pi / 2.0)
            ),
            DeclareLaunchArgument("orbbec_publish_mount_tf", default_value="true"),
            DeclareLaunchArgument("orbbec_enable_depth", default_value="false"),
            DeclareLaunchArgument("orbbec_enable_color", default_value="false"),
            DeclareLaunchArgument("orbbec_enable_left_ir", default_value="true"),
            DeclareLaunchArgument("orbbec_enable_right_ir", default_value="true"),
            DeclareLaunchArgument("orbbec_enable_point_cloud", default_value="false"),
            DeclareLaunchArgument(
                "orbbec_enable_colored_point_cloud", default_value="false"
            ),
            DeclareLaunchArgument(
                "orbbec_enable_sync_output_accel_gyro", default_value="true"
            ),
            DeclareLaunchArgument(
                "orbbec_enable_publish_extrinsic",
                default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_publish_extrinsic"],
            ),
            DeclareLaunchArgument("orbbec_enable_accel", default_value="true"),
            DeclareLaunchArgument("orbbec_enable_gyro", default_value="true"),
            DeclareLaunchArgument(
                "orbbec_accel_rate",
                default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["accel_rate"],
            ),
            DeclareLaunchArgument(
                "orbbec_gyro_rate",
                default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["gyro_rate"],
            ),
            DeclareLaunchArgument(
                "orbbec_left_ir_width", default_value=DEFAULT_ORBBEC_IR_STREAM["width"]
            ),
            DeclareLaunchArgument(
                "orbbec_left_ir_height", default_value=DEFAULT_ORBBEC_IR_STREAM["height"]
            ),
            DeclareLaunchArgument(
                "orbbec_left_ir_fps", default_value=DEFAULT_ORBBEC_IR_STREAM["fps"]
            ),
            DeclareLaunchArgument(
                "orbbec_left_ir_format", default_value=DEFAULT_ORBBEC_IR_STREAM["format"]
            ),
            DeclareLaunchArgument(
                "orbbec_right_ir_width", default_value=DEFAULT_ORBBEC_IR_STREAM["width"]
            ),
            DeclareLaunchArgument(
                "orbbec_right_ir_height", default_value=DEFAULT_ORBBEC_IR_STREAM["height"]
            ),
            DeclareLaunchArgument(
                "orbbec_right_ir_fps", default_value=DEFAULT_ORBBEC_IR_STREAM["fps"]
            ),
            DeclareLaunchArgument(
                "orbbec_right_ir_format", default_value=DEFAULT_ORBBEC_IR_STREAM["format"]
            ),
            DeclareLaunchArgument(
                "orbbec_enable_ir_auto_exposure",
                default_value=DEFAULT_ORBBEC_IR_EXPOSURE["enable_auto_exposure"],
            ),
            DeclareLaunchArgument(
                "orbbec_ir_exposure", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["exposure"]
            ),
            DeclareLaunchArgument(
                "orbbec_ir_gain", default_value=DEFAULT_ORBBEC_IR_EXPOSURE["gain"]
            ),
            DeclareLaunchArgument(
                "orbbec_ir_ae_max_exposure",
                default_value=DEFAULT_ORBBEC_IR_EXPOSURE["ae_max_exposure"],
            ),
            DeclareLaunchArgument(
                "orbbec_ir_brightness",
                default_value=DEFAULT_ORBBEC_IR_EXPOSURE["brightness"],
            ),
            DeclareLaunchArgument(
                "orbbec_enable_laser",
                default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_laser"],
            ),
            DeclareLaunchArgument(
                "orbbec_enable_ldp",
                default_value=DEFAULT_ORBBEC_STANDALONE_PROFILE["enable_ldp"],
            ),
            orbbec_depth_camera,
            openvins,
            openvins_px4_vision_bridge,
            uav_state_bridge,
            record_bag,
            rviz,
        ]
    )
