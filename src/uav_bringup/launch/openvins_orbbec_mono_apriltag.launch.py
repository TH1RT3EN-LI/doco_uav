import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_bringup.launch_utils import namespaced_path
from uav_bringup.profile_defaults import DEFAULT_CAMERA_INTRINSICS, DEFAULT_CAMERA_TO_BODY


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    start_orbbec_camera = LaunchConfiguration("start_orbbec_camera")
    start_openvins = LaunchConfiguration("start_openvins")
    start_px4_vision_bridge = LaunchConfiguration("start_px4_vision_bridge")
    start_uav_state_bridge = LaunchConfiguration("start_uav_state_bridge")
    enable_relative_position_fusion = LaunchConfiguration("enable_relative_position_fusion")
    enable_relative_tracking = LaunchConfiguration("enable_relative_tracking")
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
    px4_bridge_expected_odom_frame_id = LaunchConfiguration("px4_bridge_expected_odom_frame_id")
    px4_bridge_expected_child_frame_id = LaunchConfiguration("px4_bridge_expected_child_frame_id")
    px4_bridge_sensor_x_in_body_m = LaunchConfiguration("px4_bridge_sensor_x_in_body_m")
    px4_bridge_sensor_y_in_body_m = LaunchConfiguration("px4_bridge_sensor_y_in_body_m")
    px4_bridge_sensor_z_in_body_m = LaunchConfiguration("px4_bridge_sensor_z_in_body_m")
    px4_bridge_sensor_roll_in_body_rad = LaunchConfiguration("px4_bridge_sensor_roll_in_body_rad")
    px4_bridge_sensor_pitch_in_body_rad = LaunchConfiguration("px4_bridge_sensor_pitch_in_body_rad")
    px4_bridge_sensor_yaw_in_body_rad = LaunchConfiguration("px4_bridge_sensor_yaw_in_body_rad")
    px4_bridge_timestamp_source = LaunchConfiguration("px4_bridge_timestamp_source")
    px4_bridge_timesync_status_topic = LaunchConfiguration("px4_bridge_timesync_status_topic")
    px4_bridge_timesync_timeout_s = LaunchConfiguration("px4_bridge_timesync_timeout_s")
    px4_bridge_log_debug = LaunchConfiguration("px4_bridge_log_debug")

    state_bridge_vehicle_local_position_topic = LaunchConfiguration(
        "state_bridge_vehicle_local_position_topic"
    )
    state_bridge_vehicle_odometry_topic = LaunchConfiguration("state_bridge_vehicle_odometry_topic")
    state_bridge_output_odometry_topic = LaunchConfiguration("state_bridge_output_odometry_topic")
    state_bridge_timestamp_source = LaunchConfiguration("state_bridge_timestamp_source")
    state_bridge_timesync_status_topic = LaunchConfiguration("state_bridge_timesync_status_topic")
    state_bridge_timesync_timeout_s = LaunchConfiguration("state_bridge_timesync_timeout_s")
    state_bridge_publish_tf = LaunchConfiguration("state_bridge_publish_tf")
    state_bridge_publish_map_to_odom_tf = LaunchConfiguration("state_bridge_publish_map_to_odom_tf")
    state_bridge_log_state = LaunchConfiguration("state_bridge_log_state")
    relative_position_fusion_config = LaunchConfiguration("relative_position_fusion_config")
    global_frame = LaunchConfiguration("global_frame")
    uav_map_frame = LaunchConfiguration("uav_map_frame")
    uav_odom_frame = LaunchConfiguration("uav_odom_frame")
    global_to_uav_map = LaunchConfiguration("global_to_uav_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")

    orbbec_camera_name = LaunchConfiguration("orbbec_camera_name")
    base_frame_id = LaunchConfiguration("base_frame_id")
    ov_body_frame_id = LaunchConfiguration("ov_body_frame_id")
    publish_ov_body_tf = LaunchConfiguration("publish_ov_body_tf")
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

    backend = LaunchConfiguration("backend")
    device = LaunchConfiguration("device")
    fourcc = LaunchConfiguration("fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    fps = LaunchConfiguration("fps")
    camera_name = LaunchConfiguration("camera_name")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
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

    tag_detection_topic = LaunchConfiguration("tag_detection_topic")
    tag_pose_topic = LaunchConfiguration("tag_pose_topic")
    tag_marker_topic = LaunchConfiguration("tag_marker_topic")
    tag_output_frame_mode = LaunchConfiguration("tag_output_frame_mode")
    target_marker_id = LaunchConfiguration("target_marker_id")
    tag_family = LaunchConfiguration("tag_family")
    tag_size_m = LaunchConfiguration("tag_size_m")
    tag_camera_roll_offset_rad = LaunchConfiguration("tag_camera_roll_offset_rad")
    tag_frame_yaw_offset_rad = LaunchConfiguration("tag_frame_yaw_offset_rad")
    mono_dx = LaunchConfiguration("mono_dx")
    mono_dy = LaunchConfiguration("mono_dy")
    mono_dz = LaunchConfiguration("mono_dz")
    mono_droll = LaunchConfiguration("mono_droll")
    mono_dpitch = LaunchConfiguration("mono_dpitch")
    mono_dyaw = LaunchConfiguration("mono_dyaw")
    odometry_timeout_s = LaunchConfiguration("odometry_timeout_s")
    publish_tag_base_tf = LaunchConfiguration("publish_tag_base_tf")
    publish_tag_odom_tf = LaunchConfiguration("publish_tag_odom_tf")
    tag_tf_frame_prefix = LaunchConfiguration("tag_tf_frame_prefix")

    actual_ov_odom_topic = namespaced_path(openvins_namespace, "odomimu")
    actual_ov_path_topic = namespaced_path(openvins_namespace, "pathimu")

    openvins_orbbec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "openvins_orbbec.launch.py")
        ),
        launch_arguments={
            "start_orbbec_camera": start_orbbec_camera,
            "start_openvins": start_openvins,
            "start_px4_vision_bridge": start_px4_vision_bridge,
            "start_uav_state_bridge": start_uav_state_bridge,
            "enable_relative_position_fusion": enable_relative_position_fusion,
            "enable_relative_tracking": enable_relative_tracking,
            "use_rviz": "false",
            "use_record_bag": "false",
            "use_sim_time": use_sim_time,
            "openvins_namespace": openvins_namespace,
            "openvins_config_path": openvins_config_path,
            "openvins_verbosity": openvins_verbosity,
            "openvins_use_stereo": openvins_use_stereo,
            "openvins_use_mask": openvins_use_mask,
            "openvins_max_cameras": openvins_max_cameras,
            "openvins_save_total_state": openvins_save_total_state,
            "openvins_yaml_left_ir_topic": openvins_yaml_left_ir_topic,
            "openvins_yaml_right_ir_topic": openvins_yaml_right_ir_topic,
            "openvins_yaml_imu_topic": openvins_yaml_imu_topic,
            "px4_visual_odometry_topic": px4_visual_odometry_topic,
            "px4_bridge_expected_odom_frame_id": px4_bridge_expected_odom_frame_id,
            "px4_bridge_expected_child_frame_id": px4_bridge_expected_child_frame_id,
            "px4_bridge_sensor_x_in_body_m": px4_bridge_sensor_x_in_body_m,
            "px4_bridge_sensor_y_in_body_m": px4_bridge_sensor_y_in_body_m,
            "px4_bridge_sensor_z_in_body_m": px4_bridge_sensor_z_in_body_m,
            "px4_bridge_sensor_roll_in_body_rad": px4_bridge_sensor_roll_in_body_rad,
            "px4_bridge_sensor_pitch_in_body_rad": px4_bridge_sensor_pitch_in_body_rad,
            "px4_bridge_sensor_yaw_in_body_rad": px4_bridge_sensor_yaw_in_body_rad,
            "px4_bridge_timestamp_source": px4_bridge_timestamp_source,
            "px4_bridge_timesync_status_topic": px4_bridge_timesync_status_topic,
            "px4_bridge_timesync_timeout_s": px4_bridge_timesync_timeout_s,
            "px4_bridge_log_debug": px4_bridge_log_debug,
            "state_bridge_vehicle_local_position_topic": state_bridge_vehicle_local_position_topic,
            "state_bridge_vehicle_odometry_topic": state_bridge_vehicle_odometry_topic,
            "state_bridge_output_odometry_topic": state_bridge_output_odometry_topic,
            "state_bridge_timestamp_source": state_bridge_timestamp_source,
            "state_bridge_timesync_status_topic": state_bridge_timesync_status_topic,
            "state_bridge_timesync_timeout_s": state_bridge_timesync_timeout_s,
            "state_bridge_publish_tf": state_bridge_publish_tf,
            "state_bridge_publish_map_to_odom_tf": state_bridge_publish_map_to_odom_tf,
            "state_bridge_log_state": state_bridge_log_state,
            "relative_position_fusion_config": relative_position_fusion_config,
            "global_frame": global_frame,
            "uav_map_frame": uav_map_frame,
            "uav_odom_frame": uav_odom_frame,
            "global_to_uav_map": global_to_uav_map,
            "publish_global_map_tf": publish_global_map_tf,
            "orbbec_camera_name": orbbec_camera_name,
            "base_frame_id": base_frame_id,
            "ov_body_frame_id": ov_body_frame_id,
            "publish_ov_body_tf": publish_ov_body_tf,
            "orbbec_camera_frame_id": orbbec_camera_frame_id,
            "orbbec_camera_x": orbbec_camera_x,
            "orbbec_camera_y": orbbec_camera_y,
            "orbbec_camera_z": orbbec_camera_z,
            "orbbec_camera_roll": orbbec_camera_roll,
            "orbbec_camera_pitch": orbbec_camera_pitch,
            "orbbec_camera_yaw": orbbec_camera_yaw,
            "orbbec_publish_mount_tf": orbbec_publish_mount_tf,
            "orbbec_enable_depth": orbbec_enable_depth,
            "orbbec_enable_color": orbbec_enable_color,
            "orbbec_enable_left_ir": orbbec_enable_left_ir,
            "orbbec_enable_right_ir": orbbec_enable_right_ir,
            "orbbec_enable_point_cloud": orbbec_enable_point_cloud,
            "orbbec_enable_colored_point_cloud": orbbec_enable_colored_point_cloud,
            "orbbec_enable_sync_output_accel_gyro": orbbec_enable_sync_output_accel_gyro,
            "orbbec_enable_publish_extrinsic": orbbec_enable_publish_extrinsic,
            "orbbec_enable_accel": orbbec_enable_accel,
            "orbbec_enable_gyro": orbbec_enable_gyro,
            "orbbec_accel_rate": orbbec_accel_rate,
            "orbbec_gyro_rate": orbbec_gyro_rate,
            "orbbec_left_ir_width": orbbec_left_ir_width,
            "orbbec_left_ir_height": orbbec_left_ir_height,
            "orbbec_left_ir_fps": orbbec_left_ir_fps,
            "orbbec_left_ir_format": orbbec_left_ir_format,
            "orbbec_right_ir_width": orbbec_right_ir_width,
            "orbbec_right_ir_height": orbbec_right_ir_height,
            "orbbec_right_ir_fps": orbbec_right_ir_fps,
            "orbbec_right_ir_format": orbbec_right_ir_format,
            "orbbec_enable_ir_auto_exposure": orbbec_enable_ir_auto_exposure,
            "orbbec_ir_exposure": orbbec_ir_exposure,
            "orbbec_ir_gain": orbbec_ir_gain,
            "orbbec_ir_ae_max_exposure": orbbec_ir_ae_max_exposure,
            "orbbec_ir_brightness": orbbec_ir_brightness,
            "orbbec_enable_laser": orbbec_enable_laser,
            "orbbec_enable_ldp": orbbec_enable_ldp,
        }.items(),
    )

    mono_apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "mono_apriltag_detection.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "backend": backend,
            "device": device,
            "fourcc": fourcc,
            "image_width": image_width,
            "image_height": image_height,
            "fps": fps,
            "camera_name": camera_name,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "base_frame_id": ov_body_frame_id,
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
            "tag_detection_topic": tag_detection_topic,
            "tag_pose_topic": tag_pose_topic,
            "tag_marker_topic": tag_marker_topic,
            "tag_output_frame_mode": tag_output_frame_mode,
            "controller_state_topic": "",
            "target_marker_id": target_marker_id,
            "tag_family": tag_family,
            "tag_size_m": tag_size_m,
            "tag_camera_roll_offset_rad": tag_camera_roll_offset_rad,
            "tag_frame_yaw_offset_rad": tag_frame_yaw_offset_rad,
            "odometry_topic": actual_ov_odom_topic,
            "odometry_timeout_s": odometry_timeout_s,
            "mono_dx": mono_dx,
            "mono_dy": mono_dy,
            "mono_dz": mono_dz,
            "mono_droll": mono_droll,
            "mono_dpitch": mono_dpitch,
            "mono_dyaw": mono_dyaw,
            "publish_tag_base_tf": publish_tag_base_tf,
            "publish_tag_odom_tf": publish_tag_odom_tf,
            "tag_tf_frame_prefix": tag_tf_frame_prefix,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="openvins_mono_apriltag_rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/ov_msckf/odomimu", actual_ov_odom_topic),
            ("/ov_msckf/pathimu", actual_ov_path_topic),
            ("/uav/visual_landing/apriltag_pose", tag_pose_topic),
            ("/uav/visual_landing/apriltag_marker", tag_marker_topic),
        ],
        condition=IfCondition(use_rviz),
    )

    default_openvins_config = os.path.join(
        bringup_share, "config", "openvins", "orbbec_gemini336", "estimator_config.flight.yaml"
    )
    default_rviz_config = os.path.join(
        bringup_share, "config", "rviz", "openvins_orbbec_mono_apriltag.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("start_orbbec_camera", default_value="true"),
        DeclareLaunchArgument("start_openvins", default_value="true"),
        DeclareLaunchArgument("start_px4_vision_bridge", default_value="true"),
        DeclareLaunchArgument("start_uav_state_bridge", default_value="true"),
        DeclareLaunchArgument("enable_relative_position_fusion", default_value="false"),
        DeclareLaunchArgument("enable_relative_tracking", default_value="false"),
        DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
        DeclareLaunchArgument("openvins_config_path", default_value=default_openvins_config),
        DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
        DeclareLaunchArgument("openvins_use_stereo", default_value="true"),
        DeclareLaunchArgument("openvins_use_mask", default_value="true"),
        DeclareLaunchArgument("openvins_max_cameras", default_value="2"),
        DeclareLaunchArgument("openvins_save_total_state", default_value="false"),
        DeclareLaunchArgument("openvins_yaml_left_ir_topic", default_value="/uav_depth_camera/left_ir/image_raw"),
        DeclareLaunchArgument("openvins_yaml_right_ir_topic", default_value="/uav_depth_camera/right_ir/image_raw"),
        DeclareLaunchArgument("openvins_yaml_imu_topic", default_value="/uav_depth_camera/gyro_accel/sample"),
        DeclareLaunchArgument("px4_visual_odometry_topic", default_value="/fmu/in/vehicle_visual_odometry"),
        DeclareLaunchArgument("px4_bridge_expected_odom_frame_id", default_value="global"),
        DeclareLaunchArgument("px4_bridge_expected_child_frame_id", default_value="imu"),
        DeclareLaunchArgument("px4_bridge_sensor_x_in_body_m", default_value="0.0"),
        DeclareLaunchArgument("px4_bridge_sensor_y_in_body_m", default_value="0.0"),
        DeclareLaunchArgument("px4_bridge_sensor_z_in_body_m", default_value="0.0"),
        DeclareLaunchArgument("px4_bridge_sensor_roll_in_body_rad", default_value="0.0"),
        DeclareLaunchArgument("px4_bridge_sensor_pitch_in_body_rad", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("px4_bridge_sensor_yaw_in_body_rad", default_value=str(math.pi / 2.0)),
        DeclareLaunchArgument("px4_bridge_timestamp_source", default_value="px4_timesync"),
        DeclareLaunchArgument("px4_bridge_timesync_status_topic", default_value="/fmu/out/timesync_status"),
        DeclareLaunchArgument("px4_bridge_timesync_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("px4_bridge_log_debug", default_value="false"),
        DeclareLaunchArgument("state_bridge_vehicle_local_position_topic", default_value="/fmu/out/vehicle_local_position"),
        DeclareLaunchArgument("state_bridge_vehicle_odometry_topic", default_value="/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("state_bridge_output_odometry_topic", default_value="/uav/state/odometry_px4"),
        DeclareLaunchArgument("state_bridge_timestamp_source", default_value="px4_timesync"),
        DeclareLaunchArgument("state_bridge_timesync_status_topic", default_value="/fmu/out/timesync_status"),
        DeclareLaunchArgument("state_bridge_timesync_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("state_bridge_publish_tf", default_value="false"),
        DeclareLaunchArgument("state_bridge_publish_map_to_odom_tf", default_value="false"),
        DeclareLaunchArgument("state_bridge_log_state", default_value="false"),
        DeclareLaunchArgument("relative_position_fusion_config", default_value=""),
        DeclareLaunchArgument("global_frame", default_value="global"),
        DeclareLaunchArgument("uav_map_frame", default_value="uav_map"),
        DeclareLaunchArgument("uav_odom_frame", default_value="uav_odom"),
        DeclareLaunchArgument("global_to_uav_map", default_value="0,0,0,0,0,0"),
        DeclareLaunchArgument("publish_global_map_tf", default_value="false"),
        DeclareLaunchArgument("orbbec_camera_name", default_value="uav_depth_camera"),
        DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
        DeclareLaunchArgument("ov_body_frame_id", default_value="uav_ov_body"),
        DeclareLaunchArgument("publish_ov_body_tf", default_value="true"),
        DeclareLaunchArgument("orbbec_camera_frame_id", default_value="uav_stereo_camera_optical_frame"),
        DeclareLaunchArgument("orbbec_camera_x", default_value="0.0503"),
        DeclareLaunchArgument("orbbec_camera_y", default_value="0.0"),
        DeclareLaunchArgument("orbbec_camera_z", default_value="0.1043"),
        DeclareLaunchArgument("orbbec_camera_roll", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("orbbec_camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("orbbec_camera_yaw", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("orbbec_publish_mount_tf", default_value="true"),
        DeclareLaunchArgument("orbbec_enable_depth", default_value="false"),
        DeclareLaunchArgument("orbbec_enable_color", default_value="false"),
        DeclareLaunchArgument("orbbec_enable_left_ir", default_value="true"),
        DeclareLaunchArgument("orbbec_enable_right_ir", default_value="true"),
        DeclareLaunchArgument("orbbec_enable_point_cloud", default_value="false"),
        DeclareLaunchArgument("orbbec_enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("orbbec_enable_sync_output_accel_gyro", default_value="true"),
        DeclareLaunchArgument("orbbec_enable_publish_extrinsic", default_value="false"),
        DeclareLaunchArgument("orbbec_enable_accel", default_value="true"),
        DeclareLaunchArgument("orbbec_enable_gyro", default_value="true"),
        DeclareLaunchArgument("orbbec_accel_rate", default_value="200hz"),
        DeclareLaunchArgument("orbbec_gyro_rate", default_value="200hz"),
        DeclareLaunchArgument("orbbec_left_ir_width", default_value="848"),
        DeclareLaunchArgument("orbbec_left_ir_height", default_value="480"),
        DeclareLaunchArgument("orbbec_left_ir_fps", default_value="30"),
        DeclareLaunchArgument("orbbec_left_ir_format", default_value="ANY"),
        DeclareLaunchArgument("orbbec_right_ir_width", default_value="848"),
        DeclareLaunchArgument("orbbec_right_ir_height", default_value="480"),
        DeclareLaunchArgument("orbbec_right_ir_fps", default_value="30"),
        DeclareLaunchArgument("orbbec_right_ir_format", default_value="ANY"),
        DeclareLaunchArgument("orbbec_enable_ir_auto_exposure", default_value="false"),
        DeclareLaunchArgument("orbbec_ir_exposure", default_value="9000"),
        DeclareLaunchArgument("orbbec_ir_gain", default_value="80"),
        DeclareLaunchArgument("orbbec_ir_ae_max_exposure", default_value="9000"),
        DeclareLaunchArgument("orbbec_ir_brightness", default_value="-1"),
        DeclareLaunchArgument("orbbec_enable_laser", default_value="false"),
        DeclareLaunchArgument("orbbec_enable_ldp", default_value="false"),
        DeclareLaunchArgument("backend", default_value="legacy"),
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("fourcc", default_value="MJPG"),
        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("fps", default_value="120.0"),
        DeclareLaunchArgument("camera_name", default_value="uav_mono_camera"),
        DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
        DeclareLaunchArgument("camera_frame_id", default_value="uav_ov_mono_camera_optical_frame"),
        DeclareLaunchArgument("camera_x", default_value=DEFAULT_CAMERA_TO_BODY["x"]),
        DeclareLaunchArgument("camera_y", default_value=DEFAULT_CAMERA_TO_BODY["y"]),
        DeclareLaunchArgument("camera_z", default_value=DEFAULT_CAMERA_TO_BODY["z"]),
        # Keep the OV-integrated mono mount identical to the original
        # downward-looking body-mounted mono default. The detector now
        # consistently uses this configured frame for TF lookups.
        DeclareLaunchArgument("camera_roll", default_value=str(math.pi)),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("fx", default_value=DEFAULT_CAMERA_INTRINSICS["fx"]),
        DeclareLaunchArgument("fy", default_value=DEFAULT_CAMERA_INTRINSICS["fy"]),
        DeclareLaunchArgument("cx", default_value=DEFAULT_CAMERA_INTRINSICS["cx"]),
        DeclareLaunchArgument("cy", default_value=DEFAULT_CAMERA_INTRINSICS["cy"]),
        DeclareLaunchArgument("camera_hfov_rad", default_value="1.3962634"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("tag_detection_topic", default_value="/uav/visual_landing/apriltag_detection"),
        DeclareLaunchArgument("tag_pose_topic", default_value="/uav/visual_landing/apriltag_pose"),
        DeclareLaunchArgument("tag_marker_topic", default_value="/uav/visual_landing/apriltag_marker"),
        DeclareLaunchArgument("tag_output_frame_mode", default_value="odom"),
        DeclareLaunchArgument("target_marker_id", default_value="0"),
        DeclareLaunchArgument("tag_family", default_value="36h11"),
        DeclareLaunchArgument("tag_size_m", default_value="0.20"),
        DeclareLaunchArgument("tag_camera_roll_offset_rad", default_value=str(math.pi)),
        DeclareLaunchArgument("tag_frame_yaw_offset_rad", default_value=str(-math.pi / 2.0)),
        DeclareLaunchArgument("mono_dx", default_value="0.0"),
        DeclareLaunchArgument("mono_dy", default_value="0.0"),
        DeclareLaunchArgument("mono_dz", default_value="0.0"),
        DeclareLaunchArgument("mono_droll", default_value="0.0"),
        DeclareLaunchArgument("mono_dpitch", default_value="0.0"),
        DeclareLaunchArgument("mono_dyaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("publish_tag_base_tf", default_value="true"),
        DeclareLaunchArgument("publish_tag_odom_tf", default_value="true"),
        DeclareLaunchArgument("tag_tf_frame_prefix", default_value="apriltag"),
        openvins_orbbec,
        mono_apriltag,
        rviz,
    ])
