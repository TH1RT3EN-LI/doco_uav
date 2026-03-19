import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    default_config_path = os.path.join(
        bringup_share,
        "config",
        "openvins",
        "orbbec_stereo_imu",
        "estimator_config.yaml",
    )

    start_openvins = LaunchConfiguration("start_openvins")
    start_orbbec_camera = LaunchConfiguration("start_orbbec_camera")
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

    enable_depth = LaunchConfiguration("enable_depth")
    enable_color = LaunchConfiguration("enable_color")
    enable_left_ir = LaunchConfiguration("enable_left_ir")
    enable_right_ir = LaunchConfiguration("enable_right_ir")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_colored_point_cloud = LaunchConfiguration("enable_colored_point_cloud")
    enable_sync_output_accel_gyro = LaunchConfiguration("enable_sync_output_accel_gyro")
    enable_publish_extrinsic = LaunchConfiguration("enable_publish_extrinsic")
    enable_accel = LaunchConfiguration("enable_accel")
    enable_gyro = LaunchConfiguration("enable_gyro")
    accel_rate = LaunchConfiguration("accel_rate")
    gyro_rate = LaunchConfiguration("gyro_rate")
    left_ir_width = LaunchConfiguration("left_ir_width")
    left_ir_height = LaunchConfiguration("left_ir_height")
    left_ir_fps = LaunchConfiguration("left_ir_fps")
    left_ir_format = LaunchConfiguration("left_ir_format")
    right_ir_width = LaunchConfiguration("right_ir_width")
    right_ir_height = LaunchConfiguration("right_ir_height")
    right_ir_fps = LaunchConfiguration("right_ir_fps")
    right_ir_format = LaunchConfiguration("right_ir_format")
    enable_ir_auto_exposure = LaunchConfiguration("enable_ir_auto_exposure")
    enable_laser = LaunchConfiguration("enable_laser")
    enable_ldp = LaunchConfiguration("enable_ldp")
    log_level = LaunchConfiguration("log_level")

    openvins_namespace = LaunchConfiguration("openvins_namespace")
    openvins_config_path = LaunchConfiguration("openvins_config_path")
    openvins_verbosity = LaunchConfiguration("openvins_verbosity")
    openvins_save_total_state = LaunchConfiguration("openvins_save_total_state")
    openvins_publish_calibration_tf = LaunchConfiguration("openvins_publish_calibration_tf")
    openvins_filepath_est = LaunchConfiguration("openvins_filepath_est")
    openvins_filepath_std = LaunchConfiguration("openvins_filepath_std")
    openvins_filepath_gt = LaunchConfiguration("openvins_filepath_gt")
    publish_px4_external_vision = LaunchConfiguration("publish_px4_external_vision")
    fmu_namespace = LaunchConfiguration("fmu_namespace")
    sensor_roll_in_body_rad = LaunchConfiguration("sensor_roll_in_body_rad")
    sensor_pitch_in_body_rad = LaunchConfiguration("sensor_pitch_in_body_rad")
    sensor_yaw_in_body_rad = LaunchConfiguration("sensor_yaw_in_body_rad")
    imu_topic = LaunchConfiguration("imu_topic")
    left_ir_topic = LaunchConfiguration("left_ir_topic")
    right_ir_topic = LaunchConfiguration("right_ir_topic")

    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "orbbec_depth_camera.launch.py")),
        condition=IfCondition(start_orbbec_camera),
        launch_arguments={
            "start_camera": start_orbbec_camera,
            "camera_launch_file": camera_launch_file,
            "camera_name": camera_name,
            "serial_number": serial_number,
            "usb_port": usb_port,
            "base_frame_id": base_frame_id,
            "camera_frame_id": camera_frame_id,
            "camera_x": camera_x,
            "camera_y": camera_y,
            "camera_z": camera_z,
            "camera_roll": camera_roll,
            "camera_pitch": camera_pitch,
            "camera_yaw": camera_yaw,
            "enable_depth": enable_depth,
            "enable_color": enable_color,
            "enable_left_ir": enable_left_ir,
            "enable_right_ir": enable_right_ir,
            "enable_point_cloud": enable_point_cloud,
            "enable_colored_point_cloud": enable_colored_point_cloud,
            "enable_sync_output_accel_gyro": enable_sync_output_accel_gyro,
            "enable_publish_extrinsic": enable_publish_extrinsic,
            "enable_accel": enable_accel,
            "enable_gyro": enable_gyro,
            "accel_rate": accel_rate,
            "gyro_rate": gyro_rate,
            "left_ir_width": left_ir_width,
            "left_ir_height": left_ir_height,
            "left_ir_fps": left_ir_fps,
            "left_ir_format": left_ir_format,
            "right_ir_width": right_ir_width,
            "right_ir_height": right_ir_height,
            "right_ir_fps": right_ir_fps,
            "right_ir_format": right_ir_format,
            "enable_ir_auto_exposure": enable_ir_auto_exposure,
            "enable_laser": enable_laser,
            "enable_ldp": enable_ldp,
            "log_level": log_level,
        }.items(),
    )

    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(start_openvins),
        namespace=openvins_namespace,
        output="screen",
        parameters=[
            {"verbosity": openvins_verbosity},
            {"use_stereo": True},
            {"max_cameras": 2},
            {"save_total_state": openvins_save_total_state},
            {"publish_calibration_tf": openvins_publish_calibration_tf},
            {"filepath_est": openvins_filepath_est},
            {"filepath_std": openvins_filepath_std},
            {"filepath_gt": openvins_filepath_gt},
            {"config_path": openvins_config_path},
        ],
    )

    openvins_px4_bridge = Node(
        package="uav_bridge",
        executable="openvins_px4_vision_bridge_node",
        condition=IfCondition(publish_px4_external_vision),
        name="openvins_px4_vision_bridge",
        output="screen",
        parameters=[
            {"odometry_topic": ["/", openvins_namespace, "/odomimu"]},
            {"visual_odometry_topic": [fmu_namespace, "/in/vehicle_visual_odometry"]},
            {"expected_odom_frame_id": "global"},
            {"expected_child_frame_id": "imu"},
            {"sensor_roll_in_body_rad": sensor_roll_in_body_rad},
            {"sensor_pitch_in_body_rad": sensor_pitch_in_body_rad},
            {"sensor_yaw_in_body_rad": sensor_yaw_in_body_rad},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_openvins", default_value="true"),
            DeclareLaunchArgument("start_orbbec_camera", default_value="true"),
            DeclareLaunchArgument("camera_launch_file", default_value="gemini_330_series.launch.py"),
            DeclareLaunchArgument("camera_name", default_value="uav_depth_camera"),
            DeclareLaunchArgument("serial_number", default_value=""),
            DeclareLaunchArgument("usb_port", default_value=""),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument("camera_frame_id", default_value="uav_stereo_camera_optical_frame"),
            DeclareLaunchArgument("camera_x", default_value="0.0503"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.1043"),
            DeclareLaunchArgument("camera_roll", default_value=str(-3.141592653589793 / 2.0)),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value=str(-3.141592653589793 / 2.0)),
            DeclareLaunchArgument("enable_depth", default_value="false"),
            DeclareLaunchArgument("enable_color", default_value="false"),
            DeclareLaunchArgument("enable_left_ir", default_value="true"),
            DeclareLaunchArgument("enable_right_ir", default_value="true"),
            DeclareLaunchArgument("enable_point_cloud", default_value="false"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="true"),
            DeclareLaunchArgument("enable_publish_extrinsic", default_value="true"),
            DeclareLaunchArgument("enable_accel", default_value="true"),
            DeclareLaunchArgument("enable_gyro", default_value="true"),
            DeclareLaunchArgument("accel_rate", default_value="200hz"),
            DeclareLaunchArgument("gyro_rate", default_value="200hz"),
            DeclareLaunchArgument("left_ir_width", default_value="0"),
            DeclareLaunchArgument("left_ir_height", default_value="0"),
            DeclareLaunchArgument("left_ir_fps", default_value="0"),
            DeclareLaunchArgument("left_ir_format", default_value="ANY"),
            DeclareLaunchArgument("right_ir_width", default_value="0"),
            DeclareLaunchArgument("right_ir_height", default_value="0"),
            DeclareLaunchArgument("right_ir_fps", default_value="0"),
            DeclareLaunchArgument("right_ir_format", default_value="ANY"),
            DeclareLaunchArgument("enable_ir_auto_exposure", default_value="true"),
            DeclareLaunchArgument("enable_laser", default_value="false"),
            DeclareLaunchArgument("enable_ldp", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("openvins_namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("openvins_config_path", default_value=default_config_path),
            DeclareLaunchArgument("openvins_verbosity", default_value="INFO"),
            DeclareLaunchArgument("openvins_save_total_state", default_value="false"),
            DeclareLaunchArgument("openvins_publish_calibration_tf", default_value="true"),
            DeclareLaunchArgument("openvins_filepath_est", default_value="/tmp/ov_estimate.txt"),
            DeclareLaunchArgument("openvins_filepath_std", default_value="/tmp/ov_estimate_std.txt"),
            DeclareLaunchArgument("openvins_filepath_gt", default_value="/tmp/ov_groundtruth.txt"),
            DeclareLaunchArgument("publish_px4_external_vision", default_value="true"),
            DeclareLaunchArgument("fmu_namespace", default_value="/fmu"),
            DeclareLaunchArgument("sensor_roll_in_body_rad", default_value="0.0"),
            DeclareLaunchArgument("sensor_pitch_in_body_rad", default_value="0.0"),
            DeclareLaunchArgument("sensor_yaw_in_body_rad", default_value="0.0"),
            DeclareLaunchArgument("imu_topic", default_value=["/", camera_name, "/gyro_accel/sample"]),
            DeclareLaunchArgument("left_ir_topic", default_value=["/", camera_name, "/left_ir/image_raw"]),
            DeclareLaunchArgument("right_ir_topic", default_value=["/", camera_name, "/right_ir/image_raw"]),
            orbbec_camera_launch,
            openvins_node,
            openvins_px4_bridge,
        ]
    )
