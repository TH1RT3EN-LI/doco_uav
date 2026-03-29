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
CONTROLLER_STATE_TOPIC = "/uav/visual_landing/controller_state"


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    visual_landing_share = get_package_share_directory("uav_visual_landing")

    takeoff_height_m = LaunchConfiguration("takeoff_height_m")
    motion_guard_enabled = LaunchConfiguration("motion_guard_enabled")
    motion_guard_soft_dwell_s = LaunchConfiguration("motion_guard_soft_dwell_s")
    motion_guard_pose_gap_reset_s = LaunchConfiguration("motion_guard_pose_gap_reset_s")
    motion_guard_soft_xy_mps = LaunchConfiguration("motion_guard_soft_xy_mps")
    motion_guard_soft_z_mps = LaunchConfiguration("motion_guard_soft_z_mps")
    motion_guard_soft_yaw_radps = LaunchConfiguration("motion_guard_soft_yaw_radps")
    motion_guard_hard_xy_mps = LaunchConfiguration("motion_guard_hard_xy_mps")
    motion_guard_hard_z_mps = LaunchConfiguration("motion_guard_hard_z_mps")
    motion_guard_hard_yaw_radps = LaunchConfiguration("motion_guard_hard_yaw_radps")
    motion_guard_feedback_hard_xy_mps = LaunchConfiguration("motion_guard_feedback_hard_xy_mps")
    motion_guard_feedback_hard_z_mps = LaunchConfiguration("motion_guard_feedback_hard_z_mps")
    motion_guard_pose_soft_xy_step_m = LaunchConfiguration("motion_guard_pose_soft_xy_step_m")
    motion_guard_pose_soft_z_step_m = LaunchConfiguration("motion_guard_pose_soft_z_step_m")
    motion_guard_pose_soft_yaw_step_rad = LaunchConfiguration("motion_guard_pose_soft_yaw_step_rad")
    motion_guard_pose_hard_xy_step_m = LaunchConfiguration("motion_guard_pose_hard_xy_step_m")
    motion_guard_pose_hard_z_step_m = LaunchConfiguration("motion_guard_pose_hard_z_step_m")
    motion_guard_pose_hard_yaw_step_rad = LaunchConfiguration("motion_guard_pose_hard_yaw_step_rad")
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
    start_openvins_orbbec = LaunchConfiguration("start_openvins_orbbec")
    publish_px4_external_vision = LaunchConfiguration("publish_px4_external_vision")
    openvins_orbbec_namespace = LaunchConfiguration("openvins_orbbec_namespace")
    openvins_orbbec_config_path = LaunchConfiguration("openvins_orbbec_config_path")
    openvins_sensor_roll_in_body_rad = LaunchConfiguration("openvins_sensor_roll_in_body_rad")
    openvins_sensor_pitch_in_body_rad = LaunchConfiguration("openvins_sensor_pitch_in_body_rad")
    openvins_sensor_yaw_in_body_rad = LaunchConfiguration("openvins_sensor_yaw_in_body_rad")
    use_sim_time = LaunchConfiguration("use_sim_time")
    height_measurement_transport = LaunchConfiguration("height_measurement_transport")
    height_measurement_topic = LaunchConfiguration("height_measurement_topic")
    height_measurement_mode = LaunchConfiguration("height_measurement_mode")
    height_measurement_frame_id = LaunchConfiguration("height_measurement_frame_id")

    fmu_namespace = "/fmu"
    vehicle_local_position_topic = f"{fmu_namespace}/out/vehicle_local_position"
    distance_sensor_topic = f"{fmu_namespace}/out/distance_sensor"
    default_recording_path = str(
        Path.home() / "uav_recordings" / f"visual_landing_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    )

    minimal_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "minimal_control.launch.py")
        ),
        launch_arguments={
            "takeoff_height_m": takeoff_height_m,
            "motion_guard_enabled": motion_guard_enabled,
            "motion_guard_soft_dwell_s": motion_guard_soft_dwell_s,
            "motion_guard_pose_gap_reset_s": motion_guard_pose_gap_reset_s,
            "motion_guard_soft_xy_mps": motion_guard_soft_xy_mps,
            "motion_guard_soft_z_mps": motion_guard_soft_z_mps,
            "motion_guard_soft_yaw_radps": motion_guard_soft_yaw_radps,
            "motion_guard_hard_xy_mps": motion_guard_hard_xy_mps,
            "motion_guard_hard_z_mps": motion_guard_hard_z_mps,
            "motion_guard_hard_yaw_radps": motion_guard_hard_yaw_radps,
            "motion_guard_feedback_hard_xy_mps": motion_guard_feedback_hard_xy_mps,
            "motion_guard_feedback_hard_z_mps": motion_guard_feedback_hard_z_mps,
            "motion_guard_pose_soft_xy_step_m": motion_guard_pose_soft_xy_step_m,
            "motion_guard_pose_soft_z_step_m": motion_guard_pose_soft_z_step_m,
            "motion_guard_pose_soft_yaw_step_rad": motion_guard_pose_soft_yaw_step_rad,
            "motion_guard_pose_hard_xy_step_m": motion_guard_pose_hard_xy_step_m,
            "motion_guard_pose_hard_z_step_m": motion_guard_pose_hard_z_step_m,
            "motion_guard_pose_hard_yaw_step_rad": motion_guard_pose_hard_yaw_step_rad,
            "record_mono_video": "false",
            "start_openvins_orbbec": start_openvins_orbbec,
            "publish_px4_external_vision": publish_px4_external_vision,
            "openvins_orbbec_namespace": openvins_orbbec_namespace,
            "openvins_orbbec_config_path": openvins_orbbec_config_path,
            "openvins_sensor_roll_in_body_rad": openvins_sensor_roll_in_body_rad,
            "openvins_sensor_pitch_in_body_rad": openvins_sensor_pitch_in_body_rad,
            "openvins_sensor_yaw_in_body_rad": openvins_sensor_yaw_in_body_rad,
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
            {"use_sim_time": use_sim_time},
            {"tag_size_m": tag_size_m},
        ],
    )

    height_measurement_bridge = Node(
        package="uav_bridge",
        executable="height_measurement_bridge_node",
        name="height_measurement_bridge_node",
        output="screen",
        parameters=[
            {"distance_sensor_topic": distance_sensor_topic},
            {"height_measurement_topic": height_measurement_topic},
            {"frame_id": height_measurement_frame_id},
            {"use_sim_time": use_sim_time},
        ],
    )

    visual_landing = Node(
        package="uav_visual_landing",
        executable="visual_landing_node",
        name="visual_landing_node",
        output="screen",
        parameters=[
            visual_landing_config,
            {"use_sim_time": use_sim_time},
            {"search_height_m": search_height_m},
            {"terminal_entry_height_m": terminal_entry_height_m},
            {"max_vxy": max_vxy},
            {"align_enter_lateral_m": align_enter_lateral_m},
            {"align_exit_lateral_m": align_exit_lateral_m},
            {"height_measurement_transport": height_measurement_transport},
            {"height_measurement_topic": height_measurement_topic},
            {"height_measurement_mode": height_measurement_mode},
            {"range_topic": distance_sensor_topic},
            {"vehicle_local_position_topic": vehicle_local_position_topic},
        ],
    )

    home_visual_landing_test = Node(
        package="uav_bringup",
        executable="home_visual_landing_test_node.py",
        name="home_visual_landing_test_node",
        output="screen",
        parameters=[
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
            {"output_path": record_video_output_path},
            {"fps": record_video_fps},
            {"fourcc": record_video_fourcc},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("takeoff_height_m", default_value="0.60"),
        DeclareLaunchArgument("motion_guard_enabled", default_value="true"),
        DeclareLaunchArgument("motion_guard_soft_dwell_s", default_value="2.0"),
        DeclareLaunchArgument("motion_guard_pose_gap_reset_s", default_value="0.40"),
        DeclareLaunchArgument("motion_guard_soft_xy_mps", default_value="0.40"),
        DeclareLaunchArgument("motion_guard_soft_z_mps", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_soft_yaw_radps", default_value="0.60"),
        DeclareLaunchArgument("motion_guard_hard_xy_mps", default_value="0.55"),
        DeclareLaunchArgument("motion_guard_hard_z_mps", default_value="0.35"),
        DeclareLaunchArgument("motion_guard_hard_yaw_radps", default_value="0.90"),
        DeclareLaunchArgument("motion_guard_feedback_hard_xy_mps", default_value="0.65"),
        DeclareLaunchArgument("motion_guard_feedback_hard_z_mps", default_value="0.45"),
        DeclareLaunchArgument("motion_guard_pose_soft_xy_step_m", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_pose_soft_z_step_m", default_value="0.12"),
        DeclareLaunchArgument("motion_guard_pose_soft_yaw_step_rad", default_value="0.35"),
        DeclareLaunchArgument("motion_guard_pose_hard_xy_step_m", default_value="0.50"),
        DeclareLaunchArgument("motion_guard_pose_hard_z_step_m", default_value="0.25"),
        DeclareLaunchArgument("motion_guard_pose_hard_yaw_step_rad", default_value="0.70"),
        DeclareLaunchArgument("search_height_m", default_value="0.60"),
        DeclareLaunchArgument("terminal_entry_height_m", default_value="0.40"),
        DeclareLaunchArgument("max_vxy", default_value="0.10"),
        DeclareLaunchArgument("align_enter_lateral_m", default_value="0.08"),
        DeclareLaunchArgument("align_exit_lateral_m", default_value="0.05"),
        DeclareLaunchArgument("tag_size_m", default_value="0.13"),
        DeclareLaunchArgument("record_video_source", default_value="debug"),
        DeclareLaunchArgument("record_video_output_path", default_value=default_recording_path),
        DeclareLaunchArgument("record_video_fps", default_value="30.0"),
        DeclareLaunchArgument("record_video_fourcc", default_value="MJPG"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("height_measurement_transport", default_value="stamped_range"),
        DeclareLaunchArgument(
            "height_measurement_topic", default_value="/uav/sensors/downward_range"
        ),
        DeclareLaunchArgument("height_measurement_mode", default_value=""),
        DeclareLaunchArgument(
            "height_measurement_frame_id", default_value="uav_optical_flow_range_frame"
        ),
        DeclareLaunchArgument("start_openvins_orbbec", default_value="true"),
        DeclareLaunchArgument("publish_px4_external_vision", default_value="true"),
        DeclareLaunchArgument("openvins_orbbec_namespace", default_value="ov_msckf"),
        DeclareLaunchArgument(
            "openvins_orbbec_config_path",
            default_value=os.path.join(
                bringup_share,
                "config",
                "openvins",
                "orbbec_stereo_imu",
                "frozen_final",
                "estimator_config.flight.yaml",
            ),
        ),
        DeclareLaunchArgument("openvins_sensor_roll_in_body_rad", default_value="0.0"),
        DeclareLaunchArgument("openvins_sensor_pitch_in_body_rad", default_value="0.0"),
        DeclareLaunchArgument("openvins_sensor_yaw_in_body_rad", default_value="0.0"),
        minimal_control_launch,
        aruco_detector,
        height_measurement_bridge,
        visual_landing,
        home_visual_landing_test,
        debug_video_recorder,
        mono_video_recorder,
    ])
