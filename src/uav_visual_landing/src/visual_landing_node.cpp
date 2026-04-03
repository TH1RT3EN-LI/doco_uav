#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <uav_visual_landing/msg/landing_controller_state.hpp>
#include <uav_visual_landing/msg/target_observation.hpp>
#include <uav_visual_landing/visual_landing_logic.hpp>

namespace uav_visual_landing
{

class VisualLandingNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

public:
  VisualLandingNode()
  : Node("visual_landing_node")
  {
    this->declare_parameter<std::string>(
      "target_observation_topic",
      "/uav/visual_landing/target_observation");
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>(
      "velocity_body_topic",
      "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>(
      "controller_state_topic",
      "/uav/visual_landing/controller_state");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>("disarm_service", "/uav/control/command/disarm");
    this->declare_parameter<std::string>("height_measurement_transport", "");
    this->declare_parameter<std::string>("height_measurement_topic", "/uav/sensors/downward_range");
    this->declare_parameter<std::string>("height_measurement_mode", "");
    this->declare_parameter<std::string>("range_topic", "/fmu/out/distance_sensor");
    this->declare_parameter<std::string>(
      "vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected");
    this->declare_parameter<std::string>(
      "vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    this->declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status");
    this->declare_parameter<std::string>("start_service", "/uav/visual_landing/command/start");
    this->declare_parameter<std::string>("stop_service", "/uav/visual_landing/command/stop");

    this->declare_parameter<double>("kp_xy_m", 0.80);
    this->declare_parameter<double>("kd_xy_m", 0.15);
    this->declare_parameter<double>("xy_error_alpha", 0.35);
    this->declare_parameter<double>("xy_error_rate_alpha", 0.25);
    this->declare_parameter<double>("xy_error_rate_limit", 2.0);
    this->declare_parameter<double>("xy_error_rate_dt_min_s", 0.005);
    this->declare_parameter<double>("xy_error_rate_dt_max_s", 0.20);

    this->declare_parameter<double>("kp_yaw", 1.20);
    this->declare_parameter<double>("max_vyaw", 0.60);
    this->declare_parameter<double>("vel_damping_yaw", 0.18);
    this->declare_parameter<double>("yaw_deadband_rad", 0.03);

    this->declare_parameter<double>("align_enter_lateral_m", 0.08);
    this->declare_parameter<double>("align_exit_lateral_m", 0.05);
    this->declare_parameter<double>("align_enter_yaw_rad", 0.08);
    this->declare_parameter<double>("align_exit_yaw_rad", 0.06);
    this->declare_parameter<double>("max_vxy", 0.40);

    this->declare_parameter<double>("hold_verify_s", 0.50);
    this->declare_parameter<double>("search_height_m", 1.0);
    this->declare_parameter<double>("search_height_tolerance_m", 0.10);
    this->declare_parameter<double>("descend_speed", 0.12);
    this->declare_parameter<double>("terminal_entry_height_m", 0.40);
    this->declare_parameter<double>("terminal_height_band_m", 0.03);
    this->declare_parameter<double>("terminal_align_lateral_m", 0.05);
    this->declare_parameter<double>("terminal_align_yaw_rad", 0.15);
    this->declare_parameter<double>("terminal_confirm_s", 2.0);
    this->declare_parameter<double>("terminal_land_height_m", 0.20);
    this->declare_parameter<double>("terminal_land_max_yaw_rate_radps", 0.05);
    this->declare_parameter<double>("offboard_land_vz_mps", 0.40);
    this->declare_parameter<double>("offboard_land_disarm_height_m", 0.20);
    this->declare_parameter<double>("offboard_land_disarm_hold_s", 0.25);
    this->declare_parameter<double>("offboard_land_disarm_max_xy_mps", 0.25);
    this->declare_parameter<double>("offboard_land_disarm_max_vz_mps", 0.25);
    this->declare_parameter<double>("z_hold_kp", 1.20);
    this->declare_parameter<double>("z_hold_max_vz", 0.18);
    this->declare_parameter<double>("z_descend_kp", 1.30);
    this->declare_parameter<double>("z_descend_max_vz", 0.25);
    this->declare_parameter<double>("z_terminal_max_vz", 0.35);
    this->declare_parameter<double>("vel_damping_z", 0.18);

    this->declare_parameter<double>("observation_timeout_s", 0.30);
    this->declare_parameter<int>("observation_miss_frames", 3);
    this->declare_parameter<double>("state_timeout_s", 0.20);
    this->declare_parameter<double>("target_memory_timeout_s", 0.50);
    this->declare_parameter<double>("range_timeout_s", 0.15);
    this->declare_parameter<double>("range_min_m", 0.05);
    this->declare_parameter<double>("range_max_m", 5.0);
    this->declare_parameter<double>("range_consistency_max_diff_m", 0.35);
    this->declare_parameter<double>("control_rate_hz", 30.0);
    this->declare_parameter<double>("status_rate_hz", 5.0);

    target_observation_topic_ = this->get_parameter("target_observation_topic").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    velocity_body_topic_ = this->get_parameter("velocity_body_topic").as_string();
    controller_state_topic_ = this->get_parameter("controller_state_topic").as_string();
    hold_service_ = this->get_parameter("hold_service").as_string();
    disarm_service_ = this->get_parameter("disarm_service").as_string();
    const std::string requested_height_measurement_transport =
      this->get_parameter("height_measurement_transport").as_string();
    const std::string legacy_height_measurement_mode =
      this->get_parameter("height_measurement_mode").as_string();
    const std::string normalized_height_measurement_transport =
      normalizeHeightMeasurementTransport(requested_height_measurement_transport);
    const std::string normalized_legacy_height_measurement_transport =
      normalizeLegacyHeightMeasurementMode(legacy_height_measurement_mode);
    if (!requested_height_measurement_transport.empty() &&
      normalized_height_measurement_transport.empty())
    {
      throw std::runtime_error("invalid height_measurement_transport parameter");
    }
    if (!legacy_height_measurement_mode.empty() &&
      normalized_legacy_height_measurement_transport.empty())
    {
      throw std::runtime_error("invalid legacy height_measurement_mode parameter");
    }
    if (!normalized_height_measurement_transport.empty()) {
      height_measurement_transport_ = heightMeasurementTransportFromName(
        normalized_height_measurement_transport);
    } else if (!normalized_legacy_height_measurement_transport.empty()) {
      height_measurement_transport_ = heightMeasurementTransportFromName(
        normalized_legacy_height_measurement_transport);
      RCLCPP_WARN(
        this->get_logger(),
        "height_measurement_mode is deprecated; use height_measurement_transport instead");
    } else {
      height_measurement_transport_ = HeightMeasurementTransport::StampedRange;
    }
    height_measurement_time_basis_ = heightMeasurementTimeBasisName(height_measurement_transport_);
    height_measurement_topic_ = this->get_parameter("height_measurement_topic").as_string();
    range_topic_ = this->get_parameter("range_topic").as_string();
    vehicle_land_detected_topic_ =
      this->get_parameter("vehicle_land_detected_topic").as_string();
    vehicle_local_position_topic_ =
      this->get_parameter("vehicle_local_position_topic").as_string();
    vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
    start_service_name_ = this->get_parameter("start_service").as_string();
    stop_service_name_ = this->get_parameter("stop_service").as_string();

    kp_xy_m_ = static_cast<float>(this->get_parameter("kp_xy_m").as_double());
    kd_xy_m_ = static_cast<float>(this->get_parameter("kd_xy_m").as_double());
    xy_error_alpha_ = static_cast<float>(this->get_parameter("xy_error_alpha").as_double());
    xy_error_rate_alpha_ =
      static_cast<float>(this->get_parameter("xy_error_rate_alpha").as_double());
    xy_error_rate_limit_ =
      static_cast<float>(this->get_parameter("xy_error_rate_limit").as_double());
    xy_error_rate_dt_min_s_ =
      static_cast<float>(this->get_parameter("xy_error_rate_dt_min_s").as_double());
    xy_error_rate_dt_max_s_ =
      static_cast<float>(this->get_parameter("xy_error_rate_dt_max_s").as_double());

    kp_yaw_ = static_cast<float>(this->get_parameter("kp_yaw").as_double());
    max_vyaw_ = static_cast<float>(this->get_parameter("max_vyaw").as_double());
    vel_damping_yaw_ = static_cast<float>(this->get_parameter("vel_damping_yaw").as_double());
    yaw_deadband_rad_ = static_cast<float>(this->get_parameter("yaw_deadband_rad").as_double());

    alignment_config_.align_enter_lateral_m =
      static_cast<float>(this->get_parameter("align_enter_lateral_m").as_double());
    alignment_config_.align_exit_lateral_m =
      static_cast<float>(this->get_parameter("align_exit_lateral_m").as_double());
    alignment_config_.align_enter_yaw_rad =
      static_cast<float>(this->get_parameter("align_enter_yaw_rad").as_double());
    alignment_config_.align_exit_yaw_rad =
      static_cast<float>(this->get_parameter("align_exit_yaw_rad").as_double());
    max_vxy_ = static_cast<float>(this->get_parameter("max_vxy").as_double());

    hold_verify_s_ = this->get_parameter("hold_verify_s").as_double();
    search_height_m_ = static_cast<float>(this->get_parameter("search_height_m").as_double());
    search_height_tolerance_m_ =
      static_cast<float>(this->get_parameter("search_height_tolerance_m").as_double());
    descend_speed_ = static_cast<float>(this->get_parameter("descend_speed").as_double());
    terminal_entry_height_m_ =
      static_cast<float>(this->get_parameter("terminal_entry_height_m").as_double());
    terminal_height_band_m_ =
      static_cast<float>(this->get_parameter("terminal_height_band_m").as_double());
    terminal_align_lateral_m_ =
      static_cast<float>(this->get_parameter("terminal_align_lateral_m").as_double());
    terminal_align_yaw_rad_ =
      static_cast<float>(this->get_parameter("terminal_align_yaw_rad").as_double());
    terminal_confirm_s_ = this->get_parameter("terminal_confirm_s").as_double();
    terminal_land_height_m_ =
      static_cast<float>(this->get_parameter("terminal_land_height_m").as_double());
    terminal_land_max_yaw_rate_radps_ =
      static_cast<float>(this->get_parameter("terminal_land_max_yaw_rate_radps").as_double());
    offboard_land_vz_mps_ =
      static_cast<float>(this->get_parameter("offboard_land_vz_mps").as_double());
    offboard_land_disarm_height_m_ =
      static_cast<float>(this->get_parameter("offboard_land_disarm_height_m").as_double());
    offboard_land_disarm_hold_s_ =
      this->get_parameter("offboard_land_disarm_hold_s").as_double();
    offboard_land_disarm_max_xy_mps_ =
      static_cast<float>(this->get_parameter("offboard_land_disarm_max_xy_mps").as_double());
    offboard_land_disarm_max_vz_mps_ =
      static_cast<float>(this->get_parameter("offboard_land_disarm_max_vz_mps").as_double());
    offboard_land_vz_mps_ = std::max(0.01f, offboard_land_vz_mps_);
    offboard_land_disarm_height_m_ = std::max(0.0f, offboard_land_disarm_height_m_);
    offboard_land_disarm_hold_s_ = std::max(0.0, offboard_land_disarm_hold_s_);
    offboard_land_disarm_max_xy_mps_ = std::max(0.0f, offboard_land_disarm_max_xy_mps_);
    offboard_land_disarm_max_vz_mps_ = std::max(0.0f, offboard_land_disarm_max_vz_mps_);
    z_hold_kp_ = static_cast<float>(this->get_parameter("z_hold_kp").as_double());
    z_hold_max_vz_ = static_cast<float>(this->get_parameter("z_hold_max_vz").as_double());
    z_descend_kp_ = static_cast<float>(this->get_parameter("z_descend_kp").as_double());
    z_descend_max_vz_ = static_cast<float>(this->get_parameter("z_descend_max_vz").as_double());
    z_terminal_max_vz_ = static_cast<float>(this->get_parameter("z_terminal_max_vz").as_double());
    vel_damping_z_ = static_cast<float>(this->get_parameter("vel_damping_z").as_double());

    observation_timeout_s_ = this->get_parameter("observation_timeout_s").as_double();
    observation_miss_frames_ = this->get_parameter("observation_miss_frames").as_int();
    state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    target_memory_timeout_s_ = this->get_parameter("target_memory_timeout_s").as_double();
    height_config_.timeout_s = this->get_parameter("range_timeout_s").as_double();
    height_config_.min_m = static_cast<float>(this->get_parameter("range_min_m").as_double());
    height_config_.max_m = static_cast<float>(this->get_parameter("range_max_m").as_double());
    height_config_.max_diff_m =
      static_cast<float>(this->get_parameter("range_consistency_max_diff_m").as_double());

    const double control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    const double status_rate_hz = this->get_parameter("status_rate_hz").as_double();
    control_dt_s_ = control_rate_hz >
      1.0e-6 ? static_cast<float>(1.0 / control_rate_hz) : (1.0f / 30.0f);
    status_period_s_ = status_rate_hz > 1.0e-6 ? (1.0 / status_rate_hz) : 0.2;

    auto velocity_qos = rclcpp::SensorDataQoS();
    velocity_qos.keep_last(1);
    auto observation_qos = rclcpp::SensorDataQoS();
    observation_qos.keep_last(1);

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      velocity_body_topic_,
      velocity_qos);
    controller_state_pub_ = this->create_publisher<uav_visual_landing::msg::LandingControllerState>(
      controller_state_topic_, rclcpp::QoS(1).reliable().transient_local());
    hold_client_ = this->create_client<Trigger>(hold_service_);
    disarm_client_ = this->create_client<Trigger>(disarm_service_);

    observation_sub_ = this->create_subscription<uav_visual_landing::msg::TargetObservation>(
      target_observation_topic_, observation_qos,
      [this](const uav_visual_landing::msg::TargetObservation::SharedPtr msg)
      {
        onObservation(*msg);
      });

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        const float odom_height_m = static_cast<float>(msg->pose.pose.position.z);
        const float body_vx = static_cast<float>(msg->twist.twist.linear.x);
        const float body_vy = static_cast<float>(msg->twist.twist.linear.y);
        const float body_vz = static_cast<float>(msg->twist.twist.linear.z);
        const float yaw_rate = static_cast<float>(msg->twist.twist.angular.z);
        if (
          !std::isfinite(odom_height_m) || !std::isfinite(body_vx) || !std::isfinite(body_vy) ||
          !std::isfinite(body_vz) || !std::isfinite(yaw_rate))
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "discarding odometry with non-finite z/velocity/yaw-rate");
          return;
        }
        const bool has_stamp =
        (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
        odom_height_m_ = odom_height_m;
        current_body_vx_ = body_vx;
        current_body_vy_ = body_vy;
        current_body_vz_ = body_vz;
        current_yaw_rate_ = yaw_rate;
        last_state_time_ = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
        has_state_ = true;
      });

    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
      {
        const bool was_armed = is_armed_;
        is_armed_ =
          msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        has_vehicle_status_ = true;
        if (phase_ == ControllerPhase::Land && is_armed_) {
          land_phase_armed_seen_ = true;
        }
        if (phase_ == ControllerPhase::Land && land_phase_armed_seen_ && was_armed && !is_armed_) {
          RCLCPP_INFO(this->get_logger(), "landing completed via vehicle_status disarm");
          finishAfterLandingComplete();
        }
      });

    vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
      vehicle_land_detected_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
      {
        has_vehicle_land_detected_ = true;
        ground_contact_ = msg->ground_contact;
        maybe_landed_ = msg->maybe_landed;
        landed_ = msg->landed;
      });

    if (height_measurement_transport_ == HeightMeasurementTransport::StampedRange) {
      stamped_height_measurement_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        height_measurement_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg)
        {
          onStampedHeightMeasurement(*msg);
        });
    } else if (height_measurement_transport_ ==
      HeightMeasurementTransport::LegacyPx4DistanceSensor)
    {
      legacy_distance_sensor_height_sub_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
        range_topic_, rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::DistanceSensor::SharedPtr msg)
        {
          onDistanceSensorHeightMeasurement(*msg);
        });
    } else {
      vehicle_local_position_height_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        vehicle_local_position_topic_, rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
        {
          onVehicleLocalPositionHeightMeasurement(*msg);
        });
    }

    start_srv_ = this->create_service<Trigger>(
      start_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = start(response->message);
      });

    stop_srv_ = this->create_service<Trigger>(
      stop_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = stop(response->message);
      });

    const double period_ms = control_rate_hz >
      1.0e-6 ? (1000.0 / control_rate_hz) : (1000.0 / 30.0);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&VisualLandingNode::timerCallback, this));

    publishControllerState(
      true, HeightDecision{}, 0.0f, MetricLateralError{}, 0.0f, 0.0f, 0.0f,
      0.0f);
    RCLCPP_INFO(
      this->get_logger(),
      "visual_landing_node: observation=%s state=%s velocity_body=%s hold=%s disarm=%s "
      "height_transport=%s height_topic=%s range_topic=%s vehicle_land_detected_topic=%s "
      "vehicle_local_position_topic=%s "
      "start=%s stop=%s",
      target_observation_topic_.c_str(), state_topic_.c_str(), velocity_body_topic_.c_str(),
      hold_service_.c_str(), disarm_service_.c_str(),
      heightMeasurementTransportName(height_measurement_transport_),
      height_measurement_topic_.c_str(), range_topic_.c_str(),
      vehicle_land_detected_topic_.c_str(),
      vehicle_local_position_topic_.c_str(),
      start_service_name_.c_str(), stop_service_name_.c_str());
  }

private:
  static std::string normalizeHeightMeasurementTransport(std::string transport)
  {
    std::transform(
      transport.begin(), transport.end(), transport.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (transport.empty()) {
      return "";
    }
    if (transport == "stamped_range" || transport == "range" || transport == "ros_range") {
      return "stamped_range";
    }
    if (
      transport == "legacy_px4_distance_sensor" || transport == "px4_distance_sensor" ||
      transport == "distance_sensor")
    {
      return "legacy_px4_distance_sensor";
    }
    if (
      transport == "legacy_vehicle_local_position" || transport == "vehicle_local_position" ||
      transport == "px4_vehicle_local_position" || transport == "px4_dist_bottom" ||
      transport == "dist_bottom")
    {
      return "legacy_vehicle_local_position";
    }
    return "";
  }

  static std::string normalizeLegacyHeightMeasurementMode(std::string mode)
  {
    std::transform(
      mode.begin(), mode.end(), mode.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (
      mode == "distance_sensor" || mode == "distance" || mode == "range" ||
      mode == "range_sensor")
    {
      return "legacy_px4_distance_sensor";
    }
    if (
      mode == "vehicle_local_position" || mode == "px4_vehicle_local_position" ||
      mode == "px4_dist_bottom" || mode == "dist_bottom")
    {
      return "legacy_vehicle_local_position";
    }
    return "";
  }

  static HeightMeasurementTransport heightMeasurementTransportFromName(const std::string & name)
  {
    if (name == "stamped_range") {
      return HeightMeasurementTransport::StampedRange;
    }
    if (name == "legacy_px4_distance_sensor") {
      return HeightMeasurementTransport::LegacyPx4DistanceSensor;
    }
    return HeightMeasurementTransport::LegacyVehicleLocalPosition;
  }

  static const char * heightMeasurementTimeBasisName(HeightMeasurementTransport transport)
  {
    return usesSampleTimeForHeightMeasurement(transport) ? "SAMPLE_TIME" : "LEGACY_RECEIVE_TIME";
  }

  static std::string describeDistBottomSource(uint8_t sensor_bitfield)
  {
    const bool uses_range = (sensor_bitfield &
      px4_msgs::msg::VehicleLocalPosition::DIST_BOTTOM_SENSOR_RANGE) != 0U;
    const bool uses_flow = (sensor_bitfield &
      px4_msgs::msg::VehicleLocalPosition::DIST_BOTTOM_SENSOR_FLOW) != 0U;
    if (uses_range && uses_flow) {
      return "PX4_DIST_BOTTOM[RANGE+FLOW]";
    }
    if (uses_range) {
      return "PX4_DIST_BOTTOM[RANGE]";
    }
    if (uses_flow) {
      return "PX4_DIST_BOTTOM[FLOW]";
    }
    return "PX4_DIST_BOTTOM";
  }

  bool start(std::string & message)
  {
    active_ = true;
    resetDisarmRequestState();
    land_phase_armed_seen_ = false;
    land_disarm_condition_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    initial_search_completed_ = false;
    clearObservationTrackingState();
    clearVisionTrackingState();
    clearTargetMemory();
    transitionTo(ControllerPhase::HoldWait, true);
    message = "visual landing started";
    return true;
  }

  bool stop(std::string & message)
  {
    active_ = false;
    resetDisarmRequestState();
    land_phase_armed_seen_ = false;
    land_disarm_condition_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    initial_search_completed_ = false;
    clearObservationTrackingState();
    clearVisionTrackingState();
    clearTargetMemory();
    transitionTo(ControllerPhase::Ready, true);
    message = "visual landing stopped";
    return true;
  }

  void clearVisionTrackingState()
  {
    filter_initialized_ = false;
    err_u_f_ = 0.0f;
    err_v_f_ = 0.0f;
    derr_u_f_ = 0.0f;
    derr_v_f_ = 0.0f;
    prev_err_u_raw_ = 0.0f;
    prev_err_v_raw_ = 0.0f;
    align_in_window_ = false;
    align_hold_started_ = false;
    z_target_initialized_ = false;
  }

  void clearObservationTrackingState()
  {
    has_observation_ = false;
    current_observation_detected_ = false;
    new_valid_observation_ = false;
    consecutive_observation_miss_count_ = 0;
    last_observation_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    prev_observation_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_observation_ = uav_visual_landing::msg::TargetObservation{};
  }

  void clearTargetMemory()
  {
    target_memory_ = RelativeTarget3D{};
    target_memory_observed_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    target_memory_updated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void setHeightMeasurement(
    float height_m,
    const std::string & source,
    const rclcpp::Time & receive_stamp,
    const rclcpp::Time * sample_stamp = nullptr)
  {
    height_measurement_m_ = height_m;
    height_measurement_source_ = source;
    height_measurement_receive_stamp_ = receive_stamp;
    if (sample_stamp != nullptr && sample_stamp->nanoseconds() != 0) {
      height_measurement_sample_stamp_ = *sample_stamp;
      height_measurement_has_sample_stamp_ = true;
    } else {
      height_measurement_sample_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      height_measurement_has_sample_stamp_ = false;
    }
    has_height_measurement_ = true;
  }

  void onStampedHeightMeasurement(const sensor_msgs::msg::Range & msg)
  {
    if (!std::isfinite(msg.range)) {
      return;
    }
    const bool has_sample_stamp = (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
    if (!has_sample_stamp) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "ignoring stamped height measurement without header stamp");
      return;
    }
    const rclcpp::Time sample_stamp(msg.header.stamp);
    setHeightMeasurement(msg.range, "STAMPED_RANGE", this->now(), &sample_stamp);
  }

  void onDistanceSensorHeightMeasurement(const px4_msgs::msg::DistanceSensor & msg)
  {
    if (!std::isfinite(msg.current_distance)) {
      return;
    }
    setHeightMeasurement(msg.current_distance, "DISTANCE_SENSOR_RAW", this->now());
  }

  void onVehicleLocalPositionHeightMeasurement(const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    if (!msg.dist_bottom_valid || !std::isfinite(msg.dist_bottom)) {
      return;
    }
    setHeightMeasurement(
      msg.dist_bottom,
      describeDistBottomSource(msg.dist_bottom_sensor_bitfield),
      this->now());
  }

  void onObservation(const uav_visual_landing::msg::TargetObservation & msg)
  {
    const bool has_stamp = (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg.header.stamp) : this->now();
    const bool observation_valid =
      msg.detected && std::isfinite(msg.err_u_norm) && std::isfinite(msg.err_v_norm);

    current_observation_detected_ = observation_valid;
    new_valid_observation_ = observation_valid;
    if (!observation_valid) {
      if (has_observation_) {
        ++consecutive_observation_miss_count_;
      }
      return;
    }

    last_observation_ = msg;
    if (!std::isfinite(last_observation_.yaw_err_rad)) {
      last_observation_.yaw_err_rad = 0.0f;
      last_observation_.pose_valid = false;
    }
    last_observation_time_ = stamp;
    has_observation_ = true;
    consecutive_observation_miss_count_ = 0;

    if (!filter_initialized_) {
      err_u_f_ = msg.err_u_norm;
      err_v_f_ = msg.err_v_norm;
      derr_u_f_ = 0.0f;
      derr_v_f_ = 0.0f;
      prev_err_u_raw_ = msg.err_u_norm;
      prev_err_v_raw_ = msg.err_v_norm;
      prev_observation_stamp_ = stamp;
      filter_initialized_ = true;
      return;
    }

    err_u_f_ = lerp(err_u_f_, msg.err_u_norm, xy_error_alpha_);
    err_v_f_ = lerp(err_v_f_, msg.err_v_norm, xy_error_alpha_);

    const float dt_s = static_cast<float>((stamp - prev_observation_stamp_).seconds());
    float raw_rate_u = 0.0f;
    float raw_rate_v = 0.0f;
    if (computeLimitedRate(
        msg.err_u_norm, prev_err_u_raw_, dt_s,
        xy_error_rate_dt_min_s_, xy_error_rate_dt_max_s_, xy_error_rate_limit_, raw_rate_u))
    {
      derr_u_f_ = lerp(derr_u_f_, raw_rate_u, xy_error_rate_alpha_);
    }
    if (computeLimitedRate(
        msg.err_v_norm, prev_err_v_raw_, dt_s,
        xy_error_rate_dt_min_s_, xy_error_rate_dt_max_s_, xy_error_rate_limit_, raw_rate_v))
    {
      derr_v_f_ = lerp(derr_v_f_, raw_rate_v, xy_error_rate_alpha_);
    }

    prev_err_u_raw_ = msg.err_u_norm;
    prev_err_v_raw_ = msg.err_v_norm;
    prev_observation_stamp_ = stamp;
  }

  void requestHold()
  {
    if (!hold_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "hold service not ready");
      return;
    }
    auto req = std::make_shared<Trigger::Request>();
    hold_client_->async_send_request(req);
  }

  void resetDisarmRequestState()
  {
    disarm_request_in_flight_ = false;
    disarm_request_failed_ = false;
    active_disarm_request_id_ = 0;
    last_disarm_request_attempt_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void finishAfterLandingComplete()
  {
    active_ = false;
    resetDisarmRequestState();
    clearObservationTrackingState();
    clearVisionTrackingState();
    clearTargetMemory();
    align_hold_started_ = false;
    terminal_trigger_source_ = "NONE";
    height_measurement_sample_fresh_ = false;
    height_measurement_receive_fresh_ = false;
    land_phase_armed_seen_ = false;
    land_disarm_condition_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    xy_control_mode_ = "idle";
    phase_ = ControllerPhase::Ready;
  }

  void handleDisarmResponse(uint64_t request_id, rclcpp::Client<Trigger>::SharedFuture future)
  {
    if (request_id != active_disarm_request_id_ || phase_ != ControllerPhase::Land) {
      return;
    }

    disarm_request_in_flight_ = false;
    active_disarm_request_id_ = 0;
    try {
      const auto response = future.get();
      if (!response->success) {
        disarm_request_failed_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "offboard-land disarm request rejected: %s",
          response->message.c_str());
        return;
      }
    } catch (const std::exception & error) {
      disarm_request_failed_ = true;
      RCLCPP_WARN(
        this->get_logger(), "offboard-land disarm request failed: %s", error.what());
    }
  }

  void requestDisarm()
  {
    if (disarm_request_in_flight_) {
      return;
    }
    if (has_vehicle_status_ && !is_armed_) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (last_disarm_request_attempt_.nanoseconds() != 0 &&
      (now - last_disarm_request_attempt_).seconds() < disarm_request_retry_delay_s_)
    {
      return;
    }

    last_disarm_request_attempt_ = now;
    disarm_request_failed_ = false;
    if (!disarm_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "disarm service not ready");
      disarm_request_failed_ = true;
      return;
    }

    const uint64_t request_id = ++disarm_request_sequence_;
    active_disarm_request_id_ = request_id;
    disarm_request_in_flight_ = true;
    auto req = std::make_shared<Trigger::Request>();
    try {
      disarm_client_->async_send_request(
        req,
        [this, request_id](rclcpp::Client<Trigger>::SharedFuture future)
        {
          handleDisarmResponse(request_id, future);
        });
    } catch (const std::exception & error) {
      disarm_request_in_flight_ = false;
      active_disarm_request_id_ = 0;
      disarm_request_failed_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "failed to queue offboard-land disarm request: %s",
        error.what());
    }
  }

  void transitionTo(ControllerPhase phase, bool invoke_hold)
  {
    if (phase_ == phase) {
      return;
    }

    const ControllerPhase previous_phase = phase_;
    phase_ = phase;
    align_hold_started_ = false;
    if (phase == ControllerPhase::Terminal || phase == ControllerPhase::Land) {
      z_target_initialized_ = false;
    }
    if (phase == ControllerPhase::Land) {
      resetDisarmRequestState();
      land_phase_armed_seen_ = has_vehicle_status_ && is_armed_;
      land_disarm_condition_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (phase == ControllerPhase::DescendTrack || phase == ControllerPhase::Terminal ||
      phase == ControllerPhase::Land)
    {
      clearTargetMemory();
    }
    if (phase == ControllerPhase::Ready || phase == ControllerPhase::HoldWait) {
      clearVisionTrackingState();
    }
    if (invoke_hold && (phase == ControllerPhase::Ready || phase == ControllerPhase::HoldWait)) {
      requestHold();
    }
    RCLCPP_INFO(
      this->get_logger(), "[visual_landing] phase -> %s -> %s",
      phaseName(previous_phase), phaseName(phase_));
  }

  bool observationFresh() const
  {
    const double age_s =
      has_observation_ ? (this->now() - last_observation_time_).seconds() : -1.0;
    return hasFreshTrackingObservation(
      has_observation_, age_s, observation_timeout_s_,
      consecutive_observation_miss_count_, observation_miss_frames_);
  }

  bool stateFresh() const
  {
    if (!has_state_ || last_state_time_.nanoseconds() == 0) {
      return false;
    }

    const double age_s = (this->now() - last_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= state_timeout_s_;
  }

  bool targetMemoryFresh() const
  {
    if (!target_memory_.valid || target_memory_observed_time_.nanoseconds() == 0) {
      return false;
    }

    const double age_s = (this->now() - target_memory_observed_time_).seconds();
    return age_s >= 0.0 && age_s <= target_memory_timeout_s_;
  }

  void updateTargetMemory(float control_height_m)
  {
    const float fallback_depth_m = std::isfinite(control_height_m) ? control_height_m : 0.0f;
    const float depth_m =
      (last_observation_.tag_depth_valid && std::isfinite(last_observation_.tag_depth_m) &&
      last_observation_.tag_depth_m > 1.0e-6f) ? last_observation_.tag_depth_m : fallback_depth_m;
    const float err_u_norm = filter_initialized_ ? err_u_f_ : last_observation_.err_u_norm;
    const float err_v_norm = filter_initialized_ ? err_v_f_ : last_observation_.err_v_norm;
    const RelativeTarget3D target = computeRelativeTarget3D(
      err_u_norm, err_v_norm, depth_m,
      last_observation_.pose_valid, last_observation_.yaw_err_rad);
    if (!target.valid) {
      return;
    }

    target_memory_ = target;
    target_memory_observed_time_ = last_observation_time_;
    target_memory_updated_time_ = this->now();
  }

  void propagateTargetMemory()
  {
    if (!target_memory_.valid) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (target_memory_updated_time_.nanoseconds() == 0) {
      target_memory_updated_time_ = now;
      return;
    }

    const float dt_s = static_cast<float>((now - target_memory_updated_time_).seconds());
    advanceRelativeTarget(
      target_memory_, current_body_vx_, current_body_vy_, current_body_vz_,
      current_yaw_rate_,
      dt_s);
    target_memory_updated_time_ = now;
  }

  double heightMeasurementAgeS(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return -1.0;
    }
    return (this->now() - stamp).seconds();
  }

  HeightDecision heightDecision() const
  {
    const rclcpp::Time active_stamp =
      usesSampleTimeForHeightMeasurement(height_measurement_transport_) ?
      height_measurement_sample_stamp_ :
      height_measurement_receive_stamp_;
    const double age_s = has_height_measurement_ ? heightMeasurementAgeS(active_stamp) : -1.0;
    return evaluateHeightDecision(
      odom_height_m_, has_height_measurement_, height_measurement_m_,
      age_s, height_config_);
  }

  bool aligned(const MetricLateralError & lateral_error) const
  {
    if (!filter_initialized_) {
      return false;
    }
    return isAligned(
      lateral_error, last_observation_.pose_valid, last_observation_.yaw_err_rad,
      align_in_window_,
      alignment_config_);
  }

  bool heightMeasurementFreshAtStamp(const rclcpp::Time & stamp, float & height_m) const
  {
    if (!has_height_measurement_ || !std::isfinite(height_measurement_m_)) {
      return false;
    }

    const double age_s = heightMeasurementAgeS(stamp);
    if (age_s < 0.0 || age_s > height_config_.timeout_s) {
      return false;
    }
    if (height_measurement_m_ < height_config_.min_m ||
      height_measurement_m_ > height_config_.max_m)
    {
      return false;
    }

    height_m = height_measurement_m_;
    return true;
  }

  bool heightMeasurementSampleFresh(float & height_m) const
  {
    if (!height_measurement_has_sample_stamp_) {
      return false;
    }
    return heightMeasurementFreshAtStamp(height_measurement_sample_stamp_, height_m);
  }

  bool heightMeasurementReceiveFresh(float & height_m) const
  {
    return heightMeasurementFreshAtStamp(height_measurement_receive_stamp_, height_m);
  }

  bool nearSearchHeight(float control_height_m) const
  {
    return std::isfinite(control_height_m) &&
           std::abs(control_height_m - search_height_m_) <= search_height_tolerance_m_;
  }

  bool withinTerminalHeightBand(const HeightDecision & height_decision, float control_height_m) const
  {
    return height_decision.height_valid && std::isfinite(control_height_m) &&
           std::abs(control_height_m - terminal_entry_height_m_) <= terminal_height_band_m_;
  }

  bool terminalAlignmentSatisfied(const MetricLateralError & lateral_error) const
  {
    return lateral_error.valid && std::isfinite(lateral_error.norm_m) &&
           lateral_error.norm_m <= terminal_align_lateral_m_ &&
           last_observation_.pose_valid && std::isfinite(last_observation_.yaw_err_rad) &&
           std::abs(last_observation_.yaw_err_rad) <= terminal_align_yaw_rad_;
  }

  bool terminalLandYawSettled() const
  {
    return last_observation_.pose_valid &&
           std::isfinite(last_observation_.yaw_err_rad) &&
           std::abs(last_observation_.yaw_err_rad) <= yaw_deadband_rad_ &&
           std::isfinite(current_yaw_rate_) &&
           std::abs(current_yaw_rate_) <= terminal_land_max_yaw_rate_radps_;
  }

  bool terminalLandReady(const MetricLateralError & lateral_error) const
  {
    return lateral_error.valid &&
           std::isfinite(lateral_error.norm_m) &&
           lateral_error.norm_m <= terminal_align_lateral_m_ &&
           terminalLandYawSettled();
  }

  float offboardLandDescentSpeed() const
  {
    return offboard_land_vz_mps_;
  }

  bool offboardLandDisarmReady(float control_height_m) const
  {
    if (!std::isfinite(control_height_m) || control_height_m > offboard_land_disarm_height_m_) {
      return false;
    }
    if (has_vehicle_land_detected_ && !landed_) {
      return false;
    }
    if (!std::isfinite(current_body_vx_) || !std::isfinite(current_body_vy_) ||
      !std::isfinite(current_body_vz_))
    {
      return false;
    }

    const float xy_speed_mps = std::hypot(current_body_vx_, current_body_vy_);
    return xy_speed_mps <= offboard_land_disarm_max_xy_mps_ &&
           std::abs(current_body_vz_) <= offboard_land_disarm_max_vz_mps_;
  }

  void publishVelocityBody(float vx, float vy, float vz, float yaw_rate)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "uav_base_link";
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    msg.twist.angular.z = yaw_rate;
    velocity_pub_->publish(msg);
  }

  void publishControllerState(
    bool force,
    const HeightDecision & height_decision,
    float control_height_m,
    const MetricLateralError & lateral_error,
    float cmd_vx,
    float cmd_vy,
    float cmd_vz,
    float cmd_yaw_rate)
  {
    const rclcpp::Time now = this->now();
    if (!force && status_initialized_ && ((now - last_status_time_).seconds() < status_period_s_)) {
      return;
    }

    const float z_error = z_target_initialized_ ? (z_target_height_m_ - control_height_m) : 0.0f;

    uav_visual_landing::msg::LandingControllerState msg;
    msg.header.stamp = now;
    msg.active = active_;
    msg.phase = phaseName(phase_);
    msg.target_detected = current_observation_detected_ && has_observation_ &&
      ((now - last_observation_time_).seconds() <= observation_timeout_s_);
    msg.observation_age_s =
      has_observation_ ? static_cast<float>((now - last_observation_time_).seconds()) : -1.0f;
    msg.target_confidence = has_observation_ ? last_observation_.confidence : 0.0f;
    msg.height_source = heightSourceName(height_decision.height_source);
    msg.terminal_trigger_source = terminal_trigger_source_;
    msg.odom_height_m = odom_height_m_;
    msg.height_valid = height_decision.height_valid;
    msg.height_measurement_source = has_height_measurement_ ? height_measurement_source_ : "NONE";
    msg.height_measurement_time_basis =
      has_height_measurement_ ? height_measurement_time_basis_ : "NONE";
    msg.height_measurement_sample_fresh = height_measurement_sample_fresh_;
    msg.height_measurement_receive_fresh = height_measurement_receive_fresh_;
    msg.height_measurement_m = has_height_measurement_ ? height_measurement_m_ : 0.0f;
    msg.control_height_m = control_height_m;
    msg.tag_depth_valid = has_observation_ && last_observation_.tag_depth_valid;
    msg.tag_depth_m =
      (has_observation_ &&
      last_observation_.tag_depth_valid) ? last_observation_.tag_depth_m : 0.0f;
    msg.align_enter_lateral_m = alignment_config_.align_enter_lateral_m;
    msg.align_exit_lateral_m = alignment_config_.align_exit_lateral_m;
    msg.active_max_vxy = max_vxy_;
    msg.err_u_norm_filtered = err_u_f_;
    msg.err_v_norm_filtered = err_v_f_;
    msg.err_u_rate_norm_s = derr_u_f_;
    msg.err_v_rate_norm_s = derr_v_f_;
    msg.lateral_error_valid = lateral_error.valid;
    msg.lateral_error_x_m = lateral_error.x_m;
    msg.lateral_error_y_m = lateral_error.y_m;
    msg.lateral_error_m = lateral_error.norm_m;
    msg.lateral_error_rate_x_mps = lateral_error.x_rate_mps;
    msg.lateral_error_rate_y_mps = lateral_error.y_rate_mps;
    msg.z_target_height_m = z_target_initialized_ ? z_target_height_m_ : control_height_m;
    msg.z_error_m = z_error;
    msg.xy_control_mode = xy_control_mode_;
    msg.cmd_vx = cmd_vx;
    msg.cmd_vy = cmd_vy;
    msg.cmd_vz = cmd_vz;
    msg.cmd_yaw_rate = cmd_yaw_rate;
    controller_state_pub_->publish(msg);
    last_status_time_ = now;
    status_initialized_ = true;
  }

  void computeTrackingCommand(
    const MetricLateralError & lateral_error,
    bool yaw_valid,
    float yaw_err_rad,
    float & vx,
    float & vy,
    float & yaw_rate) const
  {
    vx = 0.0f;
    vy = 0.0f;
    if (lateral_error.valid) {
      vx = -((kp_xy_m_ * lateral_error.y_m) + (kd_xy_m_ * lateral_error.y_rate_mps));
      vy = -((kp_xy_m_ * lateral_error.x_m) + (kd_xy_m_ * lateral_error.x_rate_mps));
      const float vxy_norm = std::sqrt((vx * vx) + (vy * vy));
      if (vxy_norm > max_vxy_ && vxy_norm > 1.0e-6f) {
        const float scale = max_vxy_ / vxy_norm;
        vx *= scale;
        vy *= scale;
      }
    }

    if (!yaw_valid || !std::isfinite(yaw_err_rad) || std::abs(yaw_err_rad) < yaw_deadband_rad_) {
      yaw_rate = 0.0f;
    } else {
      yaw_rate = clamp(
        (-(kp_yaw_ * yaw_err_rad)) - (vel_damping_yaw_ * current_yaw_rate_),
        max_vyaw_);
    }
  }

  void setZTarget(float target_height_m)
  {
    z_target_height_m_ = target_height_m;
    z_target_initialized_ = true;
  }

  float computeZCommand(
    float current_height_m,
    float current_body_vz,
    float target_height_m,
    float kp,
    float max_vz) const
  {
    return computeClosedLoopVelocity(
      target_height_m,
      current_height_m,
      current_body_vz,
      kp,
      vel_damping_z_,
      max_vz);
  }

  void timerCallback()
  {
    const HeightDecision height_decision = heightDecision();
    const float control_height_m = height_decision.height_m;
    const bool state_fresh = stateFresh();
    float sample_height_measurement_m = 0.0f;
    float receive_height_measurement_m = 0.0f;
    height_measurement_sample_fresh_ = heightMeasurementSampleFresh(sample_height_measurement_m);
    height_measurement_receive_fresh_ = heightMeasurementReceiveFresh(receive_height_measurement_m);
    const bool terminal_height_measurement_fresh =
      supportsTerminalHeightTrigger(height_measurement_transport_) &&
      height_measurement_sample_fresh_;
    const bool fresh_observation = observationFresh();
    const MetricLateralError lateral_error = (fresh_observation && filter_initialized_) ?
      computeMetricLateralError(err_u_f_, err_v_f_, derr_u_f_, derr_v_f_, control_height_m) :
      MetricLateralError{};
    if (!initial_search_completed_) {
      if (fresh_observation && new_valid_observation_) {
        updateTargetMemory(control_height_m);
      } else {
        propagateTargetMemory();
      }
    }
    new_valid_observation_ = false;

    const bool currently_aligned = fresh_observation && aligned(lateral_error);
    align_in_window_ = currently_aligned;
    xy_control_mode_ = "idle";
    if (phase_ != ControllerPhase::Terminal && phase_ != ControllerPhase::Land) {
      terminal_trigger_source_ = "NONE";
    }

    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_vz = 0.0f;
    float cmd_yaw_rate = 0.0f;

    if (!active_ || !has_state_) {
      publishControllerState(
        false, height_decision, control_height_m, lateral_error, 0.0f, 0.0f,
        0.0f, 0.0f);
      return;
    }

    const bool allow_terminal_without_fresh_state =
      phase_ == ControllerPhase::Terminal || phase_ == ControllerPhase::Land;
    if (!state_fresh && !allow_terminal_without_fresh_state) {
      if (phase_ != ControllerPhase::HoldWait && phase_ != ControllerPhase::Ready) {
        transitionTo(ControllerPhase::HoldWait, true);
      }
      xy_control_mode_ = "state_stale";
      publishControllerState(
        false, height_decision, control_height_m, lateral_error, 0.0f, 0.0f,
        0.0f, 0.0f);
      return;
    } else if (!state_fresh && allow_terminal_without_fresh_state) {
      xy_control_mode_ = "terminal_state_stale_bypass";
    }

    switch (phase_) {
      case ControllerPhase::Ready:
        break;
      case ControllerPhase::HoldWait:
        if (fresh_observation) {
          transitionTo(ControllerPhase::TrackAlign, false);
          break;
        }
        if (!initial_search_completed_) {
          setZTarget(search_height_m_);
          cmd_vz = computeZCommand(
            control_height_m, current_body_vz_, z_target_height_m_, z_hold_kp_,
            z_hold_max_vz_);
        }
        if (!initial_search_completed_ && targetMemoryFresh()) {
          const MetricLateralError memory_error = lateralErrorFromRelativeTarget(target_memory_);
          if (memory_error.valid) {
            computeTrackingCommand(
              memory_error, target_memory_.yaw_valid, target_memory_.yaw_err_rad,
              cmd_vx, cmd_vy, cmd_yaw_rate);
            xy_control_mode_ = "memory_search_pd";
            publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
            break;
          }
        }
        if (!initial_search_completed_) {
          xy_control_mode_ = "vertical_search_hold";
          publishVelocityBody(0.0f, 0.0f, cmd_vz, 0.0f);
        }
        break;
      case ControllerPhase::TrackAlign:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!initial_search_completed_) {
          setZTarget(search_height_m_);
        } else if (!z_target_initialized_) {
          setZTarget(control_height_m);
        }
        computeTrackingCommand(
          lateral_error, last_observation_.pose_valid, last_observation_.yaw_err_rad,
          cmd_vx, cmd_vy,
          cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_, z_hold_kp_,
          z_hold_max_vz_);
        xy_control_mode_ = "vision_pd_metric";
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (currently_aligned &&
          (initial_search_completed_ || nearSearchHeight(control_height_m)))
        {
          transitionTo(ControllerPhase::HoldVerify, false);
        }
        break;
      case ControllerPhase::HoldVerify:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!currently_aligned ||
          (!initial_search_completed_ && !nearSearchHeight(control_height_m)))
        {
          transitionTo(ControllerPhase::TrackAlign, false);
          break;
        }
        if (!initial_search_completed_) {
          setZTarget(search_height_m_);
        } else if (!z_target_initialized_) {
          setZTarget(control_height_m);
        }
        computeTrackingCommand(
          lateral_error, last_observation_.pose_valid, last_observation_.yaw_err_rad,
          cmd_vx, cmd_vy,
          cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_, z_hold_kp_,
          z_hold_max_vz_);
        xy_control_mode_ = "vision_pd_metric";
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (!align_hold_started_) {
          align_hold_started_ = true;
          align_hold_since_ = this->now();
        } else if ((this->now() - align_hold_since_).seconds() >= hold_verify_s_) {
          initial_search_completed_ = true;
          transitionTo(ControllerPhase::DescendTrack, false);
        }
        break;
      case ControllerPhase::DescendTrack:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (
          current_observation_detected_ &&
          terminal_height_measurement_fresh &&
          withinTerminalHeightBand(height_decision, control_height_m) &&
          terminalAlignmentSatisfied(lateral_error))
        {
          if (!align_hold_started_) {
            align_hold_started_ = true;
            align_hold_since_ = this->now();
          } else if ((this->now() - align_hold_since_).seconds() >= terminal_confirm_s_) {
            terminal_trigger_source_ = "HEIGHT_BAND_TERMINAL";
            transitionTo(ControllerPhase::Terminal, false);
            break;
          }
        } else {
          align_hold_started_ = false;
        }
        if (!z_target_initialized_) {
          setZTarget(control_height_m);
        }
        z_target_height_m_ =
          std::max(terminal_entry_height_m_, z_target_height_m_ - (descend_speed_ * control_dt_s_));
        computeTrackingCommand(
          lateral_error, last_observation_.pose_valid, last_observation_.yaw_err_rad,
          cmd_vx, cmd_vy,
          cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_,
          z_descend_kp_, z_descend_max_vz_);
        xy_control_mode_ = "vision_pd_metric";
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        break;
      case ControllerPhase::Terminal:
        if (terminalLandReady(lateral_error)) {
          if (!align_hold_started_) {
            align_hold_started_ = true;
            align_hold_since_ = this->now();
          } else if ((this->now() - align_hold_since_).seconds() >= hold_verify_s_) {
            terminal_trigger_source_ =
              (terminal_height_measurement_fresh &&
              std::isfinite(sample_height_measurement_m) &&
              sample_height_measurement_m <= terminal_land_height_m_) ?
              "TERMINAL_HEIGHT_SETTLED_OFFBOARD_LAND" :
              "TERMINAL_ALIGN_SETTLED_OFFBOARD_LAND";
            transitionTo(ControllerPhase::Land, false);
            break;
          }
        } else {
          align_hold_started_ = false;
        }
        if (terminal_height_measurement_fresh &&
          std::isfinite(sample_height_measurement_m) &&
          sample_height_measurement_m <= terminal_land_height_m_)
        {
          terminal_trigger_source_ = "TERMINAL_HEIGHT_WAIT_YAW_SETTLE";
        }
        if (!fresh_observation) {
          terminal_trigger_source_ = "TERMINAL_LOST_OBS_OFFBOARD_LAND";
          transitionTo(ControllerPhase::Land, false);
          break;
        }
        if (!z_target_initialized_) {
          // Hold the current terminal height while continuing visual XY/yaw alignment
          // before switching to the offboard landing descent profile.
          setZTarget(control_height_m);
        }
        computeTrackingCommand(
          lateral_error, last_observation_.pose_valid, last_observation_.yaw_err_rad,
          cmd_vx, cmd_vy,
          cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_,
          z_hold_kp_, z_terminal_max_vz_);
        xy_control_mode_ = "terminal_vision_hold";
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        break;
      case ControllerPhase::Land:
        if (fresh_observation) {
          computeTrackingCommand(
            lateral_error, last_observation_.pose_valid, last_observation_.yaw_err_rad,
            cmd_vx, cmd_vy,
            cmd_yaw_rate);
          cmd_yaw_rate = clamp(cmd_yaw_rate, terminal_land_max_yaw_rate_radps_);
          xy_control_mode_ = "offboard_land_vision_track";
        } else {
          xy_control_mode_ = "offboard_land_vertical_only";
        }
        cmd_vz = -offboardLandDescentSpeed();
        if (offboardLandDisarmReady(control_height_m)) {
          if (land_disarm_condition_since_.nanoseconds() == 0) {
            land_disarm_condition_since_ = this->now();
          } else if ((this->now() - land_disarm_condition_since_).seconds() >=
            offboard_land_disarm_hold_s_)
          {
            xy_control_mode_ = fresh_observation ?
              "offboard_land_disarm_wait_vision" :
              "offboard_land_disarm_wait_vertical";
            requestDisarm();
          }
        } else {
          land_disarm_condition_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        break;
    }

    publishControllerState(
      false, height_decision, control_height_m, lateral_error, cmd_vx, cmd_vy,
      cmd_vz, cmd_yaw_rate);
  }

  std::string target_observation_topic_;
  std::string state_topic_;
  std::string velocity_body_topic_;
  std::string controller_state_topic_;
  std::string hold_service_;
  std::string disarm_service_;
  HeightMeasurementTransport height_measurement_transport_{HeightMeasurementTransport::StampedRange};
  std::string height_measurement_topic_;
  std::string range_topic_;
  std::string vehicle_land_detected_topic_;
  std::string vehicle_local_position_topic_;
  std::string vehicle_status_topic_;
  std::string start_service_name_;
  std::string stop_service_name_;
  HeightConfig height_config_{};
  AlignmentConfig alignment_config_{};
  float kp_xy_m_{0.80f};
  float kd_xy_m_{0.15f};
  float max_vxy_{0.40f};
  float xy_error_alpha_{0.35f};
  float xy_error_rate_alpha_{0.25f};
  float xy_error_rate_limit_{2.0f};
  float xy_error_rate_dt_min_s_{0.005f};
  float xy_error_rate_dt_max_s_{0.20f};
  float kp_yaw_{1.2f};
  float max_vyaw_{0.6f};
  float vel_damping_yaw_{0.18f};
  float yaw_deadband_rad_{0.03f};
  double hold_verify_s_{0.5};
  float search_height_m_{1.0f};
  float search_height_tolerance_m_{0.10f};
  float descend_speed_{0.12f};
  float terminal_entry_height_m_{0.40f};
  float terminal_height_band_m_{0.03f};
  float terminal_align_lateral_m_{0.05f};
  float terminal_align_yaw_rad_{0.15f};
  double terminal_confirm_s_{2.0};
  float terminal_land_height_m_{0.20f};
  float terminal_land_max_yaw_rate_radps_{0.05f};
  float offboard_land_vz_mps_{0.40f};
  float offboard_land_disarm_height_m_{0.20f};
  double offboard_land_disarm_hold_s_{0.25};
  float offboard_land_disarm_max_xy_mps_{0.25f};
  float offboard_land_disarm_max_vz_mps_{0.25f};
  float z_hold_kp_{1.2f};
  float z_hold_max_vz_{0.18f};
  float z_descend_kp_{1.3f};
  float z_descend_max_vz_{0.25f};
  float z_terminal_max_vz_{0.35f};
  float vel_damping_z_{0.18f};
  double observation_timeout_s_{0.30};
  int observation_miss_frames_{3};
  double state_timeout_s_{0.20};
  double target_memory_timeout_s_{0.50};
  bool active_{false};
  ControllerPhase phase_{ControllerPhase::Ready};
  bool align_in_window_{false};
  bool align_hold_started_{false};
  bool disarm_request_in_flight_{false};
  bool disarm_request_failed_{false};
  bool initial_search_completed_{false};
  bool current_observation_detected_{false};
  bool new_valid_observation_{false};
  bool has_observation_{false};
  bool has_state_{false};
  bool has_vehicle_status_{false};
  bool has_vehicle_land_detected_{false};
  bool is_armed_{false};
  bool land_phase_armed_seen_{false};
  bool ground_contact_{false};
  bool maybe_landed_{false};
  bool landed_{false};
  bool has_height_measurement_{false};
  bool height_measurement_has_sample_stamp_{false};
  bool filter_initialized_{false};
  bool height_measurement_sample_fresh_{false};
  bool height_measurement_receive_fresh_{false};
  float odom_height_m_{0.0f};
  float current_body_vx_{0.0f};
  float current_body_vy_{0.0f};
  float current_body_vz_{0.0f};
  float current_yaw_rate_{0.0f};
  float height_measurement_m_{0.0f};
  std::string height_measurement_source_{"NONE"};
  std::string height_measurement_time_basis_{"NONE"};
  float err_u_f_{0.0f};
  float err_v_f_{0.0f};
  float derr_u_f_{0.0f};
  float derr_v_f_{0.0f};
  float prev_err_u_raw_{0.0f};
  float prev_err_v_raw_{0.0f};
  float z_target_height_m_{0.0f};
  bool z_target_initialized_{false};
  float control_dt_s_{1.0f / 30.0f};
  double disarm_request_retry_delay_s_{0.5};
  double status_period_s_{0.2};
  bool status_initialized_{false};
  int consecutive_observation_miss_count_{0};
  uint64_t disarm_request_sequence_{0};
  uint64_t active_disarm_request_id_{0};
  std::string xy_control_mode_{"idle"};
  std::string terminal_trigger_source_{"NONE"};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_observation_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_disarm_request_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time height_measurement_receive_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time height_measurement_sample_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time align_hold_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time land_disarm_condition_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_memory_observed_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_memory_updated_time_{0, 0, RCL_ROS_TIME};
  uav_visual_landing::msg::TargetObservation last_observation_{};
  RelativeTarget3D target_memory_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<uav_visual_landing::msg::LandingControllerState>::SharedPtr
    controller_state_pub_;
  rclcpp::Subscription<uav_visual_landing::msg::TargetObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr stamped_height_measurement_sub_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr
    legacy_distance_sensor_height_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    vehicle_land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_height_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Client<Trigger>::SharedPtr hold_client_;
  rclcpp::Client<Trigger>::SharedPtr disarm_client_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_visual_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_visual_landing::VisualLandingNode>());
  rclcpp::shutdown();
  return 0;
}
