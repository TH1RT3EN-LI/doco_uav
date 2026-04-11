#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "uav_bridge/math_utils.hpp"
#include "uav_bridge/openvins_ev_guard.hpp"
#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{

class OpenVinsPx4VisionBridgeNode : public rclcpp::Node
{
public:
  OpenVinsPx4VisionBridgeNode()
  : Node("openvins_px4_vision_bridge_node"),
    guard_(guard_config_)
  {
    this->declare_parameter<std::string>("odometry_topic", "/ov_msckf/odomimu");
    this->declare_parameter<std::string>(
      "visual_odometry_topic",
      "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("expected_odom_frame_id", "global");
    this->declare_parameter<std::string>("expected_child_frame_id", "imu");
    this->declare_parameter<double>("sensor_x_in_body_m", 0.0);
    this->declare_parameter<double>("sensor_y_in_body_m", 0.0);
    this->declare_parameter<double>("sensor_z_in_body_m", 0.0);
    this->declare_parameter<double>("sensor_roll_in_body_rad", 0.0);
    this->declare_parameter<double>("sensor_pitch_in_body_rad", 0.0);
    this->declare_parameter<double>("sensor_yaw_in_body_rad", 0.0);
    this->declare_parameter<double>("position_variance_floor_m2", 0.01);
    this->declare_parameter<double>("orientation_variance_floor_rad2", 0.01);
    this->declare_parameter<double>("velocity_variance_floor_m2ps2", 0.01);
    this->declare_parameter<int>("quality", 100);
    this->declare_parameter<bool>("use_odometry_stamp_for_timestamp_sample", true);
    this->declare_parameter<bool>("log_debug", false);
    this->declare_parameter<std::string>("reset_counter_bump_topic", "");
    this->declare_parameter<std::string>("timesync_status_topic", "/fmu/out/timesync_status");
    this->declare_parameter<std::string>("px4_timestamp_source", "system");
    this->declare_parameter<double>("timesync_timeout_s", 0.5);
    this->declare_parameter<std::string>("health_ok_topic", "/uav/ov/bridge/health_ok");
    this->declare_parameter<std::string>("fault_reason_topic", "/uav/ov/bridge/fault_reason");
    this->declare_parameter<bool>("guard_enable", true);
    this->declare_parameter<bool>("guard_auto_recover", true);
    this->declare_parameter<double>("guard_nominal_rate_hz", 30.0);
    this->declare_parameter<double>("guard_hold_last_budget_s", 0.12);
    this->declare_parameter<int>("guard_recovery_good_frames", 5);
    this->declare_parameter<double>("guard_max_source_gap_s", 0.20);
    this->declare_parameter<double>("guard_max_position_step_m", 0.20);
    this->declare_parameter<double>("guard_max_implied_speed_mps", 1.0);
    this->declare_parameter<double>("guard_max_reported_speed_mps", 1.0);
    this->declare_parameter<double>("guard_max_accel_mps2", 2.5);
    this->declare_parameter<double>("guard_max_yaw_rate_radps", 2.0);

    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    visual_odometry_topic_ = this->get_parameter("visual_odometry_topic").as_string();
    expected_odom_frame_id_ = this->get_parameter("expected_odom_frame_id").as_string();
    expected_child_frame_id_ = this->get_parameter("expected_child_frame_id").as_string();
    sensor_position_in_body_.setValue(
      this->get_parameter("sensor_x_in_body_m").as_double(),
      this->get_parameter("sensor_y_in_body_m").as_double(),
      this->get_parameter("sensor_z_in_body_m").as_double());
    sensor_rotation_in_body_.setRPY(
      this->get_parameter("sensor_roll_in_body_rad").as_double(),
      this->get_parameter("sensor_pitch_in_body_rad").as_double(),
      this->get_parameter("sensor_yaw_in_body_rad").as_double());
    sensor_rotation_in_body_.normalize();
    position_variance_floor_m2_ = static_cast<float>(
      std::max(0.0, this->get_parameter("position_variance_floor_m2").as_double()));
    orientation_variance_floor_rad2_ = static_cast<float>(
      std::max(0.0, this->get_parameter("orientation_variance_floor_rad2").as_double()));
    velocity_variance_floor_m2ps2_ = static_cast<float>(
      std::max(0.0, this->get_parameter("velocity_variance_floor_m2ps2").as_double()));
    quality_ = std::clamp(static_cast<int>(this->get_parameter("quality").as_int()), 0, 100);
    use_odometry_stamp_for_timestamp_sample_ =
      this->get_parameter("use_odometry_stamp_for_timestamp_sample").as_bool();
    log_debug_ = this->get_parameter("log_debug").as_bool();
    reset_counter_bump_topic_ = this->get_parameter("reset_counter_bump_topic").as_string();
    timesync_status_topic_ = this->get_parameter("timesync_status_topic").as_string();
    px4_timestamp_source_ = parsePx4TimestampSource(
      this->get_parameter("px4_timestamp_source").as_string());
    timesync_timeout_s_ = std::max(0.0, this->get_parameter("timesync_timeout_s").as_double());
    health_ok_topic_ = this->get_parameter("health_ok_topic").as_string();
    fault_reason_topic_ = this->get_parameter("fault_reason_topic").as_string();

    guard_config_.enable = this->get_parameter("guard_enable").as_bool();
    guard_config_.auto_recover = this->get_parameter("guard_auto_recover").as_bool();
    guard_config_.nominal_rate_hz = std::max(
      1.0, this->get_parameter("guard_nominal_rate_hz").as_double());
    guard_config_.hold_last_budget_s = std::max(
      0.0, this->get_parameter("guard_hold_last_budget_s").as_double());
    guard_config_.recovery_good_frames = std::max(
      1, static_cast<int>(this->get_parameter("guard_recovery_good_frames").as_int()));
    guard_config_.max_source_gap_s = std::max(
      0.0, this->get_parameter("guard_max_source_gap_s").as_double());
    guard_config_.max_position_step_m = static_cast<float>(std::max(
      0.0, this->get_parameter("guard_max_position_step_m").as_double()));
    guard_config_.max_implied_speed_mps = static_cast<float>(std::max(
      0.0, this->get_parameter("guard_max_implied_speed_mps").as_double()));
    guard_config_.max_reported_speed_mps = static_cast<float>(std::max(
      0.0, this->get_parameter("guard_max_reported_speed_mps").as_double()));
    guard_config_.max_accel_mps2 = static_cast<float>(std::max(
      0.0, this->get_parameter("guard_max_accel_mps2").as_double()));
    guard_config_.max_yaw_rate_radps = static_cast<float>(std::max(
      0.0, this->get_parameter("guard_max_yaw_rate_radps").as_double()));
    guard_ = OpenVinsEvGuard(guard_config_);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OpenVinsPx4VisionBridgeNode::odometryCallback, this, std::placeholders::_1));

    if (!reset_counter_bump_topic_.empty()) {
      reset_counter_bump_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        reset_counter_bump_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Empty::SharedPtr)
        {
          handleExternalResetCounterBump("external reset counter bump");
        });
    }

    if (px4_timestamp_source_ == Px4TimestampSource::Px4Timesync) {
      timesync_status_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        timesync_status_topic_, rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
        {
          estimated_px4_offset_us_ = msg->estimated_offset;
          last_timesync_status_time_ = this->now();
          has_timesync_status_ = true;
        });
    }

    visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
      visual_odometry_topic_, rclcpp::SensorDataQoS());

    const auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    health_ok_pub_ = this->create_publisher<std_msgs::msg::Bool>(health_ok_topic_, status_qos);
    fault_reason_pub_ = this->create_publisher<std_msgs::msg::String>(fault_reason_topic_, status_qos);

    if (sensor_position_in_body_.length2() > 1.0e-12) {
      RCLCPP_WARN(
        this->get_logger(),
        "sensor_xyz_in_body_m is ignored by openvins_px4_vision_bridge_node; "
        "configure EKF2_EV_POS_* on PX4 for EV lever-arm compensation");
    }

    publishGuardStatus(guard_.mode(), guard_.reason(), true);

    RCLCPP_INFO(
      this->get_logger(),
      "openvins_px4_vision_bridge_node: odom=%s -> %s, guard=%s hold_last_budget=%.3f s",
      odometry_topic_.c_str(),
      visual_odometry_topic_.c_str(),
      guard_config_.enable ? "enabled" : "disabled",
      guard_config_.hold_last_budget_s);
  }

private:
  static uint64_t timeToMicros(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<uint64_t>(stamp.sec) * 1000000ULL +
           static_cast<uint64_t>(stamp.nanosec / 1000U);
  }

  static uint64_t timeToMicros(const rclcpp::Time & stamp)
  {
    const int64_t nanoseconds = stamp.nanoseconds();
    if (nanoseconds <= 0) {
      return 0U;
    }
    return static_cast<uint64_t>(nanoseconds / 1000ULL);
  }

  static uint64_t applyOffsetMicros(uint64_t stamp_us, int64_t offset_us)
  {
    if (stamp_us == 0U) {
      return 0U;
    }
    if (offset_us >= 0) {
      return stamp_us + static_cast<uint64_t>(offset_us);
    }

    const uint64_t magnitude = static_cast<uint64_t>(-offset_us);
    return stamp_us > magnitude ? stamp_us - magnitude : 0U;
  }

  bool timesyncFresh() const
  {
    if (!has_timesync_status_) {
      return false;
    }
    if (timesync_timeout_s_ <= 0.0) {
      return true;
    }
    const double age_s = (this->now() - last_timesync_status_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= timesync_timeout_s_;
  }

  bool ensureTimestampSourceReady(const char * context)
  {
    if (px4_timestamp_source_ != Px4TimestampSource::Px4Timesync || timesyncFresh()) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "waiting for PX4 timesync before publishing %s",
      context);
    return false;
  }

  uint64_t nowPx4Micros()
  {
    const uint64_t ros_time_us = timeToMicros(this->get_clock()->now());
    const uint64_t system_time_us = timeToMicros(px4_timestamp_system_clock_.now());

    switch (px4_timestamp_source_) {
      case Px4TimestampSource::GazeboSim:
        return ros_time_us;

      case Px4TimestampSource::Px4Timesync:
        if (timesyncFresh()) {
          return applyOffsetMicros(system_time_us, estimated_px4_offset_us_);
        }
        return 0U;

      case Px4TimestampSource::System:
      default:
        return system_time_us;
    }
  }

  uint64_t stampToPx4Micros(const builtin_interfaces::msg::Time & stamp) const
  {
    const uint64_t stamp_us = timeToMicros(stamp);
    if (stamp_us == 0U) {
      return 0U;
    }

    switch (px4_timestamp_source_) {
      case Px4TimestampSource::GazeboSim:
        return stamp_us;

      case Px4TimestampSource::Px4Timesync:
        if (timesyncFresh()) {
          return applyOffsetMicros(stamp_us, estimated_px4_offset_us_);
        }
        return 0U;

      case Px4TimestampSource::System:
      default:
        return stamp_us;
    }
  }

  static std::array<float, 3> invalidVector3()
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return {nan, nan, nan};
  }

  static std::array<float, 4> invalidQuaternionArray()
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return {nan, nan, nan, nan};
  }

  static float covarianceOrFloor(double value, float floor)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      return floor;
    }
    return std::max(static_cast<float>(value), floor);
  }

  static bool hasHeaderStamp(const builtin_interfaces::msg::Time & stamp)
  {
    return stamp.sec != 0 || stamp.nanosec != 0;
  }

  void warnUnexpectedFrames(const nav_msgs::msg::Odometry & msg)
  {
    if (!expected_odom_frame_id_.empty() && !msg.header.frame_id.empty() &&
      msg.header.frame_id != expected_odom_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "unexpected odom header.frame_id: got '%s', expected '%s'",
        msg.header.frame_id.c_str(), expected_odom_frame_id_.c_str());
    }

    if (!expected_child_frame_id_.empty() && !msg.child_frame_id.empty() &&
      msg.child_frame_id != expected_child_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "unexpected odom child_frame_id: got '%s', expected '%s'",
        msg.child_frame_id.c_str(), expected_child_frame_id_.c_str());
    }
  }

  OpenVinsEvGuardSample makeGuardSample(
    const nav_msgs::msg::Odometry & msg,
    uint64_t receive_stamp_us) const
  {
    OpenVinsEvGuardSample sample;
    sample.receive_stamp_us = receive_stamp_us;
    sample.source_stamp_us = hasHeaderStamp(msg.header.stamp) ?
      timeToMicros(msg.header.stamp) : receive_stamp_us;
    sample.position_enu_m = {
      static_cast<float>(msg.pose.pose.position.x),
      static_cast<float>(msg.pose.pose.position.y),
      static_cast<float>(msg.pose.pose.position.z)};

    const auto & q = msg.pose.pose.orientation;
    const bool orientation_components_finite =
      std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
    const double orientation_norm_sq =
      (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
    sample.orientation_valid = orientation_components_finite &&
      std::isfinite(orientation_norm_sq) && orientation_norm_sq > 1.0e-12;
    if (sample.orientation_valid) {
      sample.yaw_enu_rad = quaternionToYaw(q);
      sample.orientation_valid = std::isfinite(sample.yaw_enu_rad);
    }

    sample.velocity_enu_mps = {
      static_cast<float>(msg.twist.twist.linear.x),
      static_cast<float>(msg.twist.twist.linear.y),
      static_cast<float>(msg.twist.twist.linear.z)};
    sample.velocity_valid = isFiniteVector(sample.velocity_enu_mps);
    return sample;
  }

  uint64_t computeTimestampSample(
    const nav_msgs::msg::Odometry & msg,
    uint64_t publish_timestamp_us) const
  {
    uint64_t timestamp_sample = 0U;
    if (use_odometry_stamp_for_timestamp_sample_) {
      timestamp_sample = stampToPx4Micros(msg.header.stamp);
    }
    if (timestamp_sample == 0U) {
      timestamp_sample = publish_timestamp_us;
    }
    return timestamp_sample;
  }

  px4_msgs::msg::VehicleOdometry buildVehicleOdometry(
    const nav_msgs::msg::Odometry & msg,
    uint64_t publish_timestamp_us,
    uint64_t timestamp_sample,
    uint8_t reset_counter) const
  {
    px4_msgs::msg::VehicleOdometry out{};
    out.timestamp = publish_timestamp_us;
    out.timestamp_sample = timestamp_sample;
    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    out.velocity = invalidVector3();
    out.angular_velocity = invalidVector3();
    out.q = invalidQuaternionArray();

    const tf2::Vector3 position_enu(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);

    const tf2::Quaternion orientation_enu_sensor(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);

    if (orientation_enu_sensor.length2() > 1.0e-12) {
      tf2::Quaternion q_enu_sensor = orientation_enu_sensor;
      q_enu_sensor.normalize();
      const tf2::Matrix3x3 rotation_enu_sensor(q_enu_sensor);
      const tf2::Matrix3x3 body_to_sensor(sensor_rotation_in_body_);
      const tf2::Matrix3x3 sensor_to_body = body_to_sensor.transpose();
      const tf2::Matrix3x3 rotation_enu_body = rotation_enu_sensor * sensor_to_body;

      const tf2::Matrix3x3 enu_to_ned(
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, -1.0);
      const tf2::Matrix3x3 frd_to_flu(
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0);

      const tf2::Matrix3x3 rotation_ned_frd = enu_to_ned * rotation_enu_body * frd_to_flu;
      tf2::Quaternion q_ned_frd;
      rotation_ned_frd.getRotation(q_ned_frd);
      q_ned_frd.normalize();
      out.q = {
        static_cast<float>(q_ned_frd.w()),
        static_cast<float>(q_ned_frd.x()),
        static_cast<float>(q_ned_frd.y()),
        static_cast<float>(q_ned_frd.z())};
    }

    out.position = enuPositionToNed(
      static_cast<float>(position_enu.x()),
      static_cast<float>(position_enu.y()),
      static_cast<float>(position_enu.z()));

    const std::array<float, 3> velocity_enu = {
      static_cast<float>(msg.twist.twist.linear.x),
      static_cast<float>(msg.twist.twist.linear.y),
      static_cast<float>(msg.twist.twist.linear.z)};
    if (isFiniteVector(velocity_enu)) {
      out.velocity = enuPositionToNed(velocity_enu[0], velocity_enu[1], velocity_enu[2]);
      out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    }

    out.position_variance = {
      covarianceOrFloor(msg.pose.covariance[0], position_variance_floor_m2_),
      covarianceOrFloor(msg.pose.covariance[7], position_variance_floor_m2_),
      covarianceOrFloor(msg.pose.covariance[14], position_variance_floor_m2_)};
    out.orientation_variance = {
      covarianceOrFloor(msg.pose.covariance[21], orientation_variance_floor_rad2_),
      covarianceOrFloor(msg.pose.covariance[28], orientation_variance_floor_rad2_),
      covarianceOrFloor(msg.pose.covariance[35], orientation_variance_floor_rad2_)};
    out.velocity_variance = {
      covarianceOrFloor(msg.twist.covariance[0], velocity_variance_floor_m2ps2_),
      covarianceOrFloor(msg.twist.covariance[7], velocity_variance_floor_m2ps2_),
      covarianceOrFloor(msg.twist.covariance[14], velocity_variance_floor_m2ps2_)};
    out.reset_counter = reset_counter;
    out.quality = static_cast<int8_t>(quality_);
    return out;
  }

  void publishGuardStatus(
    OpenVinsEvGuardMode mode,
    OpenVinsEvGuardReason reason,
    bool force)
  {
    const bool health_ok = mode == OpenVinsEvGuardMode::Healthy;
    const std::string fault_reason = openVinsEvGuardReasonName(reason);

    if (
      !force &&
      last_published_health_ok_.has_value() &&
      *last_published_health_ok_ == health_ok &&
      last_published_fault_reason_.has_value() &&
      *last_published_fault_reason_ == fault_reason)
    {
      return;
    }

    std_msgs::msg::Bool health_msg;
    health_msg.data = health_ok;
    health_ok_pub_->publish(health_msg);

    std_msgs::msg::String reason_msg;
    reason_msg.data = fault_reason;
    fault_reason_pub_->publish(reason_msg);

    last_published_health_ok_ = health_ok;
    last_published_fault_reason_ = fault_reason;

    if (
      force ||
      !last_logged_mode_.has_value() ||
      *last_logged_mode_ != mode ||
      !last_logged_reason_.has_value() ||
      *last_logged_reason_ != reason)
    {
      if (mode == OpenVinsEvGuardMode::Healthy) {
        RCLCPP_INFO(
          this->get_logger(),
          "EV guard -> %s (%s)",
          openVinsEvGuardModeName(mode),
          fault_reason.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "EV guard -> %s (%s)",
          openVinsEvGuardModeName(mode),
          fault_reason.c_str());
      }
      last_logged_mode_ = mode;
      last_logged_reason_ = reason;
    }
  }

  void handleExternalResetCounterBump(const char * reason)
  {
    ++reset_counter_;
    cached_valid_vehicle_odometry_.reset();
    guard_.reset();
    publishGuardStatus(guard_.mode(), guard_.reason(), true);
    RCLCPP_INFO(
      this->get_logger(),
      "%s, reset_counter=%u",
      reason,
      reset_counter_);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    warnUnexpectedFrames(*msg);
    if (!ensureTimestampSourceReady("EV odometry")) {
      return;
    }

    const uint64_t publish_timestamp_us = nowPx4Micros();
    if (publish_timestamp_us == 0U) {
      return;
    }
    const uint64_t receive_stamp_us = timeToMicros(this->now());
    const OpenVinsEvGuardSample guard_sample = makeGuardSample(*msg, receive_stamp_us);
    const OpenVinsEvGuardResult guard_result = guard_.observe(guard_sample);

    publishGuardStatus(guard_result.mode, guard_result.reason, false);

    std::optional<px4_msgs::msg::VehicleOdometry> fresh_output;
    if (guard_result.sample_valid) {
      if (guard_result.bump_reset_counter) {
        ++reset_counter_;
        RCLCPP_INFO(
          this->get_logger(),
          "EV guard recovery bumped reset_counter=%u",
          reset_counter_);
      }

      const uint64_t timestamp_sample = computeTimestampSample(*msg, publish_timestamp_us);
      fresh_output = buildVehicleOdometry(*msg, publish_timestamp_us, timestamp_sample, reset_counter_);
      cached_valid_vehicle_odometry_ = *fresh_output;
    }

    if (guard_result.publish_fresh && fresh_output.has_value()) {
      visual_odometry_pub_->publish(*fresh_output);
      if (log_debug_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "published fresh EV odom: pos_ned=[%.3f %.3f %.3f] vel_ned=[%.3f %.3f %.3f] reset=%u",
          fresh_output->position[0], fresh_output->position[1], fresh_output->position[2],
          fresh_output->velocity[0], fresh_output->velocity[1], fresh_output->velocity[2],
          fresh_output->reset_counter);
      }
      return;
    }

    if (guard_result.publish_hold_last && cached_valid_vehicle_odometry_.has_value()) {
      auto held_output = refreshHeldVehicleOdometry(*cached_valid_vehicle_odometry_, publish_timestamp_us);
      visual_odometry_pub_->publish(held_output);
      if (log_debug_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "published held EV odom: pos_ned=[%.3f %.3f %.3f] vel_ned=[%.3f %.3f %.3f] sample=%lu",
          held_output.position[0], held_output.position[1], held_output.position[2],
          held_output.velocity[0], held_output.velocity[1], held_output.velocity[2],
          static_cast<unsigned long>(held_output.timestamp_sample));
      }
      return;
    }

    if (guard_result.publish_hold_last && !cached_valid_vehicle_odometry_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "EV guard requested hold-last publication without a cached valid frame");
    }
  }

  std::string odometry_topic_;
  std::string visual_odometry_topic_;
  std::string expected_odom_frame_id_;
  std::string expected_child_frame_id_;
  std::string reset_counter_bump_topic_;
  std::string timesync_status_topic_;
  std::string health_ok_topic_;
  std::string fault_reason_topic_;
  float position_variance_floor_m2_{0.01f};
  float orientation_variance_floor_rad2_{0.01f};
  float velocity_variance_floor_m2ps2_{0.01f};
  int quality_{100};
  bool use_odometry_stamp_for_timestamp_sample_{true};
  bool log_debug_{false};
  double timesync_timeout_s_{0.5};
  uint8_t reset_counter_{0U};
  OpenVinsEvGuardConfig guard_config_{};
  OpenVinsEvGuard guard_;
  Px4TimestampSource px4_timestamp_source_{Px4TimestampSource::System};
  bool has_timesync_status_{false};
  int64_t estimated_px4_offset_us_{0};
  rclcpp::Time last_timesync_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Clock px4_timestamp_system_clock_{RCL_SYSTEM_TIME};
  tf2::Vector3 sensor_position_in_body_{0.0, 0.0, 0.0};
  tf2::Quaternion sensor_rotation_in_body_{0.0, 0.0, 0.0, 1.0};
  std::optional<px4_msgs::msg::VehicleOdometry> cached_valid_vehicle_odometry_;
  std::optional<bool> last_published_health_ok_;
  std::optional<std::string> last_published_fault_reason_;
  std::optional<OpenVinsEvGuardMode> last_logged_mode_;
  std::optional<OpenVinsEvGuardReason> last_logged_reason_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_counter_bump_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_ok_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_reason_pub_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::OpenVinsPx4VisionBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
