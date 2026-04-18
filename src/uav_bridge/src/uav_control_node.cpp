#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "uav_bridge/math_utils.hpp"
#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{

namespace
{

bool hasNonNegativeStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0;
}

}  // namespace

class UavControlNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

public:
  UavControlNode()
  : Node("uav_control_node")
  {
    this->declare_parameter<std::string>("position_topic", "/uav/control/position_yaw");
    this->declare_parameter<std::string>(
      "position_keep_yaw_topic", "/uav/control/position_keep_yaw");
    this->declare_parameter<std::string>(
      "velocity_body_topic", "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry_px4");
    this->declare_parameter<std::string>("execution_state_topic", "/uav/state/odometry_px4");
    this->declare_parameter<std::string>(
      "vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    this->declare_parameter<std::string>(
      "timesync_status_topic", "/fmu/out/timesync_status");
    this->declare_parameter<std::string>(
      "offboard_mode_topic", "/fmu/in/offboard_control_mode");
    this->declare_parameter<std::string>(
      "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    this->declare_parameter<std::string>("vehicle_command_topic", "/fmu/in/vehicle_command");
    this->declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status");
    this->declare_parameter<std::string>("ov_health_topic", "/uav/ov/bridge/health_ok");
    this->declare_parameter<std::string>(
      "takeoff_service", "/uav/control/command/takeoff");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>(
      "position_mode_service", "/uav/control/command/position_mode");
    this->declare_parameter<std::string>("land_service", "/uav/control/command/land");
    this->declare_parameter<std::string>("abort_service", "/uav/control/command/abort");
    this->declare_parameter<std::string>("disarm_service", "/uav/control/command/disarm");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<int>("warmup_cycles", 20);
    this->declare_parameter<int>("target_system", 1);
    this->declare_parameter<int>("target_component", 1);
    this->declare_parameter<int>("source_system", 1);
    this->declare_parameter<int>("source_component", 1);
    this->declare_parameter<double>("takeoff_height_m", 1.0);
    this->declare_parameter<double>("takeoff_height_tolerance_m", 0.10);
    this->declare_parameter<double>("takeoff_stable_s", 0.5);
    this->declare_parameter<double>("max_velocity_setpoint_mps", 1.5);
    this->declare_parameter<double>("max_acceleration_setpoint_mps2", 2.0);
    this->declare_parameter<double>("velocity_body_timeout_ms", 200.0);
    this->declare_parameter<double>("velocity_mode_max_acc_xy_mps2", 1.0);
    this->declare_parameter<double>("velocity_mode_max_acc_z_mps2", 0.6);
    this->declare_parameter<double>("velocity_mode_max_acc_yaw_radps2", 1.5);
    this->declare_parameter<double>("state_timeout_s", 0.20);
    this->declare_parameter<std::string>("base_frame_id", "uav_base_link");
    this->declare_parameter<std::string>("position_command_frame_id", "uav_odom");
    this->declare_parameter<std::string>("px4_timestamp_source", "system");
    this->declare_parameter<std::string>("gz_clock_topic", "");
    this->declare_parameter<double>("timesync_timeout_s", 0.5);

    position_topic_ = this->get_parameter("position_topic").as_string();
    position_keep_yaw_topic_ = this->get_parameter("position_keep_yaw_topic").as_string();
    velocity_body_topic_ = this->get_parameter("velocity_body_topic").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    execution_state_topic_ = this->get_parameter("execution_state_topic").as_string();
    vehicle_local_position_topic_ =
      this->get_parameter("vehicle_local_position_topic").as_string();
    timesync_status_topic_ = this->get_parameter("timesync_status_topic").as_string();
    offboard_mode_topic_ = this->get_parameter("offboard_mode_topic").as_string();
    trajectory_setpoint_topic_ = this->get_parameter("trajectory_setpoint_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("vehicle_command_topic").as_string();
    vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
    ov_health_topic_ = this->get_parameter("ov_health_topic").as_string();
    takeoff_service_ = this->get_parameter("takeoff_service").as_string();
    hold_service_ = this->get_parameter("hold_service").as_string();
    position_mode_service_ = this->get_parameter("position_mode_service").as_string();
    land_service_ = this->get_parameter("land_service").as_string();
    abort_service_ = this->get_parameter("abort_service").as_string();
    disarm_service_ = this->get_parameter("disarm_service").as_string();
    publish_rate_hz_ = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    warmup_cycles_ = this->get_parameter("warmup_cycles").as_int();
    target_system_ = static_cast<uint8_t>(this->get_parameter("target_system").as_int());
    target_component_ = static_cast<uint8_t>(this->get_parameter("target_component").as_int());
    source_system_ = static_cast<uint8_t>(this->get_parameter("source_system").as_int());
    source_component_ = static_cast<uint8_t>(this->get_parameter("source_component").as_int());
    takeoff_height_m_ = static_cast<float>(this->get_parameter("takeoff_height_m").as_double());
    takeoff_height_tolerance_m_ = static_cast<float>(
      this->get_parameter("takeoff_height_tolerance_m").as_double());
    takeoff_stable_s_ = this->get_parameter("takeoff_stable_s").as_double();
    max_velocity_setpoint_mps_ = static_cast<float>(
      this->get_parameter("max_velocity_setpoint_mps").as_double());
    max_acceleration_setpoint_mps2_ = static_cast<float>(
      this->get_parameter("max_acceleration_setpoint_mps2").as_double());
    velocity_body_timeout_ms_ = this->get_parameter("velocity_body_timeout_ms").as_double();
    velocity_mode_max_acc_xy_mps2_ = static_cast<float>(
      this->get_parameter("velocity_mode_max_acc_xy_mps2").as_double());
    velocity_mode_max_acc_z_mps2_ = static_cast<float>(
      this->get_parameter("velocity_mode_max_acc_z_mps2").as_double());
    velocity_mode_max_acc_yaw_radps2_ = static_cast<float>(
      this->get_parameter("velocity_mode_max_acc_yaw_radps2").as_double());
    state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    position_command_frame_id_ = this->get_parameter("position_command_frame_id").as_string();
    gz_clock_topic_ = this->get_parameter("gz_clock_topic").as_string();
    timesync_timeout_s_ = std::max(0.0, this->get_parameter("timesync_timeout_s").as_double());

    px4_timestamp_source_ = parsePx4TimestampSource(
      this->get_parameter("px4_timestamp_source").as_string());

    if (px4_timestamp_source_ == Px4TimestampSource::GazeboSim && gz_clock_topic_.empty()) {
      gz_clock_topic_ = "/clock";
      RCLCPP_INFO(
        this->get_logger(),
        "px4_timestamp_source=gz_sim uses ROS time; gz_clock_topic defaulted to %s",
        gz_clock_topic_.c_str());
    }

    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      offboard_mode_topic_, 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic_, 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      vehicle_command_topic_, 10);

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      position_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        handlePositionTarget(*msg);
      });

    position_keep_yaw_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      position_keep_yaw_topic_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
      {
        handlePositionKeepYawTarget(*msg);
      });

    auto velocity_qos = rclcpp::SensorDataQoS();
    velocity_qos.keep_last(1);
    velocity_body_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_body_topic_, velocity_qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
      {
        handleVelocityBodyTarget(*msg);
      });

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        handleStateOdometry(*msg);
      });

    execution_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      execution_state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        handleExecutionOdometry(*msg);
      });

    vehicle_local_position_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      vehicle_local_position_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
      {
        handleVehicleLocalPosition(*msg);
      });

    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
      {
        const bool was_armed = is_armed_;
        is_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        current_nav_state_ = msg->nav_state;
        is_offboard_mode_ =
          msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

        if (was_armed && !is_armed_) {
          enterExternalDisarmSafeState();
        }
      });

    const auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    ov_health_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      ov_health_topic_, status_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        handleOvHealth(msg->data);
      });

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

    takeoff_srv_ = this->create_service<Trigger>(
      takeoff_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleTakeoffRequest(response->message);
      });

    hold_srv_ = this->create_service<Trigger>(
      hold_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleHoldRequest(response->message);
      });

    position_mode_srv_ = this->create_service<Trigger>(
      position_mode_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handlePositionModeRequest(response->message);
      });

    land_srv_ = this->create_service<Trigger>(
      land_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleLandRequest(response->message);
      });

    abort_srv_ = this->create_service<Trigger>(
      abort_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleAbortRequest(response->message);
      });

    disarm_srv_ = this->create_service<Trigger>(
      disarm_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleDisarmRequest(response->message);
      });

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(timer_period, std::bind(&UavControlNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "uav_control_node: position=%s position_keep_yaw=%s velocity_body=%s state=%s execution=%s vlp=%s ov_health=%s",
      position_topic_.c_str(), position_keep_yaw_topic_.c_str(), velocity_body_topic_.c_str(),
      state_topic_.c_str(), execution_state_topic_.c_str(), vehicle_local_position_topic_.c_str(),
      ov_health_topic_.c_str());
  }

private:
  static const char * modeName(UavControlMode mode)
  {
    switch (mode) {
      case UavControlMode::Hold:
        return "HOLD";
      case UavControlMode::Takeoff:
        return "TAKEOFF";
      case UavControlMode::Position:
        return "POSITION";
      case UavControlMode::Px4PositionHold:
        return "PX4_POSITION_HOLD";
      case UavControlMode::Px4Stabilized:
        return "PX4_STABILIZED";
      case UavControlMode::VelocityBody:
        return "VELOCITY_BODY";
      case UavControlMode::Landing:
        return "LANDING";
    }
    return "UNKNOWN";
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

  uint64_t nowMicros()
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

  bool ensureTimestampReady(const char * context, uint64_t stamp)
  {
    if (stamp != 0U) {
      return true;
    }
    if (px4_timestamp_source_ == Px4TimestampSource::Px4Timesync) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for PX4 timesync before %s",
        context);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for valid PX4 timestamp before %s",
        context);
    }
    return false;
  }

  bool stateFresh() const
  {
    if (!has_state_ || last_state_time_.nanoseconds() == 0) {
      return false;
    }
    const double age_s = (this->now() - last_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= state_timeout_s_;
  }

  bool executionStateFresh() const
  {
    if (!has_execution_state_ || last_execution_state_time_.nanoseconds() == 0) {
      return false;
    }
    const double age_s = (this->now() - last_execution_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= state_timeout_s_;
  }

  bool currentStateAvailable(std::string & message) const
  {
    if (!has_state_) {
      message = "control state not available yet";
      return false;
    }
    if (!stateFresh()) {
      message = "control state is stale";
      return false;
    }
    if (!isFiniteVector(current_state_position_enu_) || !std::isfinite(current_state_yaw_enu_)) {
      message = "control state contains invalid pose";
      return false;
    }
    message.clear();
    return true;
  }

  bool autonomyAvailable(std::string & message) const
  {
    if (ov_fault_latched_) {
      message = "OV fault latched; autonomy disabled until restart";
      return false;
    }
    message.clear();
    return true;
  }

  bool rejectAutonomyCommand(const char * context)
  {
    if (!ov_fault_latched_) {
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "ignoring %s: OV fault latched, autonomy disabled until restart",
      context);
    return true;
  }

  void clearCommandTargets()
  {
    hold_target_valid_ = false;
    position_target_valid_ = false;
    has_velocity_body_cmd_ = false;
    last_velocity_body_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    velocity_mode_rate_limit_initialized_ = false;
    last_velocity_mode_ned_ = {0.0f, 0.0f, 0.0f};
    last_velocity_mode_yawspeed_ned_ = 0.0f;
    takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void resetWarmup()
  {
    setpoint_counter_ = 0;
  }

  void logModeChange(UavControlMode previous, const char * reason)
  {
    if (previous != mode_tracker_.mode()) {
      resetWarmup();
      if (mode_tracker_.mode() != UavControlMode::VelocityBody) {
        velocity_mode_rate_limit_initialized_ = false;
        last_velocity_mode_ned_ = {0.0f, 0.0f, 0.0f};
        last_velocity_mode_yawspeed_ned_ = 0.0f;
      }
      RCLCPP_INFO(
        this->get_logger(), "control mode -> %s (%s)",
        modeName(mode_tracker_.mode()), reason);
    }
  }

  void publishVehicleCommand(
    uint32_t command,
    float param1 = 0.0f,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f)
  {
    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("vehicle command", stamp)) {
      return;
    }

    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = stamp;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.target_system = target_system_;
    msg.target_component = target_component_;
    msg.source_system = source_system_;
    msg.source_component = source_component_;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void maybeEnableOffboardAndArm(bool allow_mode_reassert)
  {
    if (!allow_mode_reassert || setpoint_counter_ < warmup_cycles_) {
      return;
    }

    constexpr int kRetryIntervalCycles = 50;
    const int cycles_after_warmup = setpoint_counter_ - warmup_cycles_;
    if (!executionStateFresh()) {
      if (cycles_after_warmup % kRetryIntervalCycles == 0) {
        RCLCPP_INFO(
          this->get_logger(),
          "waiting for PX4 fused execution state before offboard enable");
      }
      return;
    }

    if ((!is_offboard_mode_ || !is_armed_) && (cycles_after_warmup % kRetryIntervalCycles == 0)) {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
      RCLCPP_INFO(
        this->get_logger(),
        "sent offboard+arm command (armed=%s, offboard=%s)",
        is_armed_ ? "true" : "false",
        is_offboard_mode_ ? "true" : "false");
    }
  }

  bool publishPositionSetpoint(
    const std::array<float, 3> & target_position_enu,
    float target_yaw_enu,
    bool allow_mode_reassert,
    const char * context)
  {
    if (!isFiniteVector(target_position_enu) || !std::isfinite(target_yaw_enu)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting non-finite position target for %s",
        context);
      return false;
    }

    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady(context, stamp)) {
      return false;
    }

    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = stamp;
    offboard_mode.position = true;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;
    offboard_mode.thrust_and_torque = false;
    offboard_mode.direct_actuator = false;
    offboard_mode_pub_->publish(offboard_mode);

    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    const auto target_position_ned = enuPositionToNed(
      target_position_enu[0], target_position_enu[1], target_position_enu[2]);

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = stamp;
    setpoint.position = {
      target_position_ned[0], target_position_ned[1], target_position_ned[2]};
    setpoint.velocity = {kNaN, kNaN, kNaN};
    setpoint.acceleration = {kNaN, kNaN, kNaN};
    setpoint.jerk = {kNaN, kNaN, kNaN};
    setpoint.yaw = enuYawToNed(target_yaw_enu);
    setpoint.yawspeed = 0.0f;
    trajectory_setpoint_pub_->publish(setpoint);

    maybeEnableOffboardAndArm(allow_mode_reassert);
    ++setpoint_counter_;
    return true;
  }

  void applyVelocityModeRateLimit(std::array<float, 3> & velocity_ned, double yaw_rate_enu)
  {
    const float dt =
      publish_rate_hz_ > 1.0e-6 ? static_cast<float>(1.0 / publish_rate_hz_) : 0.02f;

    if (!velocity_mode_rate_limit_initialized_) {
      last_velocity_mode_ned_ = velocity_ned;
      last_velocity_mode_yawspeed_ned_ = static_cast<float>(-yaw_rate_enu);
      velocity_mode_rate_limit_initialized_ = true;
      return;
    }

    const float max_dxy = std::max(0.0f, velocity_mode_max_acc_xy_mps2_) * dt;
    if (max_dxy > 0.0f) {
      const float dvx = velocity_ned[0] - last_velocity_mode_ned_[0];
      const float dvy = velocity_ned[1] - last_velocity_mode_ned_[1];
      const float dxy = std::sqrt((dvx * dvx) + (dvy * dvy));
      if (dxy > max_dxy && dxy > 1.0e-6f) {
        const float scale = max_dxy / dxy;
        velocity_ned[0] = last_velocity_mode_ned_[0] + (dvx * scale);
        velocity_ned[1] = last_velocity_mode_ned_[1] + (dvy * scale);
      }
    }

    const float max_dz = std::max(0.0f, velocity_mode_max_acc_z_mps2_) * dt;
    velocity_ned[2] = std::clamp(
      velocity_ned[2],
      last_velocity_mode_ned_[2] - max_dz,
      last_velocity_mode_ned_[2] + max_dz);

    const float target_yawspeed_ned = static_cast<float>(-yaw_rate_enu);
    const float max_dyaw = std::max(0.0f, velocity_mode_max_acc_yaw_radps2_) * dt;
    last_velocity_mode_yawspeed_ned_ = std::clamp(
      target_yawspeed_ned,
      last_velocity_mode_yawspeed_ned_ - max_dyaw,
      last_velocity_mode_yawspeed_ned_ + max_dyaw);

    last_velocity_mode_ned_ = velocity_ned;
  }

  bool publishVelocityBodySetpoint(
    const geometry_msgs::msg::TwistStamped & cmd,
    bool allow_mode_reassert)
  {
    if (!executionStateFresh()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "waiting for PX4 fused execution state before velocity_body setpoint");
      return false;
    }

    if (!isFiniteVelocityBodyCommand(
        static_cast<float>(cmd.twist.linear.x),
        static_cast<float>(cmd.twist.linear.y),
        static_cast<float>(cmd.twist.linear.z),
        static_cast<float>(cmd.twist.angular.z)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting non-finite velocity_body command");
      return false;
    }

    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("velocity setpoint", stamp)) {
      return false;
    }

    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = stamp;
    offboard_mode.position = false;
    offboard_mode.velocity = true;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;
    offboard_mode.thrust_and_torque = false;
    offboard_mode.direct_actuator = false;
    offboard_mode_pub_->publish(offboard_mode);

    const tf2::Matrix3x3 rotation_enu_flu(current_execution_orientation_enu_flu_);
    const tf2::Vector3 velocity_body(
      static_cast<double>(cmd.twist.linear.x),
      static_cast<double>(cmd.twist.linear.y),
      static_cast<double>(cmd.twist.linear.z));
    const tf2::Vector3 velocity_enu = rotation_enu_flu * velocity_body;

    std::array<float, 3> velocity_ned = {
      static_cast<float>(velocity_enu.y()),
      static_cast<float>(velocity_enu.x()),
      static_cast<float>(-velocity_enu.z())};
    clampVectorNorm(velocity_ned, max_velocity_setpoint_mps_);
    applyVelocityModeRateLimit(velocity_ned, cmd.twist.angular.z);

    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = stamp;
    setpoint.position = {kNaN, kNaN, kNaN};
    setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
    setpoint.acceleration = {kNaN, kNaN, kNaN};
    setpoint.jerk = {kNaN, kNaN, kNaN};
    setpoint.yaw = kNaN;
    setpoint.yawspeed = last_velocity_mode_yawspeed_ned_;
    trajectory_setpoint_pub_->publish(setpoint);

    maybeEnableOffboardAndArm(allow_mode_reassert);
    ++setpoint_counter_;
    return true;
  }

  bool captureCurrentPose(std::string & message)
  {
    if (!currentStateAvailable(message)) {
      return false;
    }

    hold_target_position_enu_ = current_state_position_enu_;
    hold_target_yaw_enu_ = current_state_yaw_enu_;
    hold_target_valid_ = true;
    return true;
  }

  bool startTakeoff(const char * reason, std::string & message)
  {
    if (!captureCurrentPose(message)) {
      return false;
    }

    hold_target_position_enu_[2] += takeoff_height_m_;
    takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestTakeoff();
    logModeChange(previous, reason);
    message = "takeoff target captured from PX4 fused state";
    return true;
  }

  void requestPx4PositionMode(const char * reason)
  {
    clearCommandTargets();
    position_mode_retry_counter_ = 0;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestPx4PositionHold();
    logModeChange(previous, reason);
  }

  void forcePx4PositionMode(const char * reason)
  {
    clearCommandTargets();
    position_mode_retry_counter_ = 0;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.forcePx4PositionHold();
    logModeChange(previous, reason);
  }

  void forcePx4StabilizedMode(const char * reason)
  {
    clearCommandTargets();
    stabilized_mode_retry_counter_ = 0;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.forcePx4Stabilized();
    logModeChange(previous, reason);
  }

  void enterLandingMode(const char * reason)
  {
    clearCommandTargets();
    land_command_sent_ = false;
    landing_retry_counter_ = 0;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestLanding();
    logModeChange(previous, reason);
  }

  bool acceptPositionCommandFrame(const std::string & frame_id) const
  {
    if (frame_id.empty()) {
      return true;
    }
    if (!position_command_frame_id_.empty() && frame_id == position_command_frame_id_) {
      return true;
    }
    return !current_state_frame_id_.empty() && frame_id == current_state_frame_id_;
  }

  void activatePositionTarget(
    const std::array<float, 3> & target_position_enu,
    float target_yaw_enu,
    const char * reason)
  {
    position_target_position_enu_ = target_position_enu;
    position_target_yaw_enu_ = target_yaw_enu;
    position_target_valid_ = true;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestPosition();
    logModeChange(previous, reason);
  }

  void handleStateOdometry(const nav_msgs::msg::Odometry & msg)
  {
    const std::array<float, 3> position_enu = {
      static_cast<float>(msg.pose.pose.position.x),
      static_cast<float>(msg.pose.pose.position.y),
      static_cast<float>(msg.pose.pose.position.z)};
    if (!isFiniteVector(position_enu)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring control state with non-finite position");
      return;
    }

    const double quaternion_norm_sq =
      (msg.pose.pose.orientation.x * msg.pose.pose.orientation.x) +
      (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y) +
      (msg.pose.pose.orientation.z * msg.pose.pose.orientation.z) +
      (msg.pose.pose.orientation.w * msg.pose.pose.orientation.w);
    if (!std::isfinite(quaternion_norm_sq) || quaternion_norm_sq <= 1.0e-12) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring control state with invalid orientation");
      return;
    }

    const bool has_stamp =
      (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
    if (has_stamp && !hasNonNegativeStamp(msg.header.stamp)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring control state with negative stamp sec=%d nanosec=%u",
        msg.header.stamp.sec,
        msg.header.stamp.nanosec);
      return;
    }
    current_state_position_enu_ = position_enu;
    current_state_orientation_enu_flu_ = tf2::Quaternion(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
    current_state_orientation_enu_flu_.normalize();
    current_state_yaw_enu_ = quaternionToYaw(msg.pose.pose.orientation);
    current_state_frame_id_ = msg.header.frame_id;
    last_state_time_ = has_stamp ?
      rclcpp::Time(msg.header.stamp, this->get_clock()->get_clock_type()) :
      this->now();
    has_state_ = std::isfinite(current_state_yaw_enu_);
  }

  void handleExecutionOdometry(const nav_msgs::msg::Odometry & msg)
  {
    const double quaternion_norm_sq =
      (msg.pose.pose.orientation.x * msg.pose.pose.orientation.x) +
      (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y) +
      (msg.pose.pose.orientation.z * msg.pose.pose.orientation.z) +
      (msg.pose.pose.orientation.w * msg.pose.pose.orientation.w);
    if (!std::isfinite(quaternion_norm_sq) || quaternion_norm_sq <= 1.0e-12) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring execution state with invalid orientation");
      return;
    }

    const bool has_stamp =
      (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
    if (has_stamp && !hasNonNegativeStamp(msg.header.stamp)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring execution state with negative stamp sec=%d nanosec=%u",
        msg.header.stamp.sec,
        msg.header.stamp.nanosec);
      return;
    }
    current_execution_orientation_enu_flu_ = tf2::Quaternion(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
    current_execution_orientation_enu_flu_.normalize();
    current_execution_yaw_enu_ = quaternionToYaw(msg.pose.pose.orientation);
    last_execution_state_time_ = has_stamp ?
      rclcpp::Time(msg.header.stamp, this->get_clock()->get_clock_type()) :
      this->now();
    has_execution_state_ = std::isfinite(current_execution_yaw_enu_);
  }

  void shiftTargetForPositionReset(std::array<float, 3> & target_position_enu, float delta_north_m, float delta_east_m)
  {
    target_position_enu[0] += delta_east_m;
    target_position_enu[1] += delta_north_m;
  }

  void handleVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    if (!has_local_position_reset_reference_) {
      last_xy_reset_counter_ = msg.xy_reset_counter;
      last_z_reset_counter_ = msg.z_reset_counter;
      last_heading_reset_counter_ = msg.heading_reset_counter;
      has_local_position_reset_reference_ = true;
      return;
    }

    const bool xy_reset = msg.xy_reset_counter != last_xy_reset_counter_;
    const bool z_reset = msg.z_reset_counter != last_z_reset_counter_;
    const bool heading_reset = msg.heading_reset_counter != last_heading_reset_counter_;

    if (xy_reset) {
      if (hold_target_valid_) {
        shiftTargetForPositionReset(
          hold_target_position_enu_, msg.delta_xy[0], msg.delta_xy[1]);
      }
      if (position_target_valid_) {
        shiftTargetForPositionReset(
          position_target_position_enu_, msg.delta_xy[0], msg.delta_xy[1]);
      }
    }

    if (z_reset) {
      if (hold_target_valid_) {
        hold_target_position_enu_[2] -= msg.delta_z;
      }
      if (position_target_valid_) {
        position_target_position_enu_[2] -= msg.delta_z;
      }
    }

    if (heading_reset) {
      if (hold_target_valid_) {
        hold_target_yaw_enu_ = normalizeAngle(hold_target_yaw_enu_ - msg.delta_heading);
      }
      if (position_target_valid_) {
        position_target_yaw_enu_ = normalizeAngle(
          position_target_yaw_enu_ - msg.delta_heading);
      }
    }

    if (xy_reset || z_reset || heading_reset) {
      RCLCPP_WARN(
        this->get_logger(),
        "applied PX4 local reset to stored targets (xy=%s z=%s heading=%s)",
        xy_reset ? "true" : "false",
        z_reset ? "true" : "false",
        heading_reset ? "true" : "false");
    }

    last_xy_reset_counter_ = msg.xy_reset_counter;
    last_z_reset_counter_ = msg.z_reset_counter;
    last_heading_reset_counter_ = msg.heading_reset_counter;
  }

  void handlePositionTarget(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (rejectAutonomyCommand("position target")) {
      return;
    }

    if (!acceptPositionCommandFrame(msg.header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring position target frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), position_command_frame_id_.c_str());
      return;
    }

    const std::array<float, 3> target_position_enu = {
      static_cast<float>(msg.pose.position.x),
      static_cast<float>(msg.pose.position.y),
      static_cast<float>(msg.pose.position.z)};
    if (!isFiniteVector(target_position_enu)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring non-finite position target");
      return;
    }

    const double quat_norm =
      (msg.pose.orientation.x * msg.pose.orientation.x) +
      (msg.pose.orientation.y * msg.pose.orientation.y) +
      (msg.pose.orientation.z * msg.pose.orientation.z) +
      (msg.pose.orientation.w * msg.pose.orientation.w);
    if (!std::isfinite(quat_norm) || quat_norm <= 1.0e-6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting position target without a valid orientation; use '%s' for position-only commands",
        position_keep_yaw_topic_.c_str());
      return;
    }

    const float target_yaw_enu = quaternionToYaw(msg.pose.orientation);
    if (!std::isfinite(target_yaw_enu)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting position target with non-finite yaw");
      return;
    }

    activatePositionTarget(target_position_enu, target_yaw_enu, "position target");
  }

  void handlePositionKeepYawTarget(const geometry_msgs::msg::PointStamped & msg)
  {
    if (rejectAutonomyCommand("position_keep_yaw target")) {
      return;
    }

    if (!acceptPositionCommandFrame(msg.header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring position_keep_yaw target frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), position_command_frame_id_.c_str());
      return;
    }

    const std::array<float, 3> target_position_enu = {
      static_cast<float>(msg.point.x),
      static_cast<float>(msg.point.y),
      static_cast<float>(msg.point.z)};
    if (!isFiniteVector(target_position_enu)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring non-finite position_keep_yaw target");
      return;
    }

    std::string state_message;
    if (!currentStateAvailable(state_message)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting position_keep_yaw target: %s",
        state_message.c_str());
      return;
    }

    activatePositionTarget(
      target_position_enu, current_state_yaw_enu_, "position_keep_yaw target");
  }

  void handleVelocityBodyTarget(const geometry_msgs::msg::TwistStamped & msg)
  {
    if (rejectAutonomyCommand("velocity_body command")) {
      return;
    }

    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring velocity_body frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }

    if (!isFiniteVelocityBodyCommand(
        static_cast<float>(msg.twist.linear.x),
        static_cast<float>(msg.twist.linear.y),
        static_cast<float>(msg.twist.linear.z),
        static_cast<float>(msg.twist.angular.z)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring non-finite velocity_body command");
      return;
    }

    last_velocity_body_cmd_ = msg;
    last_velocity_body_cmd_time_ = this->now();
    has_velocity_body_cmd_ = true;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestVelocityBody();
    logModeChange(previous, "velocity body");
  }

  bool handleTakeoffRequest(std::string & message)
  {
    if (!autonomyAvailable(message)) {
      return false;
    }
    return startTakeoff("takeoff", message);
  }

  bool handleHoldRequest(std::string & message)
  {
    if (!autonomyAvailable(message)) {
      return false;
    }
    if (!captureCurrentPose(message)) {
      return false;
    }

    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestHold();
    logModeChange(previous, "hold");
    message = "hold target captured from PX4 fused state";
    return true;
  }

  bool handleLandRequest(std::string & message)
  {
    if (!autonomyAvailable(message)) {
      return false;
    }
    if (mode_tracker_.mode() == UavControlMode::Landing) {
      message = "landing already in progress";
      return true;
    }

    enterLandingMode("land");
    message = "PX4 landing requested";
    return true;
  }

  bool handlePositionModeRequest(std::string & message)
  {
    if (ov_fault_latched_) {
      forcePx4PositionMode("position_mode_ov_fault");
      message = "OV fault latched; keeping PX4 Position mode";
    } else {
      requestPx4PositionMode("position_mode");
      message = "PX4 Position mode requested";
    }
    return true;
  }

  bool handleAbortRequest(std::string & message)
  {
    forcePx4StabilizedMode("abort");
    message = "abort requested, switching to PX4 Stabilized mode";
    return true;
  }

  bool handleDisarmRequest(std::string & message)
  {
    clearCommandTargets();
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
    forcePx4PositionMode("disarm");
    message = "disarm requested";
    return true;
  }

  void publishHoldTarget()
  {
    if (!hold_target_valid_) {
      return;
    }
    publishPositionSetpoint(hold_target_position_enu_, hold_target_yaw_enu_, true, "hold");
  }

  void publishPositionTarget()
  {
    if (!position_target_valid_) {
      return;
    }
    publishPositionSetpoint(
      position_target_position_enu_, position_target_yaw_enu_, true, "position");
  }

  void handleTakeoffMode()
  {
    if (!hold_target_valid_) {
      return;
    }

    if (!publishPositionSetpoint(
        hold_target_position_enu_, hold_target_yaw_enu_, true, "takeoff"))
    {
      return;
    }

    if (!stateFresh()) {
      return;
    }

    const float height_error_m = std::abs(hold_target_position_enu_[2] - current_state_position_enu_[2]);
    if (height_error_m > takeoff_height_tolerance_m_) {
      takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }

    if (takeoff_reached_since_.nanoseconds() == 0) {
      takeoff_reached_since_ = this->now();
      return;
    }

    if ((this->now() - takeoff_reached_since_).seconds() >= takeoff_stable_s_) {
      const auto previous = mode_tracker_.mode();
      mode_tracker_.requestHold();
      logModeChange(previous, "takeoff reached");
    }
  }

  void handleVelocityBodyMode()
  {
    if (!has_velocity_body_cmd_) {
      return;
    }

    if (last_velocity_body_cmd_time_.nanoseconds() != 0) {
      const double age_ms =
        (this->now() - last_velocity_body_cmd_time_).seconds() * 1000.0;
      if (std::isfinite(age_ms) && age_ms > velocity_body_timeout_ms_) {
        last_velocity_body_cmd_.header.stamp = this->now();
        if (last_velocity_body_cmd_.header.frame_id.empty()) {
          last_velocity_body_cmd_.header.frame_id = base_frame_id_;
        }
        last_velocity_body_cmd_.twist.linear.x = 0.0;
        last_velocity_body_cmd_.twist.linear.y = 0.0;
        last_velocity_body_cmd_.twist.linear.z = 0.0;
        last_velocity_body_cmd_.twist.angular.x = 0.0;
        last_velocity_body_cmd_.twist.angular.y = 0.0;
        last_velocity_body_cmd_.twist.angular.z = 0.0;
        last_velocity_body_cmd_time_ = this->now();
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "velocity_body command timed out, holding zero velocity in offboard");
      }
    }

    publishVelocityBodySetpoint(last_velocity_body_cmd_, true);
  }

  void handleLandingMode()
  {
    constexpr int kRetryIntervalCycles = 50;
    const bool is_auto_land_mode =
      current_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;

    if (!land_command_sent_ ||
      (!is_auto_land_mode && (landing_retry_counter_ % kRetryIntervalCycles == 0)))
    {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
      land_command_sent_ = true;
    }

    ++landing_retry_counter_;
  }

  void handlePx4PositionMode()
  {
    if (current_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL) {
      return;
    }

    constexpr int kRetryIntervalCycles = 50;
    constexpr float kMavStandardModePositionHold = 1.0f;
    constexpr float kMavModeFlagCustomModeEnabled = 1.0f;
    constexpr float kPx4CustomMainModePosctl = 3.0f;
    constexpr float kPx4CustomSubModePosctl = 0.0f;

    if (position_mode_retry_counter_ % kRetryIntervalCycles == 0) {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_STANDARD_MODE,
        kMavStandardModePositionHold,
        0.0f);
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        kMavModeFlagCustomModeEnabled,
        kPx4CustomMainModePosctl,
        kPx4CustomSubModePosctl);
    }

    ++position_mode_retry_counter_;
  }

  void handlePx4StabilizedMode()
  {
    if (current_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB) {
      return;
    }

    constexpr int kRetryIntervalCycles = 50;
    constexpr float kMavModeFlagCustomModeEnabled = 1.0f;
    constexpr float kPx4CustomMainModeStabilized = 7.0f;

    if (stabilized_mode_retry_counter_ % kRetryIntervalCycles == 0) {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        kMavModeFlagCustomModeEnabled,
        kPx4CustomMainModeStabilized,
        0.0f);
    }

    ++stabilized_mode_retry_counter_;
  }

  void enterExternalDisarmSafeState()
  {
    clearCommandTargets();
    land_command_sent_ = false;
    landing_retry_counter_ = 0;
    position_mode_retry_counter_ = 0;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.forcePx4PositionHold();
    logModeChange(previous, "external disarm");
  }

  void handleOvHealth(bool health_ok)
  {
    has_ov_health_status_ = true;
    ov_health_ok_ = health_ok;
    if (health_ok || ov_fault_latched_) {
      return;
    }

    ov_fault_latched_ = true;
    clearCommandTargets();
    land_command_sent_ = false;
    landing_retry_counter_ = 0;
    position_mode_retry_counter_ = 0;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.forcePx4PositionHold();
    logModeChange(previous, "ov_guard_fault");
    RCLCPP_ERROR(
      this->get_logger(),
      "OV guard unhealthy; stopping autonomy and forcing PX4 Position mode");
  }

  void timerCallback()
  {
    switch (mode_tracker_.mode()) {
      case UavControlMode::Hold:
        publishHoldTarget();
        break;

      case UavControlMode::Takeoff:
        handleTakeoffMode();
        break;

      case UavControlMode::Position:
        publishPositionTarget();
        break;

      case UavControlMode::Px4PositionHold:
        handlePx4PositionMode();
        break;

      case UavControlMode::Px4Stabilized:
        handlePx4StabilizedMode();
        break;

      case UavControlMode::VelocityBody:
        handleVelocityBodyMode();
        break;

      case UavControlMode::Landing:
        handleLandingMode();
        break;
    }
  }

  std::string position_topic_;
  std::string position_keep_yaw_topic_;
  std::string velocity_body_topic_;
  std::string state_topic_;
  std::string execution_state_topic_;
  std::string vehicle_local_position_topic_;
  std::string timesync_status_topic_;
  std::string offboard_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_command_topic_;
  std::string vehicle_status_topic_;
  std::string ov_health_topic_;
  std::string takeoff_service_;
  std::string hold_service_;
  std::string position_mode_service_;
  std::string land_service_;
  std::string abort_service_;
  std::string disarm_service_;
  std::string base_frame_id_;
  std::string position_command_frame_id_;
  std::string current_state_frame_id_;
  std::string gz_clock_topic_;
  double publish_rate_hz_{50.0};
  int warmup_cycles_{20};
  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};
  float takeoff_height_m_{1.0f};
  float takeoff_height_tolerance_m_{0.10f};
  double takeoff_stable_s_{0.5};
  float max_velocity_setpoint_mps_{1.5f};
  float max_acceleration_setpoint_mps2_{2.0f};
  double velocity_body_timeout_ms_{200.0};
  float velocity_mode_max_acc_xy_mps2_{1.0f};
  float velocity_mode_max_acc_z_mps2_{0.6f};
  float velocity_mode_max_acc_yaw_radps2_{1.5f};
  double state_timeout_s_{0.20};
  double timesync_timeout_s_{0.5};
  Px4TimestampSource px4_timestamp_source_{Px4TimestampSource::System};
  UavControlModeTracker mode_tracker_{};
  int setpoint_counter_{0};
  int landing_retry_counter_{0};
  int position_mode_retry_counter_{0};
  int stabilized_mode_retry_counter_{0};
  bool land_command_sent_{false};
  bool has_state_{false};
  bool has_execution_state_{false};
  bool has_ov_health_status_{false};
  bool is_armed_{false};
  bool is_offboard_mode_{false};
  bool has_velocity_body_cmd_{false};
  bool velocity_mode_rate_limit_initialized_{false};
  bool has_timesync_status_{false};
  bool has_local_position_reset_reference_{false};
  bool ov_health_ok_{true};
  bool ov_fault_latched_{false};
  uint8_t current_nav_state_{px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL};
  uint8_t last_xy_reset_counter_{0};
  uint8_t last_z_reset_counter_{0};
  uint8_t last_heading_reset_counter_{0};
  int64_t estimated_px4_offset_us_{0};
  float current_state_yaw_enu_{0.0f};
  float current_execution_yaw_enu_{0.0f};
  float hold_target_yaw_enu_{0.0f};
  float position_target_yaw_enu_{0.0f};
  bool hold_target_valid_{false};
  bool position_target_valid_{false};
  std::array<float, 3> current_state_position_enu_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> hold_target_position_enu_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> position_target_position_enu_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> last_velocity_mode_ned_{0.0f, 0.0f, 0.0f};
  float last_velocity_mode_yawspeed_ned_{0.0f};
  geometry_msgs::msg::TwistStamped last_velocity_body_cmd_{};
  tf2::Quaternion current_state_orientation_enu_flu_{0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion current_execution_orientation_enu_flu_{0.0, 0.0, 0.0, 1.0};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_execution_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_body_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_reached_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_timesync_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Clock px4_timestamp_system_clock_{RCL_SYSTEM_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_keep_yaw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr execution_state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ov_health_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
  rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<Trigger>::SharedPtr position_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr land_srv_;
  rclcpp::Service<Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<Trigger>::SharedPtr disarm_srv_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::UavControlNode>());
  rclcpp::shutdown();
  return 0;
}
