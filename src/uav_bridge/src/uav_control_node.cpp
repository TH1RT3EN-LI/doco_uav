#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "uav_bridge/math_utils.hpp"
#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{

class UavControlNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

  enum class MotionGuardViolation : uint8_t
  {
    None = 0,
    Soft = 1,
    Hard = 2,
  };

  enum class MotionGuardAction : uint8_t
  {
    Accept = 0,
    Reject = 1,
    Trip = 2,
  };

public:
  UavControlNode()
  : Node("uav_control_node")
  {
    this->declare_parameter<std::string>("position_topic", "/uav/control/pose");
    this->declare_parameter<std::string>("velocity_body_topic", "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("execution_state_topic", "/uav/state/odometry_px4");
    this->declare_parameter<std::string>("offboard_mode_topic", "/fmu/in/offboard_control_mode");
    this->declare_parameter<std::string>("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    this->declare_parameter<std::string>("vehicle_command_topic", "/fmu/in/vehicle_command");
    this->declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status");
    this->declare_parameter<std::string>("takeoff_service", "/uav/control/command/takeoff");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>("position_mode_service", "/uav/control/command/position_mode");
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
    this->declare_parameter<double>("ov_hold_kp_xy", 0.8);
    this->declare_parameter<double>("ov_hold_kp_z", 0.8);
    this->declare_parameter<double>("ov_hold_kp_yaw", 1.0);
    this->declare_parameter<double>("ov_hold_max_vxy", 0.4);
    this->declare_parameter<double>("ov_hold_max_vz", 0.2);
    this->declare_parameter<double>("ov_hold_max_yaw_rate", 0.4);
    this->declare_parameter<double>("ov_target_xy_tolerance_m", 0.05);
    this->declare_parameter<double>("ov_target_z_tolerance_m", 0.05);
    this->declare_parameter<double>("ov_target_yaw_tolerance_rad", 0.08);
    this->declare_parameter<double>("ov_fault_pose_timeout_s", 0.20);
    this->declare_parameter<double>("ov_fault_max_xy_step_m", 0.30);
    this->declare_parameter<double>("ov_fault_max_z_step_m", 0.20);
    this->declare_parameter<double>("ov_fault_max_yaw_step_rad", 0.35);
    this->declare_parameter<bool>("motion_guard_enabled", true);
    this->declare_parameter<double>("motion_guard_soft_dwell_s", 2.0);
    this->declare_parameter<double>("motion_guard_pose_gap_reset_s", 0.40);
    this->declare_parameter<double>("motion_guard_soft_xy_mps", 0.40);
    this->declare_parameter<double>("motion_guard_soft_z_mps", 0.25);
    this->declare_parameter<double>("motion_guard_soft_yaw_radps", 0.60);
    this->declare_parameter<double>("motion_guard_hard_xy_mps", 0.55);
    this->declare_parameter<double>("motion_guard_hard_z_mps", 0.35);
    this->declare_parameter<double>("motion_guard_hard_yaw_radps", 0.90);
    this->declare_parameter<double>("motion_guard_feedback_hard_xy_mps", 0.65);
    this->declare_parameter<double>("motion_guard_feedback_hard_z_mps", 0.45);
    this->declare_parameter<double>("motion_guard_pose_soft_xy_step_m", 0.25);
    this->declare_parameter<double>("motion_guard_pose_soft_z_step_m", 0.12);
    this->declare_parameter<double>("motion_guard_pose_soft_yaw_step_rad", 0.35);
    this->declare_parameter<double>("motion_guard_pose_hard_xy_step_m", 0.50);
    this->declare_parameter<double>("motion_guard_pose_hard_z_step_m", 0.25);
    this->declare_parameter<double>("motion_guard_pose_hard_yaw_step_rad", 0.70);
    this->declare_parameter<std::string>("base_frame_id", "uav_base_link");
    this->declare_parameter<std::string>("position_command_frame_id", "global");
    this->declare_parameter<std::string>("px4_timestamp_source", "system");
    this->declare_parameter<std::string>("gz_world_name", "test");
    this->declare_parameter<std::string>("gz_clock_topic", "");

    position_topic_ = this->get_parameter("position_topic").as_string();
    velocity_body_topic_ = this->get_parameter("velocity_body_topic").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    execution_state_topic_ = this->get_parameter("execution_state_topic").as_string();
    offboard_mode_topic_ = this->get_parameter("offboard_mode_topic").as_string();
    trajectory_setpoint_topic_ = this->get_parameter("trajectory_setpoint_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("vehicle_command_topic").as_string();
    vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
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
    takeoff_height_tolerance_m_ = static_cast<float>(this->get_parameter("takeoff_height_tolerance_m").as_double());
    takeoff_stable_s_ = this->get_parameter("takeoff_stable_s").as_double();
    max_velocity_setpoint_mps_ = static_cast<float>(this->get_parameter("max_velocity_setpoint_mps").as_double());
    max_acceleration_setpoint_mps2_ = static_cast<float>(this->get_parameter("max_acceleration_setpoint_mps2").as_double());
    velocity_body_timeout_ms_ = this->get_parameter("velocity_body_timeout_ms").as_double();
    velocity_mode_max_acc_xy_mps2_ = static_cast<float>(this->get_parameter("velocity_mode_max_acc_xy_mps2").as_double());
    velocity_mode_max_acc_z_mps2_ = static_cast<float>(this->get_parameter("velocity_mode_max_acc_z_mps2").as_double());
    velocity_mode_max_acc_yaw_radps2_ = static_cast<float>(this->get_parameter("velocity_mode_max_acc_yaw_radps2").as_double());
    execution_state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    ov_hold_kp_xy_ = static_cast<float>(this->get_parameter("ov_hold_kp_xy").as_double());
    ov_hold_kp_z_ = static_cast<float>(this->get_parameter("ov_hold_kp_z").as_double());
    ov_hold_kp_yaw_ = static_cast<float>(this->get_parameter("ov_hold_kp_yaw").as_double());
    ov_hold_max_vxy_ = static_cast<float>(this->get_parameter("ov_hold_max_vxy").as_double());
    ov_hold_max_vz_ = static_cast<float>(this->get_parameter("ov_hold_max_vz").as_double());
    ov_hold_max_yaw_rate_ = static_cast<float>(this->get_parameter("ov_hold_max_yaw_rate").as_double());
    ov_target_xy_tolerance_m_ = static_cast<float>(this->get_parameter("ov_target_xy_tolerance_m").as_double());
    ov_target_z_tolerance_m_ = static_cast<float>(this->get_parameter("ov_target_z_tolerance_m").as_double());
    ov_target_yaw_tolerance_rad_ = static_cast<float>(this->get_parameter("ov_target_yaw_tolerance_rad").as_double());
    ov_fault_pose_timeout_s_ = this->get_parameter("ov_fault_pose_timeout_s").as_double();
    ov_fault_max_xy_step_m_ = static_cast<float>(this->get_parameter("ov_fault_max_xy_step_m").as_double());
    ov_fault_max_z_step_m_ = static_cast<float>(this->get_parameter("ov_fault_max_z_step_m").as_double());
    ov_fault_max_yaw_step_rad_ = static_cast<float>(this->get_parameter("ov_fault_max_yaw_step_rad").as_double());
    motion_guard_enabled_ = this->get_parameter("motion_guard_enabled").as_bool();
    motion_guard_soft_dwell_s_ = this->get_parameter("motion_guard_soft_dwell_s").as_double();
    motion_guard_pose_gap_reset_s_ = this->get_parameter("motion_guard_pose_gap_reset_s").as_double();
    motion_guard_soft_xy_mps_ = static_cast<float>(this->get_parameter("motion_guard_soft_xy_mps").as_double());
    motion_guard_soft_z_mps_ = static_cast<float>(this->get_parameter("motion_guard_soft_z_mps").as_double());
    motion_guard_soft_yaw_radps_ = static_cast<float>(this->get_parameter("motion_guard_soft_yaw_radps").as_double());
    motion_guard_hard_xy_mps_ = static_cast<float>(this->get_parameter("motion_guard_hard_xy_mps").as_double());
    motion_guard_hard_z_mps_ = static_cast<float>(this->get_parameter("motion_guard_hard_z_mps").as_double());
    motion_guard_hard_yaw_radps_ = static_cast<float>(this->get_parameter("motion_guard_hard_yaw_radps").as_double());
    motion_guard_feedback_hard_xy_mps_ = static_cast<float>(this->get_parameter("motion_guard_feedback_hard_xy_mps").as_double());
    motion_guard_feedback_hard_z_mps_ = static_cast<float>(this->get_parameter("motion_guard_feedback_hard_z_mps").as_double());
    motion_guard_pose_soft_xy_step_m_ = static_cast<float>(this->get_parameter("motion_guard_pose_soft_xy_step_m").as_double());
    motion_guard_pose_soft_z_step_m_ = static_cast<float>(this->get_parameter("motion_guard_pose_soft_z_step_m").as_double());
    motion_guard_pose_soft_yaw_step_rad_ = static_cast<float>(this->get_parameter("motion_guard_pose_soft_yaw_step_rad").as_double());
    motion_guard_pose_hard_xy_step_m_ = static_cast<float>(this->get_parameter("motion_guard_pose_hard_xy_step_m").as_double());
    motion_guard_pose_hard_z_step_m_ = static_cast<float>(this->get_parameter("motion_guard_pose_hard_z_step_m").as_double());
    motion_guard_pose_hard_yaw_step_rad_ = static_cast<float>(this->get_parameter("motion_guard_pose_hard_yaw_step_rad").as_double());
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    position_command_frame_id_ = this->get_parameter("position_command_frame_id").as_string();
    gz_clock_topic_ = this->get_parameter("gz_clock_topic").as_string();

    const std::string timestamp_source = this->get_parameter("px4_timestamp_source").as_string();
    px4_timestamp_source_ = parsePx4TimestampSource(timestamp_source);

    if (px4_timestamp_source_ == Px4TimestampSource::GazeboSim) {
      if (gz_clock_topic_.empty()) {
        gz_clock_topic_ = "/clock";
      }
      RCLCPP_INFO(
        this->get_logger(),
        "px4_timestamp_source=gz_sim now uses the ROS clock exposed on %s; gz_world_name is kept for compatibility",
        gz_clock_topic_.c_str());
    }

    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic_, 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic_, 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, 10);

    auto velocity_body_qos = rclcpp::SensorDataQoS();
    velocity_body_qos.keep_last(1);

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      position_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        handlePositionTarget(*msg);
      });
    velocity_body_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_body_topic_, velocity_body_qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
      {
        handleVelocityBodyTarget(*msg);
      });

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        const bool has_stamp =
          (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
        const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
        const std::array<float, 3> position_enu = {
          static_cast<float>(msg->pose.pose.position.x),
          static_cast<float>(msg->pose.pose.position.y),
          static_cast<float>(msg->pose.pose.position.z)};
        if (!isFiniteVector(position_enu)) {
          setOvFault("OV control state contains non-finite position");
          return;
        }

        const double quaternion_norm_sq =
          (msg->pose.pose.orientation.x * msg->pose.pose.orientation.x) +
          (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y) +
          (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z) +
          (msg->pose.pose.orientation.w * msg->pose.pose.orientation.w);
        if (!std::isfinite(quaternion_norm_sq) || quaternion_norm_sq <= 1.0e-12) {
          setOvFault("OV control state contains invalid orientation");
          return;
        }

        const float yaw_enu = quaternionToYaw(msg->pose.pose.orientation);
        if (!std::isfinite(yaw_enu)) {
          setOvFault("OV control state contains invalid yaw");
          return;
        }

        if (ov_fault_active_) {
          has_last_ov_sample_ = false;
        }

        if (has_last_ov_sample_) {
          const float dx = position_enu[0] - last_ov_sample_position_enu_[0];
          const float dy = position_enu[1] - last_ov_sample_position_enu_[1];
          const float dz = position_enu[2] - last_ov_sample_position_enu_[2];
          const float xy_step_m = std::hypot(dx, dy);
          const float abs_z_step_m = std::abs(dz);
          const float abs_yaw_step_rad = std::abs(normalizeAngle(yaw_enu - last_ov_sample_yaw_enu_));
          if (exceedsPositiveLimit(xy_step_m, ov_fault_max_xy_step_m_)) {
            setOvFault("OV control state xy jump exceeded threshold");
            return;
          }
          if (exceedsPositiveLimit(abs_z_step_m, ov_fault_max_z_step_m_)) {
            setOvFault("OV control state z jump exceeded threshold");
            return;
          }
          if (exceedsPositiveLimit(abs_yaw_step_rad, ov_fault_max_yaw_step_rad_)) {
            setOvFault("OV control state yaw jump exceeded threshold");
            return;
          }
        }

        current_position_enu_ = position_enu;
        current_yaw_enu_ = yaw_enu;
        current_orientation_enu_flu_ = tf2::Quaternion(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);
        current_orientation_enu_flu_.normalize();
        last_state_time_ = stamp;
        has_state_ = true;
        last_ov_sample_position_enu_ = position_enu;
        last_ov_sample_yaw_enu_ = yaw_enu;
        has_last_ov_sample_ = true;
        clearOvFault("OV control state recovered");
      });

    execution_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      execution_state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        const bool has_stamp =
          (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
        const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
        const std::array<float, 3> execution_body_velocity = {
          static_cast<float>(msg->twist.twist.linear.x),
          static_cast<float>(msg->twist.twist.linear.y),
          static_cast<float>(msg->twist.twist.linear.z)};
        const double quaternion_norm_sq =
          (msg->pose.pose.orientation.x * msg->pose.pose.orientation.x) +
          (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y) +
          (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z) +
          (msg->pose.pose.orientation.w * msg->pose.pose.orientation.w);
        if (!std::isfinite(quaternion_norm_sq) || quaternion_norm_sq <= 1.0e-12) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "ignoring execution state with invalid orientation");
          return;
        }
        if (!isFiniteVector(execution_body_velocity)) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "ignoring execution state with non-finite body velocity");
          return;
        }

        current_execution_yaw_enu_ = quaternionToYaw(msg->pose.pose.orientation);
        current_execution_orientation_enu_flu_ = tf2::Quaternion(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);
        current_execution_orientation_enu_flu_.normalize();
        current_execution_body_velocity_flu_ = execution_body_velocity;
        last_execution_state_time_ = stamp;
        has_execution_state_ = true;
      });

    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
      {
        const bool was_armed = is_armed_;
        is_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        current_nav_state_ = msg->nav_state;
        is_offboard_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        if (was_armed && !is_armed_) {
          enterExternalDisarmSafeState();
        }
      });

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
      "uav_control_node: position=%s velocity_body=%s control_state=%s execution_state=%s takeoff=%s hold=%s position_mode=%s land=%s abort=%s disarm=%s",
      position_topic_.c_str(), velocity_body_topic_.c_str(), state_topic_.c_str(),
      execution_state_topic_.c_str(),
      takeoff_service_.c_str(), hold_service_.c_str(), position_mode_service_.c_str(),
      land_service_.c_str(), abort_service_.c_str(), disarm_service_.c_str());
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
      case UavControlMode::VelocityBody:
        return "VELOCITY_BODY";
      case UavControlMode::Landing:
        return "LANDING";
    }
    return "UNKNOWN";
  }

  static bool isOvDrivenMode(UavControlMode mode)
  {
    switch (mode) {
      case UavControlMode::Hold:
      case UavControlMode::Takeoff:
      case UavControlMode::Position:
        return true;
      case UavControlMode::Px4PositionHold:
      case UavControlMode::VelocityBody:
      case UavControlMode::Landing:
        return false;
    }
    return false;
  }

  bool ovAutomaticControlActive() const
  {
    switch (mode_tracker_.mode()) {
      case UavControlMode::Hold:
      case UavControlMode::Takeoff:
        return hold_target_valid_;
      case UavControlMode::Position:
        return position_target_valid_;
      case UavControlMode::Px4PositionHold:
      case UavControlMode::VelocityBody:
      case UavControlMode::Landing:
        return false;
    }
    return false;
  }

  static float clampSigned(float value, float max_abs)
  {
    if (!std::isfinite(value) || max_abs <= 0.0f) {
      return 0.0f;
    }
    return std::clamp(value, -max_abs, max_abs);
  }

  void resetWarmup()
  {
    setpoint_counter_ = 0;
  }

  void clearManualCommands()
  {
    has_velocity_body_cmd_ = false;
    last_velocity_body_time_us_ = 0;
    velocity_mode_rate_limit_initialized_ = false;
    last_velocity_mode_ned_ = {0.0f, 0.0f, 0.0f};
    last_velocity_mode_yawspeed_ned_ = 0.0f;
    hold_target_valid_ = false;
    position_target_valid_ = false;
    clearMotionGuardSoftViolation();
  }

  void logModeChange(UavControlMode previous, const char * reason)
  {
    if (previous != mode_tracker_.mode()) {
      velocity_execution_ready_once_ = false;
      if (mode_tracker_.mode() != UavControlMode::VelocityBody) {
        velocity_mode_rate_limit_initialized_ = false;
        last_velocity_mode_ned_ = {0.0f, 0.0f, 0.0f};
        last_velocity_mode_yawspeed_ned_ = 0.0f;
      }
      RCLCPP_INFO(this->get_logger(), "control mode -> %s (%s)", modeName(mode_tracker_.mode()), reason);
      resetWarmup();
    }
  }

  static uint64_t timeToMicros(const rclcpp::Time & time)
  {
    const int64_t nanoseconds = time.nanoseconds();
    if (nanoseconds <= 0) {
      return 0;
    }
    return static_cast<uint64_t>(nanoseconds / 1000);
  }

  uint64_t nowMicros()
  {
    const uint64_t ros_time_us = timeToMicros(this->get_clock()->now());
    const uint64_t system_time_us = timeToMicros(px4_timestamp_system_clock_.now());
    return selectPx4TimestampMicros(px4_timestamp_source_, ros_time_us, system_time_us);
  }

  bool ensureTimestampReady(const char * context, uint64_t stamp)
  {
    if (stamp != 0) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "waiting for valid timestamp before %s", context);
    return false;
  }

  bool isPx4ExecutionReady() const
  {
    return has_execution_state_ && executionStateFresh();
  }

  bool isPx4VelocityExecutionReady() const
  {
    return isPx4ExecutionReady();
  }

  bool controlStateFresh() const
  {
    if (!has_state_ || last_state_time_.nanoseconds() == 0) {
      return false;
    }

    const double age_s = (this->now() - last_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= ov_fault_pose_timeout_s_;
  }

  bool executionStateFresh() const
  {
    if (last_execution_state_time_.nanoseconds() == 0) {
      return false;
    }

    const double age_s = (this->now() - last_execution_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= execution_state_timeout_s_;
  }

  static bool exceedsPositiveLimit(float value, float limit)
  {
    return std::isfinite(value) && limit > 0.0f && value > limit;
  }

  static void updateMotionGuardViolation(
    MotionGuardViolation candidate,
    const char * candidate_reason,
    MotionGuardViolation & current,
    const char * & current_reason)
  {
    if (static_cast<uint8_t>(candidate) > static_cast<uint8_t>(current)) {
      current = candidate;
      current_reason = candidate_reason;
    }
  }

  void clearMotionGuardSoftViolation()
  {
    soft_violation_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  bool motionGuardActive() const
  {
    return motion_guard_enabled_ && is_armed_;
  }

  void resetMotionGuardState()
  {
    motion_guard_tripped_ = false;
    motion_guard_trip_reason_.clear();
    has_last_accepted_pose_target_ = false;
    last_accepted_pose_target_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    clearMotionGuardSoftViolation();
  }

  bool rejectIfMotionGuardTripped(const char * action, std::string & message) const
  {
    if (!motion_guard_tripped_) {
      return false;
    }

    message = "motion guard latched";
    if (!motion_guard_trip_reason_.empty()) {
      message += " (";
      message += motion_guard_trip_reason_;
      message += ")";
    }
    message += "; disarm before ";
    message += action;
    return true;
  }

  bool rejectIfLandingLocked(const char * action, std::string & message) const
  {
    if (mode_tracker_.mode() != UavControlMode::Landing) {
      return false;
    }

    message = "landing in progress; cannot ";
    message += action;
    message += " until abort, position_mode, disarm, or landing completes";
    return true;
  }

  void triggerMotionGuardLanding(const char * reason)
  {
    if (!motionGuardActive() || motion_guard_tripped_) {
      return;
    }

    motion_guard_tripped_ = true;
    motion_guard_trip_reason_ = reason != nullptr ? reason : "motion guard violated";
    clearManualCommands();
    enterLandingMode("land");
    RCLCPP_ERROR(
      this->get_logger(),
      "motion guard tripped: %s",
      motion_guard_trip_reason_.c_str());
  }

  MotionGuardAction motionGuardActionForViolation(
    MotionGuardViolation violation,
    const char * reason,
    const rclcpp::Time & now)
  {
    if (!motion_guard_enabled_) {
      return MotionGuardAction::Accept;
    }
    if (motion_guard_tripped_) {
      return MotionGuardAction::Reject;
    }

    if (violation == MotionGuardViolation::None) {
      clearMotionGuardSoftViolation();
      return MotionGuardAction::Accept;
    }

    if (!motionGuardActive()) {
      clearMotionGuardSoftViolation();
      return MotionGuardAction::Reject;
    }

    if (violation == MotionGuardViolation::Hard || motion_guard_soft_dwell_s_ <= 0.0) {
      triggerMotionGuardLanding(reason);
      return MotionGuardAction::Trip;
    }

    if (soft_violation_since_.nanoseconds() == 0) {
      soft_violation_since_ = now;
      return MotionGuardAction::Accept;
    }

    if ((now - soft_violation_since_).seconds() >= motion_guard_soft_dwell_s_) {
      triggerMotionGuardLanding(reason);
      return MotionGuardAction::Trip;
    }
    return MotionGuardAction::Accept;
  }

  MotionGuardViolation classifyVelocityEnvelope(
    float xy_mps,
    float abs_z_mps,
    float abs_yaw_radps,
    const char * & reason) const
  {
    if (exceedsPositiveLimit(xy_mps, motion_guard_hard_xy_mps_)) {
      reason = "velocity_body xy exceeded hard limit";
      return MotionGuardViolation::Hard;
    }
    if (exceedsPositiveLimit(abs_z_mps, motion_guard_hard_z_mps_)) {
      reason = "velocity_body z exceeded hard limit";
      return MotionGuardViolation::Hard;
    }
    if (exceedsPositiveLimit(abs_yaw_radps, motion_guard_hard_yaw_radps_)) {
      reason = "velocity_body yaw rate exceeded hard limit";
      return MotionGuardViolation::Hard;
    }
    if (exceedsPositiveLimit(xy_mps, motion_guard_soft_xy_mps_)) {
      reason = "velocity_body xy exceeded soft limit";
      return MotionGuardViolation::Soft;
    }
    if (exceedsPositiveLimit(abs_z_mps, motion_guard_soft_z_mps_)) {
      reason = "velocity_body z exceeded soft limit";
      return MotionGuardViolation::Soft;
    }
    if (exceedsPositiveLimit(abs_yaw_radps, motion_guard_soft_yaw_radps_)) {
      reason = "velocity_body yaw rate exceeded soft limit";
      return MotionGuardViolation::Soft;
    }

    reason = nullptr;
    return MotionGuardViolation::None;
  }

  MotionGuardViolation classifyPoseStepEnvelope(
    float xy_step_m,
    float abs_z_step_m,
    float abs_yaw_step_rad,
    const char * & reason) const
  {
    if (exceedsPositiveLimit(xy_step_m, motion_guard_pose_hard_xy_step_m_)) {
      reason = "position target xy step exceeded hard limit";
      return MotionGuardViolation::Hard;
    }
    if (exceedsPositiveLimit(abs_z_step_m, motion_guard_pose_hard_z_step_m_)) {
      reason = "position target z step exceeded hard limit";
      return MotionGuardViolation::Hard;
    }
    if (exceedsPositiveLimit(abs_yaw_step_rad, motion_guard_pose_hard_yaw_step_rad_)) {
      reason = "position target yaw step exceeded hard limit";
      return MotionGuardViolation::Hard;
    }
    if (exceedsPositiveLimit(xy_step_m, motion_guard_pose_soft_xy_step_m_)) {
      reason = "position target xy step exceeded soft limit";
      return MotionGuardViolation::Soft;
    }
    if (exceedsPositiveLimit(abs_z_step_m, motion_guard_pose_soft_z_step_m_)) {
      reason = "position target z step exceeded soft limit";
      return MotionGuardViolation::Soft;
    }
    if (exceedsPositiveLimit(abs_yaw_step_rad, motion_guard_pose_soft_yaw_step_rad_)) {
      reason = "position target yaw step exceeded soft limit";
      return MotionGuardViolation::Soft;
    }

    reason = nullptr;
    return MotionGuardViolation::None;
  }

  bool enforceFeedbackMotionGuard()
  {
    if (!motionGuardActive() || motion_guard_tripped_ || !executionStateFresh() || !has_execution_state_) {
      return false;
    }

    switch (mode_tracker_.mode()) {
      case UavControlMode::Hold:
      case UavControlMode::Takeoff:
      case UavControlMode::Position:
      case UavControlMode::VelocityBody:
        break;
      case UavControlMode::Px4PositionHold:
      case UavControlMode::Landing:
        return false;
    }

    const float xy_mps = std::hypot(
      current_execution_body_velocity_flu_[0], current_execution_body_velocity_flu_[1]);
    const float abs_z_mps = std::abs(current_execution_body_velocity_flu_[2]);

    if (exceedsPositiveLimit(xy_mps, motion_guard_feedback_hard_xy_mps_)) {
      triggerMotionGuardLanding("px4 execution body velocity xy exceeded hard limit");
      return true;
    }
    if (exceedsPositiveLimit(abs_z_mps, motion_guard_feedback_hard_z_mps_)) {
      triggerMotionGuardLanding("px4 execution body velocity z exceeded hard limit");
      return true;
    }

    return false;
  }

  void clearOvFault(const char * reason)
  {
    if (!ov_fault_active_) {
      return;
    }

    ov_fault_active_ = false;
    ov_fault_reason_.clear();
    if (reason != nullptr && reason[0] != '\0') {
      RCLCPP_INFO(this->get_logger(), "%s", reason);
    }
  }

  void setOvFault(const std::string & reason)
  {
    const bool reason_changed = !ov_fault_active_ || ov_fault_reason_ != reason;
    ov_fault_active_ = true;
    ov_fault_reason_ = reason;
    if (reason_changed) {
      RCLCPP_ERROR(this->get_logger(), "%s", ov_fault_reason_.c_str());
    }

    if (!ovAutomaticControlActive()) {
      return;
    }

    takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    requestPx4PositionMode("ov fault");
  }

  bool ensureOvControlStateAvailable(std::string & message) const
  {
    if (ov_fault_active_) {
      message = "OV control state fault";
      if (!ov_fault_reason_.empty()) {
        message += ": ";
        message += ov_fault_reason_;
      }
      return false;
    }
    if (!has_state_ || !isFiniteVector(current_position_enu_)) {
      message = "OV control state is not available yet";
      return false;
    }
    if (!controlStateFresh()) {
      message = "OV control state is stale";
      return false;
    }
    return true;
  }

  bool ensureVelocityExecutionAvailable(const char * context, bool exit_to_position_on_loss)
  {
    if (isPx4VelocityExecutionReady()) {
      velocity_execution_ready_once_ = true;
      return true;
    }

    if (velocity_execution_ready_once_ && exit_to_position_on_loss) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "PX4 execution state lost during %s, switching to px4 position mode",
        context);
      requestPx4PositionMode("execution state lost");
      return false;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "waiting for PX4 execution state before %s",
      context);
    return false;
  }

  void monitorOvControlHealth()
  {
    if (!ovAutomaticControlActive()) {
      return;
    }

    if (!has_state_) {
      setOvFault("OV control state is unavailable during automatic control");
      return;
    }

    if (!controlStateFresh()) {
      setOvFault("OV control state timed out during automatic control");
    }
  }

  void publishVehicleCommand(
    uint32_t command,
    float param1,
    float param2,
    float param3 = 0.0f,
    float param4 = 0.0f)
  {
    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("vehicle command", stamp)) {
      return;
    }
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = stamp;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = target_component_;
    msg.source_system = source_system_;
    msg.source_component = source_component_;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void requestPx4PositionMode(const char * reason)
  {
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestPx4PositionHold();
    position_mode_retry_counter_ = 0;
    clearManualCommands();
    logModeChange(previous, reason);
  }

  void forcePx4PositionMode(const char * reason)
  {
    const auto previous = mode_tracker_.mode();
    mode_tracker_.forcePx4PositionHold();
    position_mode_retry_counter_ = 0;
    land_command_sent_ = false;
    landing_retry_counter_ = 0;
    clearManualCommands();
    logModeChange(previous, reason);
  }

  bool startTakeoff(const char * reason, std::string & message)
  {
    if (!captureCurrentPose(message)) {
      return false;
    }
    clearMotionGuardSoftViolation();
    hold_target_position_enu_[2] += takeoff_height_m_;
    takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestTakeoff();
    logModeChange(previous, reason);
    message = "takeoff started";
    return true;
  }

  void enterLandingMode(const char * reason)
  {
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestLanding();
    land_command_sent_ = false;
    landing_retry_counter_ = 0;
    logModeChange(previous, reason);
  }

  void enterExternalDisarmSafeState()
  {
    resetMotionGuardState();
    takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    forcePx4PositionMode("external disarm");
    resetWarmup();
  }

  void maybeEnableOffboardAndArm()
  {
    if (setpoint_counter_ < warmup_cycles_) {
      ++setpoint_counter_;
      return;
    }
    if (!isPx4ExecutionReady()) {
      ++setpoint_counter_;
      return;
    }
    const int cycles_after_warmup = setpoint_counter_ - warmup_cycles_;
    constexpr int kRetryIntervalCycles = 50;
    if ((!is_offboard_mode_ || !is_armed_) && (cycles_after_warmup % kRetryIntervalCycles == 0)) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
    }
    ++setpoint_counter_;
  }

  bool publishVelocityBodySetpoint(const geometry_msgs::msg::TwistStamped & cmd)
  {
    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("velocity setpoint", stamp) || !has_execution_state_) {
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

    if (!isPx4VelocityExecutionReady()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "waiting for PX4 execution state before velocity offboard setpoint");
      return false;
    }

    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = stamp;
    offboard_mode.position = false;
    offboard_mode.velocity = true;
    offboard_mode.acceleration = false;
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

    if (!isFiniteVelocitySetpoint(velocity_ned, last_velocity_mode_yawspeed_ned_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting non-finite NED velocity setpoint");
      return false;
    }

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = stamp;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    setpoint.position = {nan, nan, nan};
    setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
    setpoint.acceleration = {nan, nan, nan};
    setpoint.jerk = {nan, nan, nan};
    setpoint.yaw = nan;
    setpoint.yawspeed = last_velocity_mode_yawspeed_ned_;
    trajectory_setpoint_pub_->publish(setpoint);
    maybeEnableOffboardAndArm();
    return true;
  }

  bool buildOvOuterLoopCommand(
    const std::array<float, 3> & target_position_enu,
    float target_yaw_enu,
    geometry_msgs::msg::TwistStamped & cmd,
    float * height_error_m = nullptr,
    float * xy_error_m = nullptr,
    float * yaw_error_rad = nullptr) const
  {
    if (!has_state_ || !controlStateFresh() ||
      !isFiniteVector(current_position_enu_) || !std::isfinite(current_yaw_enu_))
    {
      return false;
    }

    const float ex_world = target_position_enu[0] - current_position_enu_[0];
    const float ey_world = target_position_enu[1] - current_position_enu_[1];
    const float ez_world = target_position_enu[2] - current_position_enu_[2];
    const float yaw_error = normalizeAngle(target_yaw_enu - current_yaw_enu_);

    const float cos_yaw = std::cos(current_yaw_enu_);
    const float sin_yaw = std::sin(current_yaw_enu_);
    const float ex_body = (cos_yaw * ex_world) + (sin_yaw * ey_world);
    const float ey_body = (-sin_yaw * ex_world) + (cos_yaw * ey_world);

    float vx_body = clampSigned(ov_hold_kp_xy_ * ex_body, ov_hold_max_vxy_);
    float vy_body = clampSigned(ov_hold_kp_xy_ * ey_body, ov_hold_max_vxy_);
    float vz_body = clampSigned(ov_hold_kp_z_ * ez_world, ov_hold_max_vz_);
    float yaw_rate = clampSigned(ov_hold_kp_yaw_ * yaw_error, ov_hold_max_yaw_rate_);

    const float xy_error = std::hypot(ex_world, ey_world);
    if (xy_error <= ov_target_xy_tolerance_m_) {
      vx_body = 0.0f;
      vy_body = 0.0f;
    }
    if (std::abs(ez_world) <= ov_target_z_tolerance_m_) {
      vz_body = 0.0f;
    }
    if (std::abs(yaw_error) <= ov_target_yaw_tolerance_rad_) {
      yaw_rate = 0.0f;
    }

    cmd.header.stamp = this->now();
    cmd.header.frame_id = base_frame_id_;
    cmd.twist.linear.x = vx_body;
    cmd.twist.linear.y = vy_body;
    cmd.twist.linear.z = vz_body;
    cmd.twist.angular.z = yaw_rate;

    if (height_error_m != nullptr) {
      *height_error_m = std::abs(ez_world);
    }
    if (xy_error_m != nullptr) {
      *xy_error_m = xy_error;
    }
    if (yaw_error_rad != nullptr) {
      *yaw_error_rad = std::abs(yaw_error);
    }
    return true;
  }

  bool publishOvOuterLoopTarget(
    const std::array<float, 3> & target_position_enu,
    float target_yaw_enu,
    const char * context,
    float * height_error_m = nullptr,
    float * xy_error_m = nullptr,
    float * yaw_error_rad = nullptr)
  {
    if (!ensureVelocityExecutionAvailable(context, true)) {
      return false;
    }

    geometry_msgs::msg::TwistStamped cmd;
    if (!buildOvOuterLoopCommand(
        target_position_enu, target_yaw_enu, cmd,
        height_error_m, xy_error_m, yaw_error_rad))
    {
      return false;
    }

    return publishVelocityBodySetpoint(cmd);
  }

  void applyVelocityModeRateLimit(std::array<float, 3> & velocity_ned, double yaw_rate_enu)
  {
    const float dt = publish_rate_hz_ > 1.0e-6 ? static_cast<float>(1.0 / publish_rate_hz_) : 0.02f;
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

  bool captureCurrentPose(std::string & message)
  {
    if (!ensureOvControlStateAvailable(message)) {
      return false;
    }
    hold_target_position_enu_ = current_position_enu_;
    hold_target_yaw_enu_ = current_yaw_enu_;
    hold_target_valid_ = true;
    return true;
  }

  void handlePositionTarget(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!position_command_frame_id_.empty() &&
      !msg.header.frame_id.empty() &&
      msg.header.frame_id != position_command_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring position target frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), position_command_frame_id_.c_str());
      return;
    }

    if (motion_guard_tripped_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring position target while motion guard is latched");
      return;
    }

    if (ov_fault_active_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring position target while OV control state fault is active: %s",
        ov_fault_reason_.c_str());
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

    float target_yaw_enu = position_target_yaw_enu_;
    const double quat_norm =
      (msg.pose.orientation.x * msg.pose.orientation.x) +
      (msg.pose.orientation.y * msg.pose.orientation.y) +
      (msg.pose.orientation.z * msg.pose.orientation.z) +
      (msg.pose.orientation.w * msg.pose.orientation.w);
    if (quat_norm > 1.0e-6) {
      target_yaw_enu = quaternionToYaw(msg.pose.orientation);
    } else if (has_state_) {
      target_yaw_enu = current_yaw_enu_;
    } else if (has_last_accepted_pose_target_) {
      target_yaw_enu = last_accepted_pose_target_yaw_enu_;
    }

    const auto now = this->now();
    if (motion_guard_enabled_) {
      MotionGuardViolation combined_violation = MotionGuardViolation::None;
      const char * combined_reason = nullptr;

      const bool allow_pose_history_reference = motionGuardActive();
      bool has_recent_pose_reference =
        allow_pose_history_reference &&
        has_last_accepted_pose_target_ &&
        last_accepted_pose_target_time_.nanoseconds() != 0;
      double pose_dt_s = 0.0;
      if (has_recent_pose_reference) {
        pose_dt_s = (now - last_accepted_pose_target_time_).seconds();
        has_recent_pose_reference =
          std::isfinite(pose_dt_s) &&
          pose_dt_s >= 0.0 &&
          pose_dt_s <= motion_guard_pose_gap_reset_s_;
      }

      if (has_last_accepted_pose_target_ && !has_recent_pose_reference) {
        clearMotionGuardSoftViolation();
      }

      if (has_recent_pose_reference && pose_dt_s > 1.0e-6) {
        const float dx = target_position_enu[0] - last_accepted_pose_target_position_enu_[0];
        const float dy = target_position_enu[1] - last_accepted_pose_target_position_enu_[1];
        const float dz = target_position_enu[2] - last_accepted_pose_target_position_enu_[2];
        const float xy_speed_mps = std::hypot(dx, dy) / static_cast<float>(pose_dt_s);
        const float abs_z_speed_mps = std::abs(dz) / static_cast<float>(pose_dt_s);
        const float abs_yaw_rate_radps = std::abs(
          normalizeAngle(target_yaw_enu - last_accepted_pose_target_yaw_enu_)) /
          static_cast<float>(pose_dt_s);
        const char * velocity_reason = nullptr;
        updateMotionGuardViolation(
          classifyVelocityEnvelope(xy_speed_mps, abs_z_speed_mps, abs_yaw_rate_radps, velocity_reason),
          velocity_reason, combined_violation, combined_reason);
      }

      std::array<float, 3> pose_reference_position_enu{};
      float pose_reference_yaw_enu = 0.0f;
      bool has_pose_step_reference = false;
      if (has_recent_pose_reference) {
        pose_reference_position_enu = last_accepted_pose_target_position_enu_;
        pose_reference_yaw_enu = last_accepted_pose_target_yaw_enu_;
        has_pose_step_reference = true;
      } else if (has_state_ && controlStateFresh() && isFiniteVector(current_position_enu_)) {
        pose_reference_position_enu = current_position_enu_;
        pose_reference_yaw_enu = current_yaw_enu_;
        has_pose_step_reference = true;
      }

      if (!has_pose_step_reference) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "ignoring position target without a fresh pose reference for motion guard");
        return;
      }

      const float dx = target_position_enu[0] - pose_reference_position_enu[0];
      const float dy = target_position_enu[1] - pose_reference_position_enu[1];
      const float dz = target_position_enu[2] - pose_reference_position_enu[2];
      const float xy_step_m = std::hypot(dx, dy);
      const float abs_z_step_m = std::abs(dz);
      const float abs_yaw_step_rad = std::abs(
        normalizeAngle(target_yaw_enu - pose_reference_yaw_enu));
      const char * step_reason = nullptr;
      updateMotionGuardViolation(
        classifyPoseStepEnvelope(xy_step_m, abs_z_step_m, abs_yaw_step_rad, step_reason),
        step_reason, combined_violation, combined_reason);

      const MotionGuardAction guard_action = motionGuardActionForViolation(
        combined_violation, combined_reason, now);
      if (guard_action == MotionGuardAction::Reject) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rejecting position target before motion guard activation: %s",
          combined_reason != nullptr ? combined_reason : "motion guard violation");
        return;
      }
      if (guard_action == MotionGuardAction::Trip) {
        return;
      }
    }

    position_target_position_enu_ = target_position_enu;
    position_target_yaw_enu_ = target_yaw_enu;
    position_target_valid_ = true;
    if (motionGuardActive()) {
      last_accepted_pose_target_position_enu_ = target_position_enu;
      last_accepted_pose_target_yaw_enu_ = target_yaw_enu;
      last_accepted_pose_target_time_ = now;
      has_last_accepted_pose_target_ = true;
    } else {
      has_last_accepted_pose_target_ = false;
      last_accepted_pose_target_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestPosition();
    logModeChange(previous, "position target");
  }

  void handleVelocityBodyTarget(const geometry_msgs::msg::TwistStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring velocity_body frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }

    if (motion_guard_tripped_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring velocity_body command while motion guard is latched");
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

    if (!isPx4VelocityExecutionReady()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring velocity_body command until PX4 execution state is valid");
      return;
    }

    if (motion_guard_enabled_) {
      const float xy_mps = std::hypot(
        static_cast<float>(msg.twist.linear.x),
        static_cast<float>(msg.twist.linear.y));
      const float abs_z_mps = std::abs(static_cast<float>(msg.twist.linear.z));
      const float abs_yaw_radps = std::abs(static_cast<float>(msg.twist.angular.z));
      const auto now = this->now();
      const char * reason = nullptr;
      const MotionGuardViolation violation = classifyVelocityEnvelope(
        xy_mps, abs_z_mps, abs_yaw_radps, reason);
      const MotionGuardAction guard_action = motionGuardActionForViolation(
        violation, reason, now);
      if (guard_action == MotionGuardAction::Reject) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rejecting velocity_body command before motion guard activation: %s",
          reason != nullptr ? reason : "motion guard violation");
        return;
      }
      if (guard_action == MotionGuardAction::Trip) {
        return;
      }
    }

    last_velocity_body_cmd_ = msg;
    last_velocity_body_time_us_ = nowMicros();
    has_velocity_body_cmd_ = true;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestVelocityBody();
    logModeChange(previous, "velocity body");
  }

  bool handleTakeoffRequest(std::string & message)
  {
    if (rejectIfMotionGuardTripped("takeoff", message)) {
      return false;
    }
    if (rejectIfLandingLocked("takeoff", message)) {
      return false;
    }

    return startTakeoff("takeoff", message);
  }

  bool handleHoldRequest(std::string & message)
  {
    if (rejectIfMotionGuardTripped("hold", message)) {
      return false;
    }
    if (rejectIfLandingLocked("hold", message)) {
      return false;
    }
    if (!captureCurrentPose(message)) {
      return false;
    }
    clearMotionGuardSoftViolation();
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestHold();
    logModeChange(previous, "hold");
    message = "hold target captured";
    return true;
  }

  bool handleLandRequest(std::string & message)
  {
    if (mode_tracker_.mode() == UavControlMode::Landing) {
      message = "landing already in progress";
      return true;
    }
    std::string ignored;
    if (!captureCurrentPose(ignored)) {
      hold_target_valid_ = false;
    }
    clearMotionGuardSoftViolation();
    enterLandingMode("land");
    message = hold_target_valid_ ?
      "landing requested with OV hold fallback" :
      "landing requested without OV hold fallback";
    return true;
  }

  bool handlePositionModeRequest(std::string & message)
  {
    if (rejectIfMotionGuardTripped("position_mode", message)) {
      return false;
    }
    if (mode_tracker_.mode() == UavControlMode::Landing) {
      forcePx4PositionMode("px4 position mode");
      message = "landing interrupted, switching to px4 position mode";
      return true;
    }
    requestPx4PositionMode("px4 position mode");
    message = "px4 position mode requested";
    return true;
  }

  bool handleAbortRequest(std::string & message)
  {
    if (mode_tracker_.mode() == UavControlMode::Landing) {
      forcePx4PositionMode("abort");
      message = "landing interrupted, switching to px4 position mode";
      return true;
    }
    requestPx4PositionMode("abort");
    message = "abort requested, switching to px4 position mode";
    return true;
  }

  bool handleDisarmRequest(std::string & message)
  {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
    message = "disarm requested";
    return true;
  }

  void publishPositionTarget()
  {
    if (!position_target_valid_) {
      return;
    }
    publishOvOuterLoopTarget(
      position_target_position_enu_,
      position_target_yaw_enu_,
      "OV position control");
  }

  void publishHoldTarget()
  {
    if (!hold_target_valid_) {
      return;
    }
    publishOvOuterLoopTarget(
      hold_target_position_enu_,
      hold_target_yaw_enu_,
      "OV hold control");
  }

  void handleTakeoffMode()
  {
    if (!hold_target_valid_ || !has_state_) {
      return;
    }
    float height_error = std::numeric_limits<float>::infinity();
    if (!publishOvOuterLoopTarget(
        hold_target_position_enu_,
        hold_target_yaw_enu_,
        "OV takeoff control",
        &height_error))
    {
      return;
    }
    if (height_error <= takeoff_height_tolerance_m_) {
      if (takeoff_reached_since_.nanoseconds() == 0) {
        takeoff_reached_since_ = this->now();
      } else if ((this->now() - takeoff_reached_since_).seconds() >= takeoff_stable_s_) {
        const auto previous = mode_tracker_.mode();
        mode_tracker_.requestHold();
        logModeChange(previous, "takeoff reached");
      }
    } else {
      takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  void handleVelocityBodyMode()
  {
    if (!ensureVelocityExecutionAvailable("velocity body control", true)) {
      return;
    }

    if (!has_velocity_body_cmd_) {
      return;
    }
    const uint64_t now_us = nowMicros();
    if (!ensureTimestampReady("velocity command", now_us)) {
      return;
    }
    last_velocity_body_time_us_ = initializeVelocityBodyCommandTimestamp(
      now_us, last_velocity_body_time_us_);
    if (isVelocityBodyCommandTimedOut(now_us, last_velocity_body_time_us_, velocity_body_timeout_ms_)) {
      has_velocity_body_cmd_ = false;
      last_velocity_body_time_us_ = 0;
      mode_tracker_.onVelocityCommandTimeout();
      std::string ignored;
      handleHoldRequest(ignored);
      return;
    }
    publishVelocityBodySetpoint(last_velocity_body_cmd_);
  }

  void handleLandingMode()
  {
    constexpr int kRetryIntervalCycles = 50;
    const bool is_auto_land_mode =
      current_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
    if (!land_command_sent_ || (!is_auto_land_mode && landing_retry_counter_ % kRetryIntervalCycles == 0)) {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
        0.0f, 0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN());
      land_command_sent_ = true;
    }
    if (!is_auto_land_mode && is_armed_ && hold_target_valid_ &&
      !ov_fault_active_ && controlStateFresh() &&
      ensureVelocityExecutionAvailable("landing hold fallback", false))
    {
      geometry_msgs::msg::TwistStamped cmd;
      if (buildOvOuterLoopCommand(hold_target_position_enu_, hold_target_yaw_enu_, cmd)) {
        publishVelocityBodySetpoint(cmd);
      }
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

  void timerCallback()
  {
    enforceFeedbackMotionGuard();
    monitorOvControlHealth();

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
      case UavControlMode::VelocityBody:
        handleVelocityBodyMode();
        break;
      case UavControlMode::Landing:
        handleLandingMode();
        break;
    }
  }

  std::string position_topic_;
  std::string velocity_body_topic_;
  std::string state_topic_;
  std::string execution_state_topic_;
  std::string offboard_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_command_topic_;
  std::string vehicle_status_topic_;
  std::string takeoff_service_;
  std::string hold_service_;
  std::string position_mode_service_;
  std::string land_service_;
  std::string abort_service_;
  std::string disarm_service_;
  std::string base_frame_id_;
  std::string position_command_frame_id_;
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
  double execution_state_timeout_s_{0.20};
  float ov_hold_kp_xy_{0.8f};
  float ov_hold_kp_z_{0.8f};
  float ov_hold_kp_yaw_{1.0f};
  float ov_hold_max_vxy_{0.4f};
  float ov_hold_max_vz_{0.2f};
  float ov_hold_max_yaw_rate_{0.4f};
  float ov_target_xy_tolerance_m_{0.05f};
  float ov_target_z_tolerance_m_{0.05f};
  float ov_target_yaw_tolerance_rad_{0.08f};
  double ov_fault_pose_timeout_s_{0.20};
  float ov_fault_max_xy_step_m_{0.30f};
  float ov_fault_max_z_step_m_{0.20f};
  float ov_fault_max_yaw_step_rad_{0.35f};
  bool motion_guard_enabled_{true};
  double motion_guard_soft_dwell_s_{2.0};
  double motion_guard_pose_gap_reset_s_{0.40};
  float motion_guard_soft_xy_mps_{0.40f};
  float motion_guard_soft_z_mps_{0.25f};
  float motion_guard_soft_yaw_radps_{0.60f};
  float motion_guard_hard_xy_mps_{0.55f};
  float motion_guard_hard_z_mps_{0.35f};
  float motion_guard_hard_yaw_radps_{0.90f};
  float motion_guard_feedback_hard_xy_mps_{0.65f};
  float motion_guard_feedback_hard_z_mps_{0.45f};
  float motion_guard_pose_soft_xy_step_m_{0.25f};
  float motion_guard_pose_soft_z_step_m_{0.12f};
  float motion_guard_pose_soft_yaw_step_rad_{0.35f};
  float motion_guard_pose_hard_xy_step_m_{0.50f};
  float motion_guard_pose_hard_z_step_m_{0.25f};
  float motion_guard_pose_hard_yaw_step_rad_{0.70f};
  Px4TimestampSource px4_timestamp_source_{Px4TimestampSource::System};
  rclcpp::Clock px4_timestamp_system_clock_{RCL_SYSTEM_TIME};
  UavControlModeTracker mode_tracker_{};
  geometry_msgs::msg::TwistStamped last_velocity_body_cmd_{};
  bool has_velocity_body_cmd_{false};
  uint64_t last_velocity_body_time_us_{0};
  std::array<float, 3> last_velocity_mode_ned_{0.0f, 0.0f, 0.0f};
  float last_velocity_mode_yawspeed_ned_{0.0f};
  bool velocity_mode_rate_limit_initialized_{false};
  std::array<float, 3> current_position_enu_{0.0f, 0.0f, 0.0f};
  tf2::Quaternion current_orientation_enu_flu_{0.0, 0.0, 0.0, 1.0};
  float current_yaw_enu_{0.0f};
  bool has_state_{false};
  bool has_execution_state_{false};
  bool ov_fault_active_{false};
  bool has_last_ov_sample_{false};
  bool motion_guard_tripped_{false};
  bool is_armed_{false};
  bool is_offboard_mode_{false};
  bool velocity_execution_ready_once_{false};
  uint8_t current_nav_state_{px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL};
  std::string ov_fault_reason_;
  std::string motion_guard_trip_reason_;
  std::array<float, 3> current_execution_body_velocity_flu_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> last_ov_sample_position_enu_{0.0f, 0.0f, 0.0f};
  tf2::Quaternion current_execution_orientation_enu_flu_{0.0, 0.0, 0.0, 1.0};
  float current_execution_yaw_enu_{0.0f};
  float last_ov_sample_yaw_enu_{0.0f};
  std::array<float, 3> hold_target_position_enu_{0.0f, 0.0f, 0.0f};
  float hold_target_yaw_enu_{0.0f};
  bool hold_target_valid_{false};
  std::array<float, 3> position_target_position_enu_{0.0f, 0.0f, 0.0f};
  float position_target_yaw_enu_{0.0f};
  bool position_target_valid_{false};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_execution_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time soft_violation_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_accepted_pose_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_reached_since_{0, 0, RCL_ROS_TIME};
  int setpoint_counter_{0};
  int position_mode_retry_counter_{0};
  std::array<float, 3> last_accepted_pose_target_position_enu_{0.0f, 0.0f, 0.0f};
  float last_accepted_pose_target_yaw_enu_{0.0f};
  bool has_last_accepted_pose_target_{false};
  bool land_command_sent_{false};
  int landing_retry_counter_{0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr execution_state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<Trigger>::SharedPtr position_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr land_srv_;
  rclcpp::Service<Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<Trigger>::SharedPtr disarm_srv_;
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
