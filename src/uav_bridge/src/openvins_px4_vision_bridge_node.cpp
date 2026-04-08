#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "uav_bridge/math_utils.hpp"
#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{

class OpenVinsPx4VisionBridgeNode : public rclcpp::Node
{
public:
  OpenVinsPx4VisionBridgeNode()
  : Node("openvins_px4_vision_bridge_node")
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

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OpenVinsPx4VisionBridgeNode::odometryCallback, this, std::placeholders::_1));

    if (!reset_counter_bump_topic_.empty()) {
      reset_counter_bump_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        reset_counter_bump_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Empty::SharedPtr)
        {
          ++reset_counter_;
          last_timestamp_sample_ = 0U;
          RCLCPP_INFO(
            this->get_logger(),
            "received OpenVINS reset counter bump, reset_counter=%u",
            reset_counter_);
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

    if (sensor_position_in_body_.length2() > 1.0e-12) {
      RCLCPP_WARN(
        this->get_logger(),
        "sensor_xyz_in_body_m is ignored by openvins_px4_vision_bridge_node; "
        "configure EKF2_EV_POS_* on PX4 for EV lever-arm compensation");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "openvins_px4_vision_bridge_node: odom=%s -> %s, bridge-side EV translation compensation disabled",
      odometry_topic_.c_str(), visual_odometry_topic_.c_str());
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

  uint64_t stampToPx4Micros(const builtin_interfaces::msg::Time & stamp)
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

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    warnUnexpectedFrames(*msg);
    if (!ensureTimestampSourceReady("EV odometry")) {
      return;
    }

    px4_msgs::msg::VehicleOdometry out{};
    out.timestamp = nowPx4Micros();

    uint64_t timestamp_sample = 0U;
    if (use_odometry_stamp_for_timestamp_sample_) {
      timestamp_sample = stampToPx4Micros(msg->header.stamp);
    }
    if (timestamp_sample == 0U) {
      timestamp_sample = out.timestamp;
    }
    if (last_timestamp_sample_ != 0U && timestamp_sample < last_timestamp_sample_) {
      ++reset_counter_;
      RCLCPP_WARN(
        this->get_logger(), "odometry timestamp moved backwards (%lu -> %lu), reset_counter=%u",
        static_cast<unsigned long>(last_timestamp_sample_),
        static_cast<unsigned long>(timestamp_sample),
        reset_counter_);
    }
    last_timestamp_sample_ = timestamp_sample;
    out.timestamp_sample = timestamp_sample;

    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    out.velocity = invalidVector3();
    out.angular_velocity = invalidVector3();
    out.q = invalidQuaternionArray();

    const tf2::Vector3 position_enu(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    const tf2::Quaternion orientation_enu_sensor(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

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
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "received invalid OpenVINS orientation, publishing position/velocity without attitude");
    }

    out.position = enuPositionToNed(
      static_cast<float>(position_enu.x()),
      static_cast<float>(position_enu.y()),
      static_cast<float>(position_enu.z()));

    const std::array<float, 3> velocity_enu = {
      static_cast<float>(msg->twist.twist.linear.x),
      static_cast<float>(msg->twist.twist.linear.y),
      static_cast<float>(msg->twist.twist.linear.z)};
    if (isFiniteVector(velocity_enu)) {
      out.velocity = enuPositionToNed(velocity_enu[0], velocity_enu[1], velocity_enu[2]);
      out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    }

    out.position_variance = {
      covarianceOrFloor(msg->pose.covariance[0], position_variance_floor_m2_),
      covarianceOrFloor(msg->pose.covariance[7], position_variance_floor_m2_),
      covarianceOrFloor(msg->pose.covariance[14], position_variance_floor_m2_)};
    out.orientation_variance = {
      covarianceOrFloor(msg->pose.covariance[21], orientation_variance_floor_rad2_),
      covarianceOrFloor(msg->pose.covariance[28], orientation_variance_floor_rad2_),
      covarianceOrFloor(msg->pose.covariance[35], orientation_variance_floor_rad2_)};
    out.velocity_variance = {
      covarianceOrFloor(msg->twist.covariance[0], velocity_variance_floor_m2ps2_),
      covarianceOrFloor(msg->twist.covariance[7], velocity_variance_floor_m2ps2_),
      covarianceOrFloor(msg->twist.covariance[14], velocity_variance_floor_m2ps2_)};
    out.reset_counter = reset_counter_;
    out.quality = static_cast<int8_t>(quality_);

    visual_odometry_pub_->publish(out);

    if (log_debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "published EV odom: pos_body_ned=[%.3f %.3f %.3f] vel_ned=[%.3f %.3f %.3f] quality=%d",
        out.position[0], out.position[1], out.position[2],
        out.velocity[0], out.velocity[1], out.velocity[2], quality_);
    }
  }

  std::string odometry_topic_;
  std::string visual_odometry_topic_;
  std::string expected_odom_frame_id_;
  std::string expected_child_frame_id_;
  std::string reset_counter_bump_topic_;
  std::string timesync_status_topic_;
  float position_variance_floor_m2_{0.01f};
  float orientation_variance_floor_rad2_{0.01f};
  float velocity_variance_floor_m2ps2_{0.01f};
  int quality_{100};
  bool use_odometry_stamp_for_timestamp_sample_{true};
  bool log_debug_{false};
  double timesync_timeout_s_{0.5};
  uint64_t last_timestamp_sample_{0U};
  uint8_t reset_counter_{0U};
  Px4TimestampSource px4_timestamp_source_{Px4TimestampSource::System};
  bool has_timesync_status_{false};
  int64_t estimated_px4_offset_us_{0};
  rclcpp::Time last_timesync_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Clock px4_timestamp_system_clock_{RCL_SYSTEM_TIME};
  tf2::Vector3 sensor_position_in_body_{0.0, 0.0, 0.0};
  tf2::Quaternion sensor_rotation_in_body_{0.0, 0.0, 0.0, 1.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_counter_bump_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_pub_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::OpenVinsPx4VisionBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
