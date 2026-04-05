#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include "uav_bridge/uav_state_bridge_logic.hpp"

namespace uav_bridge
{

namespace
{

bool isFiniteScalar(double value)
{
  return std::isfinite(value);
}

bool isFiniteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!isFiniteScalar(quaternion.x) || !isFiniteScalar(quaternion.y) ||
    !isFiniteScalar(quaternion.z) || !isFiniteScalar(quaternion.w))
  {
    return false;
  }

  const double norm_sq =
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z +
    quaternion.w * quaternion.w;
  return norm_sq > 1.0e-12;
}

bool isFiniteCovariance(const std::array<double, 36> & covariance)
{
  for (const double value : covariance) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

rclcpp::Time resolveStamp(
  const builtin_interfaces::msg::Time & stamp_msg,
  const rclcpp::Clock::SharedPtr & clock)
{
  if (stamp_msg.sec == 0 && stamp_msg.nanosec == 0) {
    return clock->now();
  }
  return rclcpp::Time(stamp_msg, clock->get_clock_type());
}

}  // namespace

class OpenVinsStateBridgeNode : public rclcpp::Node
{
public:
  OpenVinsStateBridgeNode()
  : Node("openvins_state_bridge_node")
  {
    this->declare_parameter<std::string>("odometry_topic", "/ov_msckf/odomimu");
    this->declare_parameter<std::string>("output_odometry_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("expected_odom_frame_id", "global");
    this->declare_parameter<std::string>("expected_child_frame_id", "imu");
    this->declare_parameter<std::string>("output_frame_id", "global");
    this->declare_parameter<std::string>("base_frame_id", "uav_base_link");
    this->declare_parameter<double>("sensor_x_in_body_m", 0.0);
    this->declare_parameter<double>("sensor_y_in_body_m", 0.0);
    this->declare_parameter<double>("sensor_z_in_body_m", 0.0);
    this->declare_parameter<double>("sensor_roll_in_body_rad", 0.0);
    this->declare_parameter<double>("sensor_pitch_in_body_rad", 0.0);
    this->declare_parameter<double>("sensor_yaw_in_body_rad", 0.0);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("log_debug", false);

    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    output_odometry_topic_ = this->get_parameter("output_odometry_topic").as_string();
    expected_odom_frame_id_ = this->get_parameter("expected_odom_frame_id").as_string();
    expected_child_frame_id_ = this->get_parameter("expected_child_frame_id").as_string();
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    log_debug_ = this->get_parameter("log_debug").as_bool();

    sensor_position_in_body_.setValue(
      this->get_parameter("sensor_x_in_body_m").as_double(),
      this->get_parameter("sensor_y_in_body_m").as_double(),
      this->get_parameter("sensor_z_in_body_m").as_double());
    sensor_rotation_in_body_.setRPY(
      this->get_parameter("sensor_roll_in_body_rad").as_double(),
      this->get_parameter("sensor_pitch_in_body_rad").as_double(),
      this->get_parameter("sensor_yaw_in_body_rad").as_double());
    sensor_rotation_in_body_.normalize();

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OpenVinsStateBridgeNode::odometryCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odometry_topic_, 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "openvins_state_bridge_node: odom=%s -> %s, output_frame=%s base_frame=%s",
      odometry_topic_.c_str(),
      output_odometry_topic_.c_str(),
      output_frame_id_.c_str(),
      base_frame_id_.c_str());
  }

private:
  void warnUnexpectedFrames(const nav_msgs::msg::Odometry & msg)
  {
    if (!expected_odom_frame_id_.empty() && !msg.header.frame_id.empty() &&
      msg.header.frame_id != expected_odom_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "unexpected OpenVINS odom frame_id '%s', expected '%s'",
        msg.header.frame_id.c_str(),
        expected_odom_frame_id_.c_str());
    }

    if (!expected_child_frame_id_.empty() && !msg.child_frame_id.empty() &&
      msg.child_frame_id != expected_child_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "unexpected OpenVINS odom child_frame_id '%s', expected '%s'",
        msg.child_frame_id.c_str(),
        expected_child_frame_id_.c_str());
    }
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    warnUnexpectedFrames(*msg);

    const rclcpp::Time stamp = resolveStamp(msg->header.stamp, this->get_clock());
    if (last_output_stamp_.has_value() && stamp <= *last_output_stamp_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "discarding non-monotonic OpenVINS odom stamp %.9f <= %.9f",
        stamp.seconds(),
        last_output_stamp_->seconds());
      return;
    }

    if (!isFiniteScalar(msg->pose.pose.position.x) ||
      !isFiniteScalar(msg->pose.pose.position.y) ||
      !isFiniteScalar(msg->pose.pose.position.z))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "discarding OpenVINS odom with non-finite position");
      return;
    }

    if (!isFiniteQuaternion(msg->pose.pose.orientation)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "discarding OpenVINS odom with invalid orientation");
      return;
    }

    tf2::Quaternion q_world_sensor(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    q_world_sensor.normalize();

    const tf2::Quaternion q_sensor_body = sensor_rotation_in_body_.inverse();
    tf2::Quaternion q_world_body = q_world_sensor * q_sensor_body;
    q_world_body.normalize();

    const tf2::Matrix3x3 rotation_world_body(q_world_body);
    const tf2::Vector3 position_world_sensor(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    const tf2::Vector3 position_world_body =
      position_world_sensor - (rotation_world_body * sensor_position_in_body_);

    bool used_pose_diff_velocity = false;
    tf2::Vector3 velocity_world(0.0, 0.0, 0.0);
    const bool has_world_velocity =
      isFiniteScalar(msg->twist.twist.linear.x) &&
      isFiniteScalar(msg->twist.twist.linear.y) &&
      isFiniteScalar(msg->twist.twist.linear.z);
    if (has_world_velocity) {
      velocity_world.setValue(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    } else if (last_output_stamp_.has_value() && last_position_world_body_.has_value()) {
      const double dt_sec = (stamp - *last_output_stamp_).seconds();
      if (dt_sec > 1.0e-6) {
        velocity_world =
          (position_world_body - *last_position_world_body_) * (1.0 / dt_sec);
        used_pose_diff_velocity = true;
      }
    }
    const tf2::Vector3 velocity_body = rotation_world_body.transpose() * velocity_world;

    const double yaw_world = tf2::getYaw(q_world_body);
    double yaw_rate_radps = 0.0;
    if (last_yaw_world_rad_.has_value() && last_output_stamp_.has_value()) {
      yaw_rate_radps = computeYawRateRad(
        *last_yaw_world_rad_,
        yaw_world,
        (stamp - *last_output_stamp_).seconds());
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = output_frame_id_;
    out.child_frame_id = base_frame_id_;
    out.pose.pose.position.x = position_world_body.x();
    out.pose.pose.position.y = position_world_body.y();
    out.pose.pose.position.z = position_world_body.z();
    out.pose.pose.orientation.x = q_world_body.x();
    out.pose.pose.orientation.y = q_world_body.y();
    out.pose.pose.orientation.z = q_world_body.z();
    out.pose.pose.orientation.w = q_world_body.w();
    if (isFiniteCovariance(msg->pose.covariance)) {
      out.pose.covariance = msg->pose.covariance;
    }

    out.twist.twist.linear.x = velocity_body.x();
    out.twist.twist.linear.y = velocity_body.y();
    out.twist.twist.linear.z = velocity_body.z();
    out.twist.twist.angular.z = yaw_rate_radps;
    if (isFiniteCovariance(msg->twist.covariance)) {
      out.twist.covariance = msg->twist.covariance;
    }

    odom_pub_->publish(out);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = out.header.stamp;
      tf_msg.header.frame_id = out.header.frame_id;
      tf_msg.child_frame_id = out.child_frame_id;
      tf_msg.transform.translation.x = out.pose.pose.position.x;
      tf_msg.transform.translation.y = out.pose.pose.position.y;
      tf_msg.transform.translation.z = out.pose.pose.position.z;
      tf_msg.transform.rotation = out.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }

    last_output_stamp_ = stamp;
    last_position_world_body_ = position_world_body;
    last_yaw_world_rad_ = yaw_world;

    if (log_debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "published unified state: pos=[%.3f %.3f %.3f] vel_body=[%.3f %.3f %.3f] yaw_rate=%.3f%s",
        out.pose.pose.position.x,
        out.pose.pose.position.y,
        out.pose.pose.position.z,
        out.twist.twist.linear.x,
        out.twist.twist.linear.y,
        out.twist.twist.linear.z,
        out.twist.twist.angular.z,
        used_pose_diff_velocity ? " (pose-diff velocity)" : "");
    }
  }

  std::string odometry_topic_;
  std::string output_odometry_topic_;
  std::string expected_odom_frame_id_;
  std::string expected_child_frame_id_;
  std::string output_frame_id_;
  std::string base_frame_id_;
  bool publish_tf_{true};
  bool log_debug_{false};
  tf2::Vector3 sensor_position_in_body_;
  tf2::Quaternion sensor_rotation_in_body_{0.0, 0.0, 0.0, 1.0};
  std::optional<rclcpp::Time> last_output_stamp_;
  std::optional<tf2::Vector3> last_position_world_body_;
  std::optional<double> last_yaw_world_rad_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::OpenVinsStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
