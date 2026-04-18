#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uav_bridge/math_utils.hpp"
#include "uav_bridge/srv/body_position_delta.hpp"

namespace uav_bridge
{

namespace
{

bool hasNonNegativeStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0;
}

}  // namespace

class PositionDeltaNode : public rclcpp::Node
{
  using BodyPositionDelta = uav_bridge::srv::BodyPositionDelta;

public:
  PositionDeltaNode()
  : Node("position_delta_node")
  {
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry_px4");
    this->declare_parameter<std::string>(
      "delta_service", "/uav/control/command/position_delta");
    this->declare_parameter<std::string>("output_position_topic", "/uav/control/position_keep_yaw");
    this->declare_parameter<std::string>("odom_frame_id", "uav_odom");
    this->declare_parameter<double>("state_timeout_s", 0.20);

    state_topic_ = this->get_parameter("state_topic").as_string();
    delta_service_ = this->get_parameter("delta_service").as_string();
    output_position_topic_ = this->get_parameter("output_position_topic").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    state_timeout_s_ = std::max(0.0, this->get_parameter("state_timeout_s").as_double());

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        handleState(*msg);
      });

    output_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      output_position_topic_, 10);
    delta_srv_ = this->create_service<BodyPositionDelta>(
      delta_service_,
      [this](
        const std::shared_ptr<BodyPositionDelta::Request> request,
        std::shared_ptr<BodyPositionDelta::Response> response)
      {
        handleDeltaService(*request, *response);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "position_delta_node: service=%s state=%s output=%s body_frame=FLU x-forward y-left z-up odom_frame=%s",
      delta_service_.c_str(),
      state_topic_.c_str(),
      output_position_topic_.c_str(),
      odom_frame_id_.c_str());
  }

private:
  bool stateFresh() const
  {
    if (!has_state_ || last_state_time_.nanoseconds() == 0) {
      return false;
    }
    if (state_timeout_s_ <= 0.0) {
      return true;
    }
    const double age_s = (this->now() - last_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= state_timeout_s_;
  }

  bool currentStateAvailable(std::string & message) const
  {
    if (!has_state_) {
      message = "state not available yet";
      return false;
    }
    if (!stateFresh()) {
      message = "state is stale";
      return false;
    }
    if (!isFiniteVector(current_position_enu_) || !std::isfinite(current_yaw_enu_)) {
      message = "state contains invalid pose";
      return false;
    }
    message.clear();
    return true;
  }

  std::string resolveOutputFrameId() const
  {
    return odom_frame_id_;
  }

  std::array<float, 3> bodyDeltaToEnu(const std::array<float, 3> & delta_body_flu) const
  {
    const float cos_yaw = std::cos(current_yaw_enu_);
    const float sin_yaw = std::sin(current_yaw_enu_);

    // Relative nudges are intentionally yaw-only so roll/pitch tilt does not skew XY semantics.
    return {
      (cos_yaw * delta_body_flu[0]) - (sin_yaw * delta_body_flu[1]),
      (sin_yaw * delta_body_flu[0]) + (cos_yaw * delta_body_flu[1]),
      delta_body_flu[2]};
  }

  bool submitBodyDelta(
    const std::array<float, 3> & raw_delta_body_flu,
    const char * source,
    std::string & message,
    std::array<float, 3> * target_position_enu)
  {
    std::string state_message;
    if (!currentStateAvailable(state_message)) {
      message = state_message;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting position delta from %s: %s",
        source,
        state_message.c_str());
      return false;
    }

    if (!isFiniteVector(raw_delta_body_flu)) {
      message = "delta contains non-finite values";
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting non-finite position delta from %s",
        source);
      return false;
    }

    const std::array<float, 3> delta_enu = bodyDeltaToEnu(raw_delta_body_flu);
    *target_position_enu = {
      current_position_enu_[0] + delta_enu[0],
      current_position_enu_[1] + delta_enu[1],
      current_position_enu_[2] + delta_enu[2]};
    message = "body-relative delta accepted";
    return true;
  }

  void publishTarget(
    const std::array<float, 3> & target_position_enu,
    const char * source,
    const std::array<float, 3> & raw_delta_body_flu)
  {
    geometry_msgs::msg::PointStamped output;
    output.header.stamp = this->now();
    output.header.frame_id = resolveOutputFrameId();
    output.point.x = target_position_enu[0];
    output.point.y = target_position_enu[1];
    output.point.z = target_position_enu[2];
    output_pub_->publish(output);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "position delta from %s body=[%+.3f %+.3f %+.3f] -> absolute target [%.3f %.3f %.3f] frame=%s",
      source,
      raw_delta_body_flu[0], raw_delta_body_flu[1], raw_delta_body_flu[2],
      output.point.x, output.point.y, output.point.z,
      output.header.frame_id.c_str());
  }

  void handleState(const nav_msgs::msg::Odometry & msg)
  {
    const std::array<float, 3> position_enu = {
      static_cast<float>(msg.pose.pose.position.x),
      static_cast<float>(msg.pose.pose.position.y),
      static_cast<float>(msg.pose.pose.position.z)};
    if (!isFiniteVector(position_enu)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring state with non-finite position");
      return;
    }

    const double quat_norm_sq =
      (msg.pose.pose.orientation.x * msg.pose.pose.orientation.x) +
      (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y) +
      (msg.pose.pose.orientation.z * msg.pose.pose.orientation.z) +
      (msg.pose.pose.orientation.w * msg.pose.pose.orientation.w);
    if (!std::isfinite(quat_norm_sq) || quat_norm_sq <= 1.0e-12) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring state with invalid orientation");
      return;
    }

    const bool has_stamp =
      (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
    if (has_stamp && !hasNonNegativeStamp(msg.header.stamp)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring state with negative stamp sec=%d nanosec=%u",
        msg.header.stamp.sec,
        msg.header.stamp.nanosec);
      return;
    }
    current_position_enu_ = position_enu;
    current_yaw_enu_ = quaternionToYaw(msg.pose.pose.orientation);
    last_state_time_ = has_stamp ?
      rclcpp::Time(msg.header.stamp, this->get_clock()->get_clock_type()) :
      this->now();
    has_state_ = std::isfinite(current_yaw_enu_);
  }

  void handleDeltaService(
    const BodyPositionDelta::Request & request,
    BodyPositionDelta::Response & response)
  {
    const std::array<float, 3> raw_delta_body_flu = {
      request.x,
      request.y,
      request.z};
    std::array<float, 3> target_position_enu{};
    std::string message;
    response.success = submitBodyDelta(
      raw_delta_body_flu, "service", message, &target_position_enu);
    response.message = message;
    if (!response.success) {
      return;
    }

    publishTarget(target_position_enu, "service", raw_delta_body_flu);
    response.target_x = target_position_enu[0];
    response.target_y = target_position_enu[1];
    response.target_z = target_position_enu[2];
  }

  std::string state_topic_;
  std::string delta_service_;
  std::string output_position_topic_;
  std::string odom_frame_id_;
  double state_timeout_s_{0.20};
  bool has_state_{false};
  std::array<float, 3> current_position_enu_{0.0f, 0.0f, 0.0f};
  float current_yaw_enu_{0.0f};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr output_pub_;
  rclcpp::Service<BodyPositionDelta>::SharedPtr delta_srv_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::PositionDeltaNode>());
  rclcpp::shutdown();
  return 0;
}
