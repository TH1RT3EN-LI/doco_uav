#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uav_bridge/sim_vio_odometry_logic.hpp"

namespace uav_bridge
{

namespace
{

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

SimVioOdometrySample toLogicSample(const nav_msgs::msg::Odometry & msg)
{
  SimVioOdometrySample sample;
  sample.stamp_ns = stampToNanoseconds(msg.header.stamp);
  sample.frame_id = msg.header.frame_id;
  sample.child_frame_id = msg.child_frame_id;
  sample.position = {
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z};
  sample.orientation_xyzw = {
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w};
  sample.linear_velocity = {
    msg.twist.twist.linear.x,
    msg.twist.twist.linear.y,
    msg.twist.twist.linear.z};
  sample.angular_velocity = {
    msg.twist.twist.angular.x,
    msg.twist.twist.angular.y,
    msg.twist.twist.angular.z};
  std::copy(msg.pose.covariance.begin(), msg.pose.covariance.end(), sample.pose_covariance.begin());
  std::copy(msg.twist.covariance.begin(), msg.twist.covariance.end(), sample.twist_covariance.begin());
  return sample;
}

nav_msgs::msg::Odometry toRosOdometry(const SimVioOdometrySample & sample)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = static_cast<int32_t>(sample.stamp_ns / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(sample.stamp_ns % 1000000000LL);
  msg.header.frame_id = sample.frame_id;
  msg.child_frame_id = sample.child_frame_id;
  msg.pose.pose.position.x = sample.position[0];
  msg.pose.pose.position.y = sample.position[1];
  msg.pose.pose.position.z = sample.position[2];
  msg.pose.pose.orientation.x = sample.orientation_xyzw[0];
  msg.pose.pose.orientation.y = sample.orientation_xyzw[1];
  msg.pose.pose.orientation.z = sample.orientation_xyzw[2];
  msg.pose.pose.orientation.w = sample.orientation_xyzw[3];
  msg.twist.twist.linear.x = sample.linear_velocity[0];
  msg.twist.twist.linear.y = sample.linear_velocity[1];
  msg.twist.twist.linear.z = sample.linear_velocity[2];
  msg.twist.twist.angular.x = sample.angular_velocity[0];
  msg.twist.twist.angular.y = sample.angular_velocity[1];
  msg.twist.twist.angular.z = sample.angular_velocity[2];
  std::copy(sample.pose_covariance.begin(), sample.pose_covariance.end(), msg.pose.covariance.begin());
  std::copy(sample.twist_covariance.begin(), sample.twist_covariance.end(), msg.twist.covariance.begin());
  return msg;
}

}  // namespace

class SimVioOdometryNode : public rclcpp::Node
{
public:
  explicit SimVioOdometryNode(const rclcpp::NodeOptions & options)
  : Node("sim_vio_odometry_node", options)
  {
    const auto input_odometry_topic = declare_parameter<std::string>(
      "input_odometry_topic",
      "/uav/sim/ground_truth/odom");
    const auto output_odometry_topic = declare_parameter<std::string>(
      "output_odometry_topic",
      "odomimu");
    const auto publish_rate_hz = std::max(1.0, declare_parameter<double>("output_rate_hz", 30.0));
    const auto log_debug = declare_parameter<bool>("log_debug", false);

    SimVioOdometryConfig config;
    config.output_delay_s = declare_parameter<double>("output_delay_s", 0.04);
    config.warmup_duration_s = declare_parameter<double>("warmup_duration_s", 0.5);
    config.position_noise_stddev_xy_m = declare_parameter<double>(
      "position_noise_stddev_xy_m",
      0.03);
    config.position_noise_stddev_z_m = declare_parameter<double>(
      "position_noise_stddev_z_m",
      0.02);
    config.position_error_limit_xy_m = declare_parameter<double>(
      "position_error_limit_xy_m",
      std::numeric_limits<double>::infinity());
    config.position_error_limit_z_m = declare_parameter<double>(
      "position_error_limit_z_m",
      std::numeric_limits<double>::infinity());
    config.roll_pitch_noise_stddev_rad = declare_parameter<double>(
      "roll_pitch_noise_stddev_rad",
      0.003);
    config.yaw_noise_stddev_rad = declare_parameter<double>(
      "yaw_noise_stddev_rad",
      0.01);
    config.linear_velocity_noise_stddev_mps = declare_parameter<double>(
      "linear_velocity_noise_stddev_mps",
      0.05);
    config.position_random_walk_xy_m_per_sqrt_s = declare_parameter<double>(
      "position_random_walk_xy_m_per_sqrt_s",
      0.002);
    config.position_random_walk_z_m_per_sqrt_s = declare_parameter<double>(
      "position_random_walk_z_m_per_sqrt_s",
      0.001);
    config.yaw_random_walk_rad_per_sqrt_s = declare_parameter<double>(
      "yaw_random_walk_rad_per_sqrt_s",
      0.0005);
    config.dropout_interval_mean_s = declare_parameter<double>(
      "dropout_interval_mean_s",
      25.0);
    config.dropout_duration_s = declare_parameter<double>(
      "dropout_duration_s",
      0.3);
    config.random_seed = static_cast<uint32_t>(declare_parameter<int64_t>("random_seed", 1));
    config.output_frame_id = declare_parameter<std::string>("output_frame_id", "global");
    config.output_child_frame_id = declare_parameter<std::string>("output_child_frame_id", "imu");
    logic_ = std::make_unique<SimVioOdometryLogic>(config);
    logic_->reset(get_clock()->now().nanoseconds());

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odometry_topic,
      rclcpp::SensorDataQoS());
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odometry_topic,
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto sample = toLogicSample(*msg);
        if (latest_input_stamp_ns_.has_value() && sample.stamp_ns <= *latest_input_stamp_ns_) {
          return;
        }

        latest_input_stamp_ns_ = sample.stamp_ns;
        logic_->observeGroundTruth(sample);
      });

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      [this, log_debug]() {
        if (!latest_input_stamp_ns_.has_value()) {
          return;
        }

        const auto sample = logic_->step(*latest_input_stamp_ns_);
        if (!sample.has_value()) {
          return;
        }

        auto msg = toRosOdometry(*sample);
        odometry_pub_->publish(msg);

        if (log_debug) {
          RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "published fake OpenVINS odom stamp=%lld frame=%s child=%s",
            static_cast<long long>(sample->stamp_ns),
            sample->frame_id.c_str(),
            sample->child_frame_id.c_str());
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "sim_vio_odometry_node: %s -> %s @ %.1f Hz",
      input_odometry_topic.c_str(),
      output_odometry_topic.c_str(),
      publish_rate_hz);
  }

private:
  std::unique_ptr<SimVioOdometryLogic> logic_;
  std::optional<int64_t> latest_input_stamp_ns_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::SimVioOdometryNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
