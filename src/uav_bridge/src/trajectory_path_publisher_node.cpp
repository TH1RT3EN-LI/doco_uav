#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav_bridge
{

class TrajectoryPathPublisherNode : public rclcpp::Node
{
public:
  TrajectoryPathPublisherNode()
  : Node("trajectory_path_publisher_node")
  {
    this->declare_parameter<std::string>("input_odom_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("output_path_topic", "/uav/state/trajectory");
    this->declare_parameter<std::string>("path_frame_id", "");
    this->declare_parameter<int>("max_samples", 5000);
    this->declare_parameter<double>("min_sample_distance_m", 0.02);
    this->declare_parameter<double>("min_sample_period_s", 0.10);
    this->declare_parameter<bool>("log_resets", true);

    input_odom_topic_ = this->get_parameter("input_odom_topic").as_string();
    output_path_topic_ = this->get_parameter("output_path_topic").as_string();
    path_frame_id_ = this->get_parameter("path_frame_id").as_string();
    max_samples_ = std::max(1, static_cast<int>(this->get_parameter("max_samples").as_int()));
    min_sample_distance_m_ = std::max(0.0, this->get_parameter("min_sample_distance_m").as_double());
    min_sample_period_s_ = std::max(0.0, this->get_parameter("min_sample_period_s").as_double());
    log_resets_ = this->get_parameter("log_resets").as_bool();

    const auto odom_qos = rclcpp::QoS(50).reliable();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, odom_qos,
      std::bind(&TrajectoryPathPublisherNode::handleOdometry, this, std::placeholders::_1));

    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_path_topic_, path_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "trajectory_path_publisher_node: input=%s output=%s max_samples=%d min_dist=%.3f min_dt=%.3f",
      input_odom_topic_.c_str(),
      output_path_topic_.c_str(),
      max_samples_,
      min_sample_distance_m_,
      min_sample_period_s_);
  }

private:
  static double squaredDistance(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs)
  {
    const double dx = lhs.pose.position.x - rhs.pose.position.x;
    const double dy = lhs.pose.position.y - rhs.pose.position.y;
    const double dz = lhs.pose.position.z - rhs.pose.position.z;
    return dx * dx + dy * dy + dz * dz;
  }

  void resetPath(const std::string & frame_id, const rclcpp::Time & stamp)
  {
    path_msg_.header.frame_id = frame_id;
    path_msg_.header.stamp = stamp;
    path_msg_.poses.clear();
    last_sample_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  void trimPath()
  {
    if (static_cast<int>(path_msg_.poses.size()) <= max_samples_) {
      return;
    }

    const auto remove_count = path_msg_.poses.size() - static_cast<std::size_t>(max_samples_);
    path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + remove_count);
  }

  void publishPath()
  {
    path_pub_->publish(path_msg_);
  }

  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string frame_id = path_frame_id_.empty() ? msg->header.frame_id : path_frame_id_;
    if (frame_id.empty()) {
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = frame_id;
    pose.pose = msg->pose.pose;

    if (path_msg_.header.frame_id.empty()) {
      resetPath(frame_id, stamp);
    } else if (path_msg_.header.frame_id != frame_id) {
      if (log_resets_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Trajectory path frame changed from %s to %s, clearing history.",
          path_msg_.header.frame_id.c_str(),
          frame_id.c_str());
      }
      resetPath(frame_id, stamp);
    } else if (!path_msg_.poses.empty() && stamp < last_sample_stamp_) {
      if (log_resets_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Trajectory path received out-of-order time, clearing history.");
      }
      resetPath(frame_id, stamp);
    }

    if (path_msg_.poses.empty()) {
      path_msg_.poses.push_back(pose);
      path_msg_.header.stamp = stamp;
      last_sample_stamp_ = stamp;
      publishPath();
      return;
    }

    const auto & last_pose = path_msg_.poses.back();
    const double dt_sec = (stamp - last_sample_stamp_).seconds();
    const double dist_sq = squaredDistance(last_pose, pose);
    const bool distance_ok = dist_sq >= (min_sample_distance_m_ * min_sample_distance_m_);
    const bool time_ok = dt_sec >= min_sample_period_s_;

    path_msg_.header.stamp = stamp;
    if (distance_ok || time_ok) {
      path_msg_.poses.push_back(pose);
      trimPath();
      last_sample_stamp_ = stamp;
    } else {
      path_msg_.poses.back() = pose;
    }

    publishPath();
  }

  std::string input_odom_topic_;
  std::string output_path_topic_;
  std::string path_frame_id_;
  int max_samples_{5000};
  double min_sample_distance_m_{0.02};
  double min_sample_period_s_{0.10};
  bool log_resets_{true};
  nav_msgs::msg::Path path_msg_;
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::TrajectoryPathPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
