#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace uav_bridge
{

class HeightMeasurementBridgeNode : public rclcpp::Node
{
public:
  HeightMeasurementBridgeNode()
  : Node("height_measurement_bridge_node")
  {
    this->declare_parameter<std::string>("distance_sensor_topic", "/uav/fmu/out/distance_sensor");
    this->declare_parameter<std::string>("height_measurement_topic", "/uav/sensors/downward_range");
    this->declare_parameter<std::string>("frame_id", "uav_optical_flow_range_frame");

    distance_sensor_topic_ = this->get_parameter("distance_sensor_topic").as_string();
    height_measurement_topic_ = this->get_parameter("height_measurement_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    this->get_parameter_or("use_sim_time", use_sim_time_, false);

    publisher_ = this->create_publisher<sensor_msgs::msg::Range>(
      height_measurement_topic_, rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
      distance_sensor_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::DistanceSensor::SharedPtr msg)
      {
        onDistanceSensor(*msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "height_measurement_bridge_node: distance_sensor=%s -> stamped_range=%s frame_id=%s "
      "use_sim_time=%s",
      distance_sensor_topic_.c_str(),
      height_measurement_topic_.c_str(),
      frame_id_.c_str(),
      use_sim_time_ ? "true" : "false");
  }

private:
  static uint8_t radiationTypeForDistanceSensor(uint8_t sensor_type)
  {
    if (sensor_type == px4_msgs::msg::DistanceSensor::MAV_DISTANCE_SENSOR_ULTRASOUND) {
      return sensor_msgs::msg::Range::ULTRASOUND;
    }
    return sensor_msgs::msg::Range::INFRARED;
  }

  void onDistanceSensor(const px4_msgs::msg::DistanceSensor & msg)
  {
    if (
      !std::isfinite(msg.current_distance) || !std::isfinite(msg.min_distance) ||
      !std::isfinite(msg.max_distance))
    {
      return;
    }

    if (!use_sim_time_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "height_measurement_bridge_node requires use_sim_time=true to publish sample-time range");
      return;
    }

    if (msg.timestamp == 0U) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "ignoring distance sensor message without a usable PX4 sample timestamp");
      return;
    }

    const int64_t stamp_ns = static_cast<int64_t>(msg.timestamp) * 1000LL;

    sensor_msgs::msg::Range out;
    out.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    out.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    out.header.frame_id = frame_id_;
    out.radiation_type = radiationTypeForDistanceSensor(msg.type);
    out.field_of_view = std::max(
      std::max(0.0f, msg.h_fov),
      std::max(0.0f, msg.v_fov));
    out.min_range = msg.min_distance;
    out.max_range = msg.max_distance;
    out.range = msg.current_distance;
    publisher_->publish(out);
  }

  std::string distance_sensor_topic_;
  std::string height_measurement_topic_;
  std::string frame_id_;
  bool use_sim_time_{false};
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr subscription_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::HeightMeasurementBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
