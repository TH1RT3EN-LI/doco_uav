#include <px4_msgs/msg/input_rc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace uav_bridge
{

class RcSafetyMuxNode : public rclcpp::Node
{
  using InputRc = px4_msgs::msg::InputRc;
  using String = std_msgs::msg::String;
  using Trigger = std_srvs::srv::Trigger;

  enum class Ch5Position : uint8_t
  {
    Unknown = 0,
    Low = 1,
    Middle = 2,
    High = 3,
  };

  enum class EffectiveAction : uint8_t
  {
    Unknown = 0,
    None = 1,
    Abort = 2,
    KillActive = 3,
  };

  struct PendingServiceCall
  {
    EffectiveAction action{EffectiveAction::Unknown};
    std::string service_name;
  };

public:
  RcSafetyMuxNode()
  : Node("rc_safety_mux_node")
  {
    this->declare_parameter<std::string>("input_rc_topic", "/fmu/out/input_rc");
    this->declare_parameter<std::string>("state_topic", "/uav/safety/rc_safety_mux/state");
    this->declare_parameter<std::string>("ch5_high_service", "/uav/control/command/abort");
    this->declare_parameter<std::string>("kill_active_service", "");
    this->declare_parameter<int>("ch5_channel_index", 4);
    this->declare_parameter<int>("ch7_channel_index", 6);
    this->declare_parameter<int>("channel_min_valid_us", 800);
    this->declare_parameter<int>("channel_max_valid_us", 2200);
    this->declare_parameter<int>("ch5_low_threshold_us", 1300);
    this->declare_parameter<int>("ch5_high_threshold_us", 1700);
    this->declare_parameter<int>("ch7_active_threshold_us", 1700);
    this->declare_parameter<int>("switch_hysteresis_us", 80);
    this->declare_parameter<double>("debounce_s", 0.20);
    this->declare_parameter<double>("tick_rate_hz", 10.0);
    this->declare_parameter<double>("service_retry_s", 1.0);
    this->declare_parameter<double>("status_timeout_s", 1.0);

    input_rc_topic_ = this->get_parameter("input_rc_topic").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    ch5_high_service_ = this->get_parameter("ch5_high_service").as_string();
    kill_active_service_ = this->get_parameter("kill_active_service").as_string();
    ch5_channel_index_ = this->get_parameter("ch5_channel_index").as_int();
    ch7_channel_index_ = this->get_parameter("ch7_channel_index").as_int();
    channel_min_valid_us_ = this->get_parameter("channel_min_valid_us").as_int();
    channel_max_valid_us_ = this->get_parameter("channel_max_valid_us").as_int();
    ch5_low_threshold_us_ = this->get_parameter("ch5_low_threshold_us").as_int();
    ch5_high_threshold_us_ = this->get_parameter("ch5_high_threshold_us").as_int();
    ch7_active_threshold_us_ = this->get_parameter("ch7_active_threshold_us").as_int();
    switch_hysteresis_us_ = this->get_parameter("switch_hysteresis_us").as_int();
    debounce_s_ = std::max(0.0, this->get_parameter("debounce_s").as_double());
    tick_rate_hz_ = std::max(1.0, this->get_parameter("tick_rate_hz").as_double());
    service_retry_s_ = std::max(0.1, this->get_parameter("service_retry_s").as_double());
    status_timeout_s_ = std::max(0.1, this->get_parameter("status_timeout_s").as_double());

    if (ch5_channel_index_ < 0 || ch5_channel_index_ >= static_cast<int>(InputRc::RC_INPUT_MAX_CHANNELS)) {
      throw std::runtime_error("ch5_channel_index is out of range");
    }
    if (ch7_channel_index_ < 0 || ch7_channel_index_ >= static_cast<int>(InputRc::RC_INPUT_MAX_CHANNELS)) {
      throw std::runtime_error("ch7_channel_index is out of range");
    }

    status_pub_ = this->create_publisher<String>(state_topic_, rclcpp::QoS(1).transient_local());
    ch5_high_client_ = createOptionalClient(ch5_high_service_);
    kill_active_client_ = createOptionalClient(kill_active_service_);

    input_rc_sub_ = this->create_subscription<InputRc>(
      input_rc_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RcSafetyMuxNode::onInputRc, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tick_rate_hz_));
    timer_ = this->create_wall_timer(timer_period, std::bind(&RcSafetyMuxNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "rc_safety_mux_node: rc=%s ch5(index=%d) ch7(index=%d) action(high=%s kill=%s) state=%s",
      input_rc_topic_.c_str(),
      ch5_channel_index_ + 1,
      ch7_channel_index_ + 1,
      ch5_high_service_.empty() ? "<disabled>" : ch5_high_service_.c_str(),
      kill_active_service_.empty() ? "<disabled>" : kill_active_service_.c_str(),
      state_topic_.c_str());
  }

private:
  rclcpp::Client<Trigger>::SharedPtr createOptionalClient(const std::string & service_name)
  {
    if (service_name.empty()) {
      return nullptr;
    }
    return this->create_client<Trigger>(service_name);
  }

  void onInputRc(const InputRc::SharedPtr msg)
  {
    last_input_receive_time_ = this->now();

    if (msg->rc_lost || msg->rc_failsafe) {
      rc_valid_ = false;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "ignoring RC input while rc_lost=%s rc_failsafe=%s",
        msg->rc_lost ? "true" : "false",
        msg->rc_failsafe ? "true" : "false");
      publishStateIfChanged();
      return;
    }

    const int required_channels = std::max(ch5_channel_index_, ch7_channel_index_) + 1;
    if (msg->channel_count < required_channels) {
      rc_valid_ = false;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "ignoring RC input with only %u channels; need at least %d for CH5/CH7",
        msg->channel_count, required_channels);
      publishStateIfChanged();
      return;
    }

    raw_ch5_pwm_us_ = static_cast<int>(msg->values[static_cast<std::size_t>(ch5_channel_index_)]);
    raw_ch7_pwm_us_ = static_cast<int>(msg->values[static_cast<std::size_t>(ch7_channel_index_)]);

    if (!isValidPwm(raw_ch5_pwm_us_) || !isValidPwm(raw_ch7_pwm_us_)) {
      rc_valid_ = false;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "ignoring RC input with invalid PWM values ch5=%d ch7=%d",
        raw_ch5_pwm_us_, raw_ch7_pwm_us_);
      publishStateIfChanged();
      return;
    }

    rc_valid_ = true;
    updateCh5StableState(decodeCh5(raw_ch5_pwm_us_));
    updateCh7StableState(decodeCh7(raw_ch7_pwm_us_));
    publishStateIfChanged();
  }

  bool isValidPwm(int pwm_us) const
  {
    return pwm_us >= channel_min_valid_us_ && pwm_us <= channel_max_valid_us_;
  }

  Ch5Position decodeCh5(int pwm_us) const
  {
    const int hysteresis = std::max(0, switch_hysteresis_us_);
    const Ch5Position reference =
      stable_ch5_position_ != Ch5Position::Unknown ? stable_ch5_position_ : candidate_ch5_position_;

    switch (reference) {
      case Ch5Position::Low:
        if (pwm_us < ch5_low_threshold_us_ + hysteresis) {
          return Ch5Position::Low;
        }
        break;
      case Ch5Position::High:
        if (pwm_us > ch5_high_threshold_us_ - hysteresis) {
          return Ch5Position::High;
        }
        break;
      case Ch5Position::Middle:
        if (pwm_us < ch5_low_threshold_us_ - hysteresis) {
          return Ch5Position::Low;
        }
        if (pwm_us > ch5_high_threshold_us_ + hysteresis) {
          return Ch5Position::High;
        }
        return Ch5Position::Middle;
      case Ch5Position::Unknown:
        break;
    }

    if (pwm_us < ch5_low_threshold_us_) {
      return Ch5Position::Low;
    }
    if (pwm_us > ch5_high_threshold_us_) {
      return Ch5Position::High;
    }
    return Ch5Position::Middle;
  }

  bool decodeCh7(int pwm_us) const
  {
    const int hysteresis = std::max(0, switch_hysteresis_us_);
    if (stable_ch7_active_) {
      return pwm_us > ch7_active_threshold_us_ - hysteresis;
    }
    return pwm_us > ch7_active_threshold_us_ + hysteresis;
  }

  void updateCh5StableState(Ch5Position decoded)
  {
    const rclcpp::Time now = this->now();
    if (decoded != candidate_ch5_position_) {
      candidate_ch5_position_ = decoded;
      candidate_ch5_since_ = now;
      return;
    }

    if (
      decoded != stable_ch5_position_ &&
      (now - candidate_ch5_since_).seconds() >= debounce_s_)
    {
      stable_ch5_position_ = decoded;
      RCLCPP_INFO(
        this->get_logger(),
        "CH5 stable -> %s (%d us)",
        ch5PositionName(stable_ch5_position_),
        raw_ch5_pwm_us_);
    }
  }

  void updateCh7StableState(bool decoded_active)
  {
    const rclcpp::Time now = this->now();
    if (decoded_active != candidate_ch7_active_) {
      candidate_ch7_active_ = decoded_active;
      candidate_ch7_since_ = now;
      return;
    }

    if (
      decoded_active != stable_ch7_active_ &&
      (now - candidate_ch7_since_).seconds() >= debounce_s_)
    {
      stable_ch7_active_ = decoded_active;
      if (stable_ch7_active_) {
        RCLCPP_WARN(
          this->get_logger(),
          "CH7 kill switch detected active at %d us; ROS safety mux will suppress CH5 actions while PX4 handles kill",
          raw_ch7_pwm_us_);
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "CH7 kill switch released at %d us",
          raw_ch7_pwm_us_);
      }
    }
  }

  void onTimer()
  {
    publishStateIfChanged();

    if (service_request_in_flight_) {
      return;
    }

    const EffectiveAction desired_action = computeDesiredAction();
    if (desired_action == latched_action_) {
      return;
    }

    const rclcpp::Time now = this->now();
    if ((now - last_service_attempt_time_).seconds() < service_retry_s_) {
      return;
    }

    const auto call = buildPendingServiceCall(desired_action);
    if (!call.has_value()) {
      latched_action_ = desired_action;
      publishStateIfChanged();
      return;
    }

    auto client = clientForAction(call->action);
    if (!client) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "no service client configured for desired action %s",
        effectiveActionName(call->action));
      return;
    }

    if (!client->service_is_ready()) {
      last_service_attempt_time_ = now;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "waiting for service %s for desired action %s",
        call->service_name.c_str(),
        effectiveActionName(call->action));
      return;
    }

    last_service_attempt_time_ = now;
    service_request_in_flight_ = true;
    auto request = std::make_shared<Trigger::Request>();
    client->async_send_request(
      request,
      [this, action = call->action, service_name = call->service_name](
        rclcpp::Client<Trigger>::SharedFuture future)
      {
        service_request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (response->success) {
            latched_action_ = action;
            RCLCPP_INFO(
              this->get_logger(),
              "triggered RC safety action %s via %s: %s",
              effectiveActionName(action),
              service_name.c_str(),
              response->message.c_str());
          } else {
            RCLCPP_WARN(
              this->get_logger(),
              "service %s rejected RC safety action %s: %s",
              service_name.c_str(),
              effectiveActionName(action),
              response->message.c_str());
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            this->get_logger(),
            "service %s failed for RC safety action %s: %s",
            service_name.c_str(),
            effectiveActionName(action),
            ex.what());
        }
        publishStateIfChanged();
      });
  }

  EffectiveAction computeDesiredAction() const
  {
    if (!rc_valid_) {
      return latched_action_;
    }
    if (stable_ch7_active_) {
      return EffectiveAction::KillActive;
    }

    if (stable_ch5_position_ == Ch5Position::High) {
      return EffectiveAction::Abort;
    }

    return EffectiveAction::None;
  }

  std::optional<PendingServiceCall> buildPendingServiceCall(EffectiveAction action) const
  {
    switch (action) {
      case EffectiveAction::Abort:
        if (ch5_high_service_.empty()) {
          return std::nullopt;
        }
        return PendingServiceCall{action, ch5_high_service_};
      case EffectiveAction::KillActive:
        if (kill_active_service_.empty()) {
          return std::nullopt;
        }
        return PendingServiceCall{action, kill_active_service_};
      case EffectiveAction::None:
      case EffectiveAction::Unknown:
        return std::nullopt;
    }
    return std::nullopt;
  }

  rclcpp::Client<Trigger>::SharedPtr clientForAction(EffectiveAction action) const
  {
    switch (action) {
      case EffectiveAction::Abort:
        return ch5_high_client_;
      case EffectiveAction::KillActive:
        return kill_active_client_;
      case EffectiveAction::None:
      case EffectiveAction::Unknown:
        break;
    }
    return nullptr;
  }

  void publishStateIfChanged()
  {
    String msg;
    msg.data = buildStateString();
    if (msg.data == last_state_text_) {
      return;
    }
    last_state_text_ = msg.data;
    status_pub_->publish(msg);
  }

  std::string buildStateString() const
  {
    const rclcpp::Time now = this->now();
    const bool rc_fresh =
      last_input_receive_time_.nanoseconds() != 0 &&
      (now - last_input_receive_time_).seconds() <= status_timeout_s_;

    std::ostringstream stream;
    stream
      << "rc_valid=" << (rc_valid_ ? "true" : "false")
      << " rc_fresh=" << (rc_fresh ? "true" : "false")
      << " ch5=" << ch5PositionName(stable_ch5_position_) << "(" << raw_ch5_pwm_us_ << "us)"
      << " ch7=" << (stable_ch7_active_ ? "ACTIVE" : "inactive") << "(" << raw_ch7_pwm_us_ << "us)"
      << " desired_action=" << effectiveActionName(computeDesiredAction())
      << " latched_action=" << effectiveActionName(latched_action_)
      << " in_flight=" << (service_request_in_flight_ ? "true" : "false");
    return stream.str();
  }

  static const char * ch5PositionName(Ch5Position position)
  {
    switch (position) {
      case Ch5Position::Unknown:
        return "UNKNOWN";
      case Ch5Position::Low:
        return "LOW";
      case Ch5Position::Middle:
        return "MIDDLE";
      case Ch5Position::High:
        return "HIGH";
    }
    return "UNKNOWN";
  }

  static const char * effectiveActionName(EffectiveAction action)
  {
    switch (action) {
      case EffectiveAction::Unknown:
        return "UNKNOWN";
      case EffectiveAction::None:
        return "NONE";
      case EffectiveAction::Abort:
        return "ABORT";
      case EffectiveAction::KillActive:
        return "KILL_ACTIVE";
    }
    return "UNKNOWN";
  }

  std::string input_rc_topic_;
  std::string state_topic_;
  std::string ch5_high_service_;
  std::string kill_active_service_;

  int ch5_channel_index_{4};
  int ch7_channel_index_{6};
  int channel_min_valid_us_{800};
  int channel_max_valid_us_{2200};
  int ch5_low_threshold_us_{1300};
  int ch5_high_threshold_us_{1700};
  int ch7_active_threshold_us_{1700};
  int switch_hysteresis_us_{80};

  double debounce_s_{0.20};
  double tick_rate_hz_{10.0};
  double service_retry_s_{1.0};
  double status_timeout_s_{1.0};

  rclcpp::Subscription<InputRc>::SharedPtr input_rc_sub_;
  rclcpp::Publisher<String>::SharedPtr status_pub_;
  rclcpp::Client<Trigger>::SharedPtr ch5_high_client_;
  rclcpp::Client<Trigger>::SharedPtr kill_active_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool rc_valid_{false};
  int raw_ch5_pwm_us_{0};
  int raw_ch7_pwm_us_{0};
  rclcpp::Time last_input_receive_time_{0, 0, RCL_ROS_TIME};

  Ch5Position candidate_ch5_position_{Ch5Position::Unknown};
  Ch5Position stable_ch5_position_{Ch5Position::Unknown};
  rclcpp::Time candidate_ch5_since_{0, 0, RCL_ROS_TIME};

  bool candidate_ch7_active_{false};
  bool stable_ch7_active_{false};
  rclcpp::Time candidate_ch7_since_{0, 0, RCL_ROS_TIME};

  EffectiveAction latched_action_{EffectiveAction::Unknown};
  bool service_request_in_flight_{false};
  rclcpp::Time last_service_attempt_time_{0, 0, RCL_ROS_TIME};
  std::string last_state_text_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::RcSafetyMuxNode>());
  rclcpp::shutdown();
  return 0;
}
