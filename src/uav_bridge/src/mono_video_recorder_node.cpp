#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace uav_bridge
{

class MonoVideoRecorderNode : public rclcpp::Node
{
public:
  MonoVideoRecorderNode()
  : Node("mono_video_recorder_node")
  {
    this->declare_parameter<std::string>("image_topic", "/uav/camera/image_raw");
    this->declare_parameter<std::string>("output_path", "");
    this->declare_parameter<std::string>("fourcc", "MJPG");
    this->declare_parameter<double>("fps", 120.0);
    this->declare_parameter<int>("log_interval_frames", 300);

    image_topic_ = this->get_parameter("image_topic").as_string();
    output_path_ = resolveOutputPath(this->get_parameter("output_path").as_string());
    fourcc_ = this->get_parameter("fourcc").as_string();
    fps_ = this->get_parameter("fps").as_double();
    const auto log_interval_frames = this->get_parameter("log_interval_frames").as_int();
    log_interval_frames_ = static_cast<int>(std::max<int64_t>(0, log_interval_frames));

    if (fps_ <= 1.0e-6) {
      RCLCPP_WARN(this->get_logger(), "fps must be positive, falling back to 30.0");
      fps_ = 30.0;
    }

    fourcc_code_ = encodeFourcc(fourcc_);
    if (fourcc_code_ < 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "invalid fourcc '%s', falling back to MJPG",
        fourcc_.c_str());
      fourcc_ = "MJPG";
      fourcc_code_ = encodeFourcc(fourcc_);
    }

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MonoVideoRecorderNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "mono video recorder ready: topic=%s output=%s fps=%.2f fourcc=%s",
      image_topic_.c_str(),
      output_path_.c_str(),
      fps_,
      fourcc_.c_str());
  }

  ~MonoVideoRecorderNode() override
  {
    closeWriter("shutdown");
  }

private:
  static int encodeFourcc(const std::string & value)
  {
    if (value.size() != 4) {
      return -1;
    }

    return cv::VideoWriter::fourcc(value[0], value[1], value[2], value[3]);
  }

  static std::string expandUserPath(const std::string & path)
  {
    if (path.empty() || path.front() != '~') {
      return path;
    }

    const char * home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
      return path;
    }

    if (path.size() == 1) {
      return std::string(home);
    }

    if (path[1] == '/') {
      return std::string(home) + path.substr(1);
    }

    return path;
  }

  static std::string defaultOutputPath()
  {
    std::filesystem::path base_path;
    const char * home = std::getenv("HOME");
    if (home != nullptr && home[0] != '\0') {
      base_path = home;
    } else {
      base_path = std::filesystem::current_path();
    }

    std::time_t now = std::time(nullptr);
    std::tm local_time{};
    const std::tm * local_time_ptr = std::localtime(&now);
    if (local_time_ptr != nullptr) {
      local_time = *local_time_ptr;
    }

    std::ostringstream filename;
    filename << "mono_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".avi";
    return (base_path / "uav_recordings" / filename.str()).string();
  }

  static std::string resolveOutputPath(const std::string & path)
  {
    const std::string expanded = expandUserPath(path);
    if (!expanded.empty()) {
      return expanded;
    }
    return defaultOutputPath();
  }

  static bool validateImageBuffer(const sensor_msgs::msg::Image & msg, size_t min_step)
  {
    const size_t height = static_cast<size_t>(msg.height);
    return msg.step >= min_step && msg.data.size() >= static_cast<size_t>(msg.step) * height;
  }

  cv::Mat toBgrFrame(const sensor_msgs::msg::Image & msg)
  {
    if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
      return {};
    }

    const int rows = static_cast<int>(msg.height);
    const int cols = static_cast<int>(msg.width);
    auto * raw = const_cast<unsigned char *>(msg.data.data());

    if (msg.encoding == sensor_msgs::image_encodings::MONO8) {
      const size_t min_step = static_cast<size_t>(cols);
      if (!validateImageBuffer(msg, min_step)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "invalid MONO8 frame buffer: width=%u height=%u step=%u size=%zu",
          msg.width, msg.height, msg.step, msg.data.size());
        return {};
      }
      cv::Mat mono(rows, cols, CV_8UC1, raw, static_cast<size_t>(msg.step));
      cv::Mat bgr;
      cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }

    if (msg.encoding == sensor_msgs::image_encodings::BGR8) {
      const size_t min_step = static_cast<size_t>(cols) * 3U;
      if (!validateImageBuffer(msg, min_step)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "invalid BGR8 frame buffer: width=%u height=%u step=%u size=%zu",
          msg.width, msg.height, msg.step, msg.data.size());
        return {};
      }
      cv::Mat bgr(rows, cols, CV_8UC3, raw, static_cast<size_t>(msg.step));
      return bgr.clone();
    }

    if (msg.encoding == sensor_msgs::image_encodings::RGB8) {
      const size_t min_step = static_cast<size_t>(cols) * 3U;
      if (!validateImageBuffer(msg, min_step)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "invalid RGB8 frame buffer: width=%u height=%u step=%u size=%zu",
          msg.width, msg.height, msg.step, msg.data.size());
        return {};
      }
      cv::Mat rgb(rows, cols, CV_8UC3, raw, static_cast<size_t>(msg.step));
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return bgr;
    }

    if (msg.encoding == sensor_msgs::image_encodings::BGRA8) {
      const size_t min_step = static_cast<size_t>(cols) * 4U;
      if (!validateImageBuffer(msg, min_step)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "invalid BGRA8 frame buffer: width=%u height=%u step=%u size=%zu",
          msg.width, msg.height, msg.step, msg.data.size());
        return {};
      }
      cv::Mat bgra(rows, cols, CV_8UC4, raw, static_cast<size_t>(msg.step));
      cv::Mat bgr;
      cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
      return bgr;
    }

    if (msg.encoding == sensor_msgs::image_encodings::RGBA8) {
      const size_t min_step = static_cast<size_t>(cols) * 4U;
      if (!validateImageBuffer(msg, min_step)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "invalid RGBA8 frame buffer: width=%u height=%u step=%u size=%zu",
          msg.width, msg.height, msg.step, msg.data.size());
        return {};
      }
      cv::Mat rgba(rows, cols, CV_8UC4, raw, static_cast<size_t>(msg.step));
      cv::Mat bgr;
      cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
      return bgr;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "unsupported image encoding for video recording: %s",
      msg.encoding.c_str());
    return {};
  }

  bool ensureWriter(const cv::Mat & frame)
  {
    if (writer_.isOpened()) {
      return true;
    }

    try {
      const std::filesystem::path output_path(output_path_);
      if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "failed to create output directory for %s: %s",
        output_path_.c_str(),
        ex.what());
      return false;
    }

    if (!writer_.open(output_path_, fourcc_code_, fps_, frame.size(), true)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "failed to open video writer: path=%s fourcc=%s fps=%.2f size=%dx%d",
        output_path_.c_str(),
        fourcc_.c_str(),
        fps_,
        frame.cols,
        frame.rows);
      return false;
    }

    frame_width_ = frame.cols;
    frame_height_ = frame.rows;
    RCLCPP_INFO(
      this->get_logger(),
      "recording mono video to %s (%dx%d @ %.2f fps, fourcc=%s)",
      output_path_.c_str(),
      frame_width_,
      frame_height_,
      fps_,
      fourcc_.c_str());
    return true;
  }

  void closeWriter(const char * reason)
  {
    if (!writer_.isOpened()) {
      return;
    }

    writer_.release();
    RCLCPP_INFO(
      this->get_logger(),
      "stopped mono video recording (%s): %s, frames=%zu",
      reason,
      output_path_.c_str(),
      frames_written_);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame = toBgrFrame(*msg);
    if (frame.empty()) {
      return;
    }

    if (!ensureWriter(frame)) {
      return;
    }

    if (frame.cols != frame_width_ || frame.rows != frame_height_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "dropping frame with unexpected size %dx%d, recorder expects %dx%d",
        frame.cols, frame.rows, frame_width_, frame_height_);
      return;
    }

    writer_.write(frame);
    ++frames_written_;

    if (log_interval_frames_ > 0 && (frames_written_ % static_cast<size_t>(log_interval_frames_) == 0U)) {
      RCLCPP_INFO(
        this->get_logger(),
        "mono video recorder wrote %zu frames to %s",
        frames_written_,
        output_path_.c_str());
    }
  }

  std::string image_topic_;
  std::string output_path_;
  std::string fourcc_;
  double fps_{120.0};
  int fourcc_code_{cv::VideoWriter::fourcc('M', 'J', 'P', 'G')};
  int log_interval_frames_{300};
  int frame_width_{0};
  int frame_height_{0};
  size_t frames_written_{0};
  cv::VideoWriter writer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::MonoVideoRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
