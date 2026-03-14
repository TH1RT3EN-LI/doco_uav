#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace uav_bridge
{
class MonoCameraSourceNode : public rclcpp::Node
{
public:
  MonoCameraSourceNode()
  : Node("mono_camera_source_node")
  {
    this->declare_parameter<std::string>("device", "/dev/video0");
    this->declare_parameter<std::string>("fourcc", "");
    this->declare_parameter<std::string>("frame_id", "uav_camera_optical_frame");
    this->declare_parameter<std::string>("camera_name", "uav_mono_camera");
    this->declare_parameter<std::string>("image_topic", "/uav/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/uav/camera/camera_info");
    this->declare_parameter<std::string>("camera_info_url", "");
    this->declare_parameter<std::string>("distortion_model", "plumb_bob");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<double>("fx", -1.0);
    this->declare_parameter<double>("fy", -1.0);
    this->declare_parameter<double>("cx", -1.0);
    this->declare_parameter<double>("cy", -1.0);
    this->declare_parameter<double>("camera_hfov_rad", -1.0);
    this->declare_parameter<std::vector<double>>(
      "distortion_coefficients", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    device_ = this->get_parameter("device").as_string();
    fourcc_ = this->get_parameter("fourcc").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    camera_name_ = this->get_parameter("camera_name").as_string();
    camera_info_url_ = this->get_parameter("camera_info_url").as_string();
    distortion_model_ = this->get_parameter("distortion_model").as_string();
    requested_width_ = std::max(1, static_cast<int>(this->get_parameter("image_width").as_int()));
    requested_height_ = std::max(1, static_cast<int>(this->get_parameter("image_height").as_int()));
    requested_fps_ = std::max(1.0, this->get_parameter("fps").as_double());
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();
    camera_hfov_rad_ = this->get_parameter("camera_hfov_rad").as_double();
    distortion_coefficients_ = this->get_parameter("distortion_coefficients").as_double_array();

    const auto image_topic = this->get_parameter("image_topic").as_string();
    const auto camera_info_topic = this->get_parameter("camera_info_topic").as_string();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS());

    loadCameraCalibration();
    openCapture();

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / requested_fps_));
    capture_timer_ = this->create_wall_timer(
      timer_period, std::bind(&MonoCameraSourceNode::captureAndPublishFrame, this));

    RCLCPP_INFO(
      this->get_logger(),
      "mono_camera_source_node: device=%s image=%s camera_info=%s frame_id=%s calibration=%s",
      device_.c_str(), image_topic.c_str(), camera_info_topic.c_str(), frame_id_.c_str(),
      has_calibration_ ? calibration_path_.c_str() : "<generated>");
  }

private:
  static std::string resolveCalibrationPath(const std::string & camera_info_url)
  {
    constexpr const char kFilePrefix[] = "file://";
    if (camera_info_url.rfind(kFilePrefix, 0) == 0U) {
      return camera_info_url.substr(sizeof(kFilePrefix) - 1U);
    }
    return camera_info_url;
  }

  static std::vector<double> readYamlArray(
    const cv::FileNode & parent,
    const char * key)
  {
    std::vector<double> values;
    if (parent.empty()) {
      return values;
    }

    const cv::FileNode node = parent[key];
    if (node.empty()) {
      return values;
    }

    const cv::FileNode data_node = node["data"].empty() ? node : node["data"];
    if (data_node.type() != cv::FileNode::SEQ) {
      return values;
    }

    values.reserve(data_node.size());
    for (const auto & element : data_node) {
      values.push_back(static_cast<double>(element));
    }
    return values;
  }

  void loadCameraCalibration()
  {
    has_calibration_ = false;
    calibration_path_.clear();
    calibration_width_ = 0;
    calibration_height_ = 0;

    if (camera_info_url_.empty()) {
      return;
    }

    calibration_path_ = resolveCalibrationPath(camera_info_url_);
    if (calibration_path_.empty()) {
      throw std::runtime_error("camera_info_url resolved to an empty path");
    }

    cv::FileStorage storage(calibration_path_, cv::FileStorage::READ);
    if (!storage.isOpened()) {
      throw std::runtime_error("failed to open camera calibration file: " + calibration_path_);
    }

    sensor_msgs::msg::CameraInfo info;

    const auto image_width = static_cast<int>(storage["image_width"]);
    const auto image_height = static_cast<int>(storage["image_height"]);
    if (image_width <= 0 || image_height <= 0) {
      throw std::runtime_error("camera calibration is missing valid image_width/image_height");
    }

    const std::vector<double> camera_matrix = readYamlArray(storage.root(), "camera_matrix");
    const std::vector<double> distortion = readYamlArray(storage.root(), "distortion_coefficients");
    const std::vector<double> rectification = readYamlArray(storage.root(), "rectification_matrix");
    const std::vector<double> projection = readYamlArray(storage.root(), "projection_matrix");

    if (camera_matrix.size() != info.k.size()) {
      throw std::runtime_error("camera_matrix in calibration must contain exactly 9 values");
    }

    std::copy(camera_matrix.begin(), camera_matrix.end(), info.k.begin());
    info.width = static_cast<uint32_t>(image_width);
    info.height = static_cast<uint32_t>(image_height);
    info.distortion_model = distortion_model_;
    if (!storage["distortion_model"].empty()) {
      info.distortion_model = static_cast<std::string>(storage["distortion_model"]);
    }
    info.d = distortion.empty() ? distortion_coefficients_ : distortion;

    if (rectification.size() == info.r.size()) {
      std::copy(rectification.begin(), rectification.end(), info.r.begin());
    } else {
      info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    }

    if (projection.size() == info.p.size()) {
      std::copy(projection.begin(), projection.end(), info.p.begin());
    } else {
      info.p = {
        info.k[0], 0.0, info.k[2], 0.0,
        0.0, info.k[4], info.k[5], 0.0,
        0.0, 0.0, 1.0, 0.0};
    }

    calibration_template_ = info;
    calibration_width_ = image_width;
    calibration_height_ = image_height;
    has_calibration_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "loaded camera calibration %s (%dx%d, distortion_model=%s, d=%zu)",
      calibration_path_.c_str(),
      calibration_width_,
      calibration_height_,
      calibration_template_.distortion_model.c_str(),
      calibration_template_.d.size());
  }

  static int encodeFourcc(const std::string & value)
  {
    if (value.size() != 4) {
      return -1;
    }

    return cv::VideoWriter::fourcc(value[0], value[1], value[2], value[3]);
  }

  static std::string decodeFourcc(int value)
  {
    if (value <= 0) {
      return "auto";
    }

    std::string fourcc(4, ' ');
    fourcc[0] = static_cast<char>(value & 0xff);
    fourcc[1] = static_cast<char>((value >> 8) & 0xff);
    fourcc[2] = static_cast<char>((value >> 16) & 0xff);
    fourcc[3] = static_cast<char>((value >> 24) & 0xff);
    return fourcc;
  }

  static bool fillImageData(const cv::Mat & gray, sensor_msgs::msg::Image & msg)
  {
    if (gray.empty() || gray.type() != CV_8UC1) {
      return false;
    }

    msg.height = static_cast<uint32_t>(gray.rows);
    msg.width = static_cast<uint32_t>(gray.cols);
    msg.encoding = sensor_msgs::image_encodings::MONO8;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(gray.cols);
    msg.data.resize(static_cast<size_t>(gray.rows * gray.cols));

    if (gray.isContinuous()) {
      std::copy(gray.datastart, gray.dataend, msg.data.begin());
      return true;
    }

    for (int row = 0; row < gray.rows; ++row) {
      const auto * src = gray.ptr<uint8_t>(row);
      std::copy(src, src + gray.cols, msg.data.begin() + static_cast<size_t>(row * gray.cols));
    }
    return true;
  }

  cv::Mat convertToMono8(const cv::Mat & frame)
  {
    cv::Mat gray;

    try {
      if (frame.empty()) {
        return gray;
      }

      if (frame.type() == CV_8UC1) {
        return frame;
      }

      if (frame.type() == CV_16UC1) {
        frame.convertTo(gray, CV_8U, 1.0 / 256.0);
        return gray;
      }

      if (frame.type() == CV_8UC3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        return gray;
      }

      if (frame.type() == CV_8UC4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
        return gray;
      }

      if (frame.type() == CV_8UC2) {
        cv::cvtColor(frame, gray, cv::COLOR_YUV2GRAY_YUY2);
        return gray;
      }
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "mono camera frame conversion failed: %s", error.what());
      return cv::Mat();
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "unsupported mono camera frame type: %d", frame.type());
    return cv::Mat();
  }

  void openCapture()
  {
    capture_.release();

    if (device_.empty()) {
      throw std::runtime_error("mono camera device parameter must not be empty");
    }

    if (!capture_.open(device_, cv::CAP_V4L2)) {
      capture_.open(device_);
    }

    if (!capture_.isOpened()) {
      throw std::runtime_error("failed to open mono camera device: " + device_);
    }

    capture_.set(cv::CAP_PROP_CONVERT_RGB, 1.0);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(requested_width_));
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(requested_height_));
    capture_.set(cv::CAP_PROP_FPS, requested_fps_);

    if (!fourcc_.empty()) {
      const int encoded_fourcc = encodeFourcc(fourcc_);
      if (encoded_fourcc > 0) {
        capture_.set(cv::CAP_PROP_FOURCC, static_cast<double>(encoded_fourcc));
      } else {
        RCLCPP_WARN(
          this->get_logger(), "ignoring invalid fourcc '%s'; expected 4 characters", fourcc_.c_str());
      }
    }

    cv::Mat warmup_frame;
    if (capture_.read(warmup_frame) && !warmup_frame.empty()) {
      actual_width_ = warmup_frame.cols;
      actual_height_ = warmup_frame.rows;
    }

    if (actual_width_ <= 0) {
      actual_width_ = std::max(1, static_cast<int>(std::lround(capture_.get(cv::CAP_PROP_FRAME_WIDTH))));
    }
    if (actual_height_ <= 0) {
      actual_height_ = std::max(1, static_cast<int>(std::lround(capture_.get(cv::CAP_PROP_FRAME_HEIGHT))));
    }

    const auto actual_fps = capture_.get(cv::CAP_PROP_FPS);
    const auto actual_fourcc = static_cast<int>(std::lround(capture_.get(cv::CAP_PROP_FOURCC)));

    RCLCPP_INFO(
      this->get_logger(),
      "opened mono camera %s (%dx%d @ %.2f fps, fourcc=%s, camera_name=%s)",
      device_.c_str(),
      actual_width_,
      actual_height_,
      actual_fps,
      decodeFourcc(actual_fourcc).c_str(),
      camera_name_.c_str());
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(const sensor_msgs::msg::Image & image_msg) const
  {
    if (has_calibration_) {
      sensor_msgs::msg::CameraInfo info = calibration_template_;
      info.header = image_msg.header;
      info.width = image_msg.width;
      info.height = image_msg.height;

      if (calibration_width_ > 0 && calibration_height_ > 0 &&
        (static_cast<int>(image_msg.width) != calibration_width_ ||
        static_cast<int>(image_msg.height) != calibration_height_))
      {
        const double scale_x =
          static_cast<double>(image_msg.width) / static_cast<double>(calibration_width_);
        const double scale_y =
          static_cast<double>(image_msg.height) / static_cast<double>(calibration_height_);

        info.k[0] *= scale_x;
        info.k[2] *= scale_x;
        info.k[4] *= scale_y;
        info.k[5] *= scale_y;

        info.p[0] *= scale_x;
        info.p[2] *= scale_x;
        info.p[3] *= scale_x;
        info.p[5] *= scale_y;
        info.p[6] *= scale_y;
        info.p[7] *= scale_y;
      }

      return info;
    }

    sensor_msgs::msg::CameraInfo info;
    info.header = image_msg.header;
    info.width = image_msg.width;
    info.height = image_msg.height;
    info.distortion_model = distortion_model_;
    info.d = distortion_coefficients_;

    const double width = std::max(1.0, static_cast<double>(image_msg.width));
    const double height = std::max(1.0, static_cast<double>(image_msg.height));
    const double fallback_fx = camera_hfov_rad_ > 1.0e-6 ? (width / (2.0 * std::tan(camera_hfov_rad_ / 2.0))) : (width / 2.0);
    const double fallback_fy = fallback_fx;
    const double fallback_cx = width / 2.0;
    const double fallback_cy = height / 2.0;
    const double fx = fx_ > 0.0 ? fx_ : fallback_fx;
    const double fy = fy_ > 0.0 ? fy_ : fallback_fy;
    const double cx = cx_ > 0.0 ? cx_ : fallback_cx;
    const double cy = cy_ > 0.0 ? cy_ : fallback_cy;

    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void captureAndPublishFrame()
  {
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "failed to read frame from mono camera device %s", device_.c_str());
      return;
    }

    cv::Mat gray = convertToMono8(frame);
    if (gray.empty()) {
      return;
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = this->now();
    image_msg.header.frame_id = frame_id_;
    if (!fillImageData(gray, image_msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "failed to serialize mono camera frame from device %s", device_.c_str());
      return;
    }

    auto camera_info_msg = makeCameraInfo(image_msg);
    image_pub_->publish(image_msg);
    camera_info_pub_->publish(camera_info_msg);
  }

  cv::VideoCapture capture_;
  std::string device_;
  std::string fourcc_;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string calibration_path_;
  std::string distortion_model_;
  std::vector<double> distortion_coefficients_;
  sensor_msgs::msg::CameraInfo calibration_template_;
  int requested_width_{640};
  int requested_height_{480};
  int actual_width_{0};
  int actual_height_{0};
  int calibration_width_{0};
  int calibration_height_{0};
  double requested_fps_{30.0};
  double fx_{-1.0};
  double fy_{-1.0};
  double cx_{-1.0};
  double cy_{-1.0};
  double camera_hfov_rad_{-1.0};
  bool has_calibration_{false};

  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};
}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::MonoCameraSourceNode>());
  rclcpp::shutdown();
  return 0;
}
