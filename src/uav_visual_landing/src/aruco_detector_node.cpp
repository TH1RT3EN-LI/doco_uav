#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <uav_visual_landing/msg/april_tag_detection.hpp>
#include <uav_visual_landing/msg/landing_controller_state.hpp>
#include <uav_visual_landing/msg/target_observation.hpp>
#include <uav_visual_landing/visual_landing_logic.hpp>

namespace uav_visual_landing
{

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector_node")
  {
    this->declare_parameter<std::string>("image_topic", "/uav/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/uav/camera/camera_info");
    this->declare_parameter<std::string>(
      "target_observation_topic",
      "/uav/visual_landing/target_observation");
    this->declare_parameter<std::string>(
      "tag_detection_topic",
      "/uav/visual_landing/apriltag_detection");
    this->declare_parameter<std::string>(
      "tag_pose_topic",
      "/uav/visual_landing/apriltag_pose");
    this->declare_parameter<std::string>(
      "tag_marker_topic",
      "/uav/visual_landing/apriltag_marker");
    this->declare_parameter<std::string>(
      "controller_state_topic",
      "/uav/visual_landing/controller_state");
    this->declare_parameter<std::string>("debug_image_topic", "/uav/visual_landing/debug_image");
    this->declare_parameter<std::string>("tag_family", "36h11");
    this->declare_parameter<int>("target_marker_id", 0);
    this->declare_parameter<double>("tag_size_m", 0.20);
    this->declare_parameter<std::string>("camera_frame_id", "");
    this->declare_parameter<std::string>("base_frame_id", "uav_base_link");
    this->declare_parameter<std::string>("odometry_topic", "/ov_msckf/odomimu");
    this->declare_parameter<double>("odometry_timeout_s", 0.20);
    this->declare_parameter<double>("mono_in_ov_x_m", 0.0);
    this->declare_parameter<double>("mono_in_ov_y_m", 0.0);
    this->declare_parameter<double>("mono_in_ov_z_m", 0.0);
    this->declare_parameter<bool>("publish_tag_base_tf", true);
    this->declare_parameter<bool>("publish_tag_odom_tf", true);
    this->declare_parameter<std::string>("tag_tf_frame_prefix", "apriltag");
    this->declare_parameter<double>("confidence_reproj_scale_px", 4.0);
    this->declare_parameter<double>("confidence_min_marker_span_px", 18.0);
    this->declare_parameter<double>("confidence_full_marker_span_px", 72.0);

    image_topic_ = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    target_observation_topic_ = this->get_parameter("target_observation_topic").as_string();
    tag_detection_topic_ = this->get_parameter("tag_detection_topic").as_string();
    tag_pose_topic_ = this->get_parameter("tag_pose_topic").as_string();
    tag_marker_topic_ = this->get_parameter("tag_marker_topic").as_string();
    controller_state_topic_ = this->get_parameter("controller_state_topic").as_string();
    debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
    configured_camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    target_marker_id_ = this->get_parameter("target_marker_id").as_int();
    tag_size_m_ = static_cast<float>(this->get_parameter("tag_size_m").as_double());
    odometry_timeout_s_ = std::max(0.0, this->get_parameter("odometry_timeout_s").as_double());
    mono_in_ov_x_m_ = this->get_parameter("mono_in_ov_x_m").as_double();
    mono_in_ov_y_m_ = this->get_parameter("mono_in_ov_y_m").as_double();
    mono_in_ov_z_m_ = this->get_parameter("mono_in_ov_z_m").as_double();
    publish_tag_base_tf_ = this->get_parameter("publish_tag_base_tf").as_bool();
    publish_tag_odom_tf_ = this->get_parameter("publish_tag_odom_tf").as_bool();
    tag_tf_frame_prefix_ = sanitizeFrameToken(
      this->get_parameter("tag_tf_frame_prefix").as_string(), "apriltag");
    confidence_reproj_scale_px_ =
      static_cast<float>(this->get_parameter("confidence_reproj_scale_px").as_double());
    confidence_min_marker_span_px_ =
      static_cast<float>(this->get_parameter("confidence_min_marker_span_px").as_double());
    confidence_full_marker_span_px_ =
      static_cast<float>(this->get_parameter("confidence_full_marker_span_px").as_double());

    tag_dict_ = resolveTagDictionary(
      this->get_parameter("tag_family").as_string(), tag_family_);
    tag_params_ = cv::aruco::DetectorParameters::create();
    tag_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    if (publish_tag_base_tf_ || publish_tag_odom_tf_) {
      tag_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    auto observation_qos = rclcpp::SensorDataQoS();
    observation_qos.keep_last(1);
    observation_pub_ = this->create_publisher<uav_visual_landing::msg::TargetObservation>(
      target_observation_topic_, observation_qos);
    tag_detection_pub_ = this->create_publisher<uav_visual_landing::msg::AprilTagDetection>(
      tag_detection_topic_, observation_qos);
    tag_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      tag_pose_topic_, rclcpp::QoS(10).reliable());
    tag_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      tag_marker_topic_, rclcpp::QoS(1).reliable().transient_local());
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      debug_image_topic_, rclcpp::SensorDataQoS());
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
      {
        onCameraInfo(msg);
      });
    if (!controller_state_topic_.empty()) {
      controller_state_sub_ =
        this->create_subscription<uav_visual_landing::msg::LandingControllerState>(
        controller_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const uav_visual_landing::msg::LandingControllerState::SharedPtr msg)
        {
          telemetry_.phase = msg->phase;
          telemetry_.height_source = msg->height_source;
          telemetry_.terminal_trigger_source = msg->terminal_trigger_source;
          telemetry_.height_valid = msg->height_valid;
          telemetry_.height_measurement_source = msg->height_measurement_source;
          telemetry_.height_measurement_time_basis = msg->height_measurement_time_basis;
          telemetry_.height_measurement_sample_fresh = msg->height_measurement_sample_fresh;
          telemetry_.height_measurement_receive_fresh = msg->height_measurement_receive_fresh;
          telemetry_.height_measurement_m = msg->height_measurement_m;
          telemetry_.control_height_m = msg->control_height_m;
          telemetry_.err_u_norm_filtered = msg->err_u_norm_filtered;
          telemetry_.err_v_norm_filtered = msg->err_v_norm_filtered;
          telemetry_.err_u_rate_norm_s = msg->err_u_rate_norm_s;
          telemetry_.err_v_rate_norm_s = msg->err_v_rate_norm_s;
          telemetry_.lateral_error_valid = msg->lateral_error_valid;
          telemetry_.lateral_error_x_m = msg->lateral_error_x_m;
          telemetry_.lateral_error_y_m = msg->lateral_error_y_m;
          telemetry_.lateral_error_m = msg->lateral_error_m;
          telemetry_.lateral_error_rate_x_mps = msg->lateral_error_rate_x_mps;
          telemetry_.lateral_error_rate_y_mps = msg->lateral_error_rate_y_mps;
          telemetry_.active_max_vxy = msg->active_max_vxy;
          telemetry_.align_enter_lateral_m = msg->align_enter_lateral_m;
          telemetry_.align_exit_lateral_m = msg->align_exit_lateral_m;
          telemetry_.z_target_height_m = msg->z_target_height_m;
          telemetry_.z_error_m = msg->z_error_m;
          telemetry_.xy_control_mode = msg->xy_control_mode;
        });
    }
    if (!odometry_topic_.empty()) {
      odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          last_odometry_ = *msg;
          has_odometry_ = true;
        });
    }
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg)
      {
        onImage(msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "aruco_detector_node: image=%s camera_info=%s target_observation=%s tag_detection=%s "
      "tag_pose=%s tag_marker=%s debug_image=%s marker_id=%d family=%s tag_size=%.3f "
      "odom=%s mono_in_ov=(%.3f, %.3f, %.3f) tag_tf(base=%s odom=%s prefix=%s)",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      target_observation_topic_.c_str(),
      tag_detection_topic_.c_str(),
      tag_pose_topic_.c_str(),
      tag_marker_topic_.c_str(),
      debug_image_topic_.c_str(),
      target_marker_id_,
      tag_family_.c_str(),
      tag_size_m_,
      odometry_topic_.empty() ? "<disabled>" : odometry_topic_.c_str(),
      mono_in_ov_x_m_,
      mono_in_ov_y_m_,
      mono_in_ov_z_m_,
      publish_tag_base_tf_ ? "on" : "off",
      publish_tag_odom_tf_ ? "on" : "off",
      tag_tf_frame_prefix_.c_str());
  }

private:
  struct PoseCandidate
  {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    float yaw_rad{0.0f};
    float reproj_err_px{std::numeric_limits<float>::infinity()};
    float tag_depth_m{0.0f};
  };

  struct ControllerTelemetry
  {
    std::string phase{"READY"};
    std::string height_source{"ODOM"};
    std::string terminal_trigger_source{"NONE"};
    bool height_valid{false};
    std::string height_measurement_source{"NONE"};
    std::string height_measurement_time_basis{"NONE"};
    bool height_measurement_sample_fresh{false};
    bool height_measurement_receive_fresh{false};
    float height_measurement_m{0.0f};
    float control_height_m{0.0f};
    float err_u_norm_filtered{0.0f};
    float err_v_norm_filtered{0.0f};
    float err_u_rate_norm_s{0.0f};
    float err_v_rate_norm_s{0.0f};
    bool lateral_error_valid{false};
    float lateral_error_x_m{0.0f};
    float lateral_error_y_m{0.0f};
    float lateral_error_m{0.0f};
    float lateral_error_rate_x_mps{0.0f};
    float lateral_error_rate_y_mps{0.0f};
    float active_max_vxy{0.0f};
    float align_enter_lateral_m{0.08f};
    float align_exit_lateral_m{0.05f};
    float z_target_height_m{0.0f};
    float z_error_m{0.0f};
    std::string xy_control_mode{"vision_pd_metric"};
  };

  static float clamp01(float value)
  {
    return std::max(0.0f, std::min(value, 1.0f));
  }

  static float normalizeAngle(float angle)
  {
    while (angle > static_cast<float>(M_PI)) {
      angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
      angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
  }

  static cv::Vec3d matToVec3d(const cv::Mat & mat)
  {
    cv::Mat reshaped = mat.reshape(1, 3);
    cv::Mat numeric;
    reshaped.convertTo(numeric, CV_64F);
    return {numeric.at<double>(0, 0), numeric.at<double>(1, 0), numeric.at<double>(2, 0)};
  }

  static std::string sanitizeFrameToken(const std::string & raw, const char * fallback)
  {
    std::string sanitized;
    sanitized.reserve(raw.size());
    for (const unsigned char ch : raw) {
      if (std::isalnum(ch) || ch == '_' || ch == '/') {
        sanitized.push_back(static_cast<char>(ch));
      } else {
        sanitized.push_back('_');
      }
    }
    while (!sanitized.empty() && sanitized.front() == '/') {
      sanitized.erase(sanitized.begin());
    }
    while (!sanitized.empty() && sanitized.back() == '/') {
      sanitized.pop_back();
    }
    if (sanitized.empty()) {
      sanitized = fallback;
    }
    return sanitized;
  }

  static std::string canonicalizeTagFamily(const std::string & family)
  {
    std::string normalized;
    normalized.reserve(family.size());
    for (const char ch : family) {
      if (ch == '_' || ch == '-' || std::isspace(static_cast<unsigned char>(ch))) {
        continue;
      }
      normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
    }
    constexpr const char kDictPrefix[] = "DICTAPRILTAG";
    constexpr const char kAprilPrefix[] = "APRILTAG";
    if (normalized.rfind(kDictPrefix, 0) == 0U) {
      normalized.erase(0, sizeof(kDictPrefix) - 1U);
    } else if (normalized.rfind(kAprilPrefix, 0) == 0U) {
      normalized.erase(0, sizeof(kAprilPrefix) - 1U);
    }
    return normalized;
  }

  static cv::Ptr<cv::aruco::Dictionary> resolveTagDictionary(
    const std::string & family,
    std::string & resolved_family)
  {
    const std::string normalized = canonicalizeTagFamily(family);
    if (normalized == "16H5") {
      resolved_family = "16h5";
      return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    }
    if (normalized == "25H9") {
      resolved_family = "25h9";
      return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
    }
    if (normalized == "36H10") {
      resolved_family = "36h10";
      return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
    }
    if (normalized == "36H11") {
      resolved_family = "36h11";
      return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    }

    throw std::runtime_error(
            "unsupported AprilTag family '" + family +
            "' (supported: 16h5, 25h9, 36h10, 36h11)");
  }

  std::vector<cv::Point3f> buildMarkerObjectPoints() const
  {
    const float half = tag_size_m_ * 0.5f;
    return {
      {-half, half, 0.0f},
      {half, half, 0.0f},
      {half, -half, 0.0f},
      {-half, -half, 0.0f}};
  }

  static float computePoseYawRad(const cv::Vec3d & rvec)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    return normalizeAngle(
      static_cast<float>(std::atan2(
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0))));
  }

  static float computeTagDepthFromTvec(const cv::Vec3d & tvec)
  {
    return static_cast<float>(tvec[2]);
  }

  float computeTagDepthFromSpanPx(float marker_span_px) const
  {
    const float focal_mean = 0.5f * (fx_ + fy_);
    if (marker_span_px <= 1.0e-6f || focal_mean <= 1.0e-6f || tag_size_m_ <= 1.0e-6f) {
      return 0.0f;
    }
    return (tag_size_m_ * focal_mean) / marker_span_px;
  }

  static std::string legacyTimeBasisSuffix(const std::string & time_basis)
  {
    if (time_basis == "NONE" || time_basis == "SAMPLE_TIME") {
      return "";
    }
    return " legacy_time_basis=receive_time";
  }

  static tf2::Transform transformFromPose(const geometry_msgs::msg::Pose & pose)
  {
    tf2::Quaternion rotation(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    if (rotation.length2() <= 1.0e-12) {
      rotation.setRPY(0.0, 0.0, 0.0);
    } else {
      rotation.normalize();
    }
    return tf2::Transform(
      rotation,
      tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  }

  static tf2::Transform transformFromMsg(const geometry_msgs::msg::Transform & transform)
  {
    tf2::Quaternion rotation(
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z,
      transform.rotation.w);
    if (rotation.length2() <= 1.0e-12) {
      rotation.setRPY(0.0, 0.0, 0.0);
    } else {
      rotation.normalize();
    }
    return tf2::Transform(
      rotation,
      tf2::Vector3(
        transform.translation.x,
        transform.translation.y,
        transform.translation.z));
  }

  static geometry_msgs::msg::Point pointToMsg(const tf2::Vector3 & point)
  {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
    return msg;
  }

  bool rotateCameraPointIntoOvChildFallback(
    const std::string & camera_frame,
    const tf2::Vector3 & camera_point,
    tf2::Vector3 & child_point)
  {
    if (base_frame_id_.empty() || camera_frame.empty()) {
      return false;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_id_, camera_frame, tf2::TimePointZero);
      const tf2::Transform base_from_camera = transformFromMsg(transform.transform);
      const tf2::Transform rotation_only(base_from_camera.getRotation());
      child_point = rotation_only * camera_point;
      child_point += tf2::Vector3(mono_in_ov_x_m_, mono_in_ov_y_m_, mono_in_ov_z_m_);
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000,
        "failed to resolve fallback rotation %s <- %s for mono_in_ov translation: %s",
        base_frame_id_.c_str(),
        camera_frame.c_str(),
        e.what());
      return false;
    }
  }

  void publishDeleteMarker(const builtin_interfaces::msg::Time & stamp)
  {
    if (!marker_visible_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id =
      last_marker_frame_id_.empty() ? base_frame_id_ : last_marker_frame_id_;
    marker.ns = "apriltag";
    marker.id = last_marker_id_;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tag_marker_pub_->publish(marker);
    marker_visible_ = false;
  }

  std::string makeTagTfFrameId(int tag_id, const char * suffix) const
  {
    std::string frame_id = tag_tf_frame_prefix_;
    frame_id += "_";
    frame_id += tag_id >= 0 ? std::to_string(tag_id) : "unknown";
    if (suffix != nullptr && suffix[0] != '\0') {
      frame_id += "_";
      frame_id += suffix;
    }
    return frame_id;
  }

  void publishTagTransform(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & parent_frame,
    const geometry_msgs::msg::Point & position,
    int tag_id,
    const char * suffix)
  {
    if (!tag_tf_broadcaster_ || parent_frame.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = makeTagTfFrameId(tag_id, suffix);
    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;
    transform.transform.rotation.w = 1.0;
    tag_tf_broadcaster_->sendTransform(transform);
  }

  void publishTagTransforms(const uav_visual_landing::msg::AprilTagDetection & detection)
  {
    if (publish_tag_base_tf_ && detection.position_base_valid && !detection.base_frame_id.empty()) {
      publishTagTransform(
        detection.header.stamp,
        detection.base_frame_id,
        detection.position_base_m,
        detection.tag_id,
        "rel");
    }

    if (publish_tag_odom_tf_ && detection.position_odom_valid && !detection.odom_frame_id.empty()) {
      publishTagTransform(
        detection.header.stamp,
        detection.odom_frame_id,
        detection.position_odom_m,
        detection.tag_id,
        "");
    }
  }

  void publishPoseAndMarker(const uav_visual_landing::msg::AprilTagDetection & detection)
  {
    if (!detection.position_odom_valid || detection.odom_frame_id.empty()) {
      publishDeleteMarker(detection.header.stamp);
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = detection.header;
    pose.header.frame_id = detection.odom_frame_id;
    pose.pose.position.x = detection.position_odom_m.x;
    pose.pose.position.y = detection.position_odom_m.y;
    pose.pose.position.z = detection.position_odom_m.z;
    pose.pose.orientation.w = 1.0;
    tag_pose_pub_->publish(pose);

    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "apriltag";
    marker.id = detection.tag_id >= 0 ? detection.tag_id : 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = std::max(0.01f, detection.tag_size_m);
    marker.scale.y = std::max(0.01f, detection.tag_size_m);
    marker.scale.z = std::max(0.01f, 0.05f * detection.tag_size_m);
    marker.color.r = 0.10f;
    marker.color.g = 0.95f;
    marker.color.b = 0.15f;
    marker.color.a = 0.85f;
    tag_marker_pub_->publish(marker);

    marker_visible_ = true;
    last_marker_id_ = marker.id;
    last_marker_frame_id_ = marker.header.frame_id;
  }

  float computeSpanConfidence(float marker_span_px) const
  {
    const float span_den = std::max(
      confidence_full_marker_span_px_ - confidence_min_marker_span_px_, 1.0f);
    return clamp01((marker_span_px - confidence_min_marker_span_px_) / span_den);
  }

  float computeReprojectionErrorPx(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & corners,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec) const
  {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);
    if (projected_points.size() != corners.size() || projected_points.empty()) {
      return std::numeric_limits<float>::infinity();
    }

    double sum_sq = 0.0;
    for (size_t index = 0; index < corners.size(); ++index) {
      const double dx = static_cast<double>(projected_points[index].x - corners[index].x);
      const double dy = static_cast<double>(projected_points[index].y - corners[index].y);
      sum_sq += (dx * dx) + (dy * dy);
    }
    return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(corners.size())));
  }

  float computeMarkerSpanPx(const std::vector<cv::Point2f> & corners) const
  {
    if (corners.size() != 4) {
      return 0.0f;
    }
    float total = 0.0f;
    for (size_t index = 0; index < corners.size(); ++index) {
      const auto & a = corners[index];
      const auto & b = corners[(index + 1U) % corners.size()];
      total += std::hypot(b.x - a.x, b.y - a.y);
    }
    return 0.25f * total;
  }

  float computeDetectionConfidence(bool pose_valid, float reproj_err_px, float marker_span_px) const
  {
    const float size_conf = computeSpanConfidence(marker_span_px);
    if (!pose_valid || !std::isfinite(reproj_err_px)) {
      return 0.35f * size_conf;
    }
    const float reproj_scale = std::max(confidence_reproj_scale_px_, 1.0e-3f);
    const float reproj_conf = std::exp(-reproj_err_px / reproj_scale);
    return clamp01((0.70f * reproj_conf) + (0.30f * size_conf));
  }

  bool estimatePose(
    const std::vector<cv::Point2f> & corners,
    std::vector<PoseCandidate> & candidates)
  {
    candidates.clear();
    if (corners.size() != 4 || tag_size_m_ <= 1.0e-6f || !camera_info_received_) {
      return false;
    }

    const auto object_points = buildMarkerObjectPoints();
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    try {
      cv::solvePnPGeneric(
        object_points, corners, camera_matrix_, dist_coeffs_, rvecs, tvecs,
        false, cv::SOLVEPNP_IPPE_SQUARE);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "solvePnPGeneric failed: %s", e.what());
    }

    if (rvecs.empty() || tvecs.empty()) {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      try {
        const bool ok = cv::solvePnP(
          object_points, corners, camera_matrix_, dist_coeffs_, rvec, tvec,
          false, cv::SOLVEPNP_ITERATIVE);
        if (!ok) {
          return false;
        }
      } catch (const cv::Exception & e) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(), 2000, "solvePnP failed: %s", e.what());
        return false;
      }
      PoseCandidate candidate;
      candidate.rvec = rvec;
      candidate.tvec = tvec;
      candidate.yaw_rad = computePoseYawRad(candidate.rvec);
      candidate.reproj_err_px = computeReprojectionErrorPx(object_points, corners, rvec, tvec);
      candidate.tag_depth_m = computeTagDepthFromTvec(candidate.tvec);
      candidates.push_back(candidate);
      return true;
    }

    for (size_t index = 0; index < std::min(rvecs.size(), tvecs.size()); ++index) {
      PoseCandidate candidate;
      candidate.rvec = matToVec3d(rvecs[index]);
      candidate.tvec = matToVec3d(tvecs[index]);
      candidate.yaw_rad = computePoseYawRad(candidate.rvec);
      candidate.reproj_err_px = computeReprojectionErrorPx(
        object_points, corners, candidate.rvec,
        candidate.tvec);
      candidate.tag_depth_m = computeTagDepthFromTvec(candidate.tvec);
      candidates.push_back(candidate);
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const PoseCandidate & lhs, const PoseCandidate & rhs)
      {
        const bool lhs_valid = isPositiveFiniteDepth(lhs.tag_depth_m);
        const bool rhs_valid = isPositiveFiniteDepth(rhs.tag_depth_m);
        if (lhs_valid != rhs_valid) {
          return lhs_valid > rhs_valid;
        }
        return lhs.reproj_err_px < rhs.reproj_err_px;
      });
    return !candidates.empty();
  }

  int findTargetIndex(const std::vector<int> & ids) const
  {
    for (size_t index = 0; index < ids.size(); ++index) {
      if (target_marker_id_ < 0 || ids[index] == target_marker_id_) {
        return static_cast<int>(index);
      }
    }
    return -1;
  }

  std::string resolveCameraFrameId(const std::string & image_frame_id) const
  {
    if (!image_frame_id.empty()) {
      return image_frame_id;
    }
    if (!camera_info_frame_id_.empty()) {
      return camera_info_frame_id_;
    }
    return configured_camera_frame_id_;
  }

  int configuredOrUnknownMarkerId() const
  {
    return target_marker_id_ >= 0 ? target_marker_id_ : -1;
  }

  bool transformPoint(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::Vector3 & source_point,
    tf2::Vector3 & target_point,
    const char * context)
  {
    if (target_frame.empty() || source_frame.empty()) {
      return false;
    }
    if (target_frame == source_frame) {
      target_point = source_point;
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      target_point = transformFromMsg(transform.transform) * source_point;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000,
        "failed to resolve %s transform %s <- %s: %s",
        context,
        target_frame.c_str(),
        source_frame.c_str(),
        e.what());
      return false;
    }
  }

  bool fillDetectionTransforms(
    const sensor_msgs::msg::Image::SharedPtr & image_msg,
    const std::string & camera_frame,
    const cv::Vec3d & camera_tvec,
    uav_visual_landing::msg::AprilTagDetection & detection)
  {
    if (!detection.pose_valid) {
      return false;
    }

    const tf2::Vector3 camera_point(camera_tvec[0], camera_tvec[1], camera_tvec[2]);
    detection.position_camera_m = pointToMsg(camera_point);

    tf2::Vector3 base_point;
    if (transformPoint(base_frame_id_, camera_frame, camera_point, base_point, "base")) {
      detection.position_base_valid = true;
      detection.position_base_m = pointToMsg(base_point);
    }

    if (!has_odometry_ || odometry_topic_.empty()) {
      return detection.position_base_valid;
    }

    const rclcpp::Time image_stamp =
      ((image_msg->header.stamp.sec != 0) || (image_msg->header.stamp.nanosec != 0)) ?
      rclcpp::Time(image_msg->header.stamp) :
      this->now();
    const rclcpp::Time odom_stamp =
      ((last_odometry_.header.stamp.sec != 0) || (last_odometry_.header.stamp.nanosec != 0)) ?
      rclcpp::Time(last_odometry_.header.stamp) :
      this->now();
    const double sample_age_s = std::abs((image_stamp - odom_stamp).seconds());
    detection.odometry_sample_age_s = static_cast<float>(sample_age_s);
    detection.odom_frame_id = last_odometry_.header.frame_id;
    detection.odom_child_frame_id = last_odometry_.child_frame_id;
    if (!std::isfinite(sample_age_s) || sample_age_s > odometry_timeout_s_) {
      return detection.position_base_valid;
    }
    if (last_odometry_.header.frame_id.empty() || last_odometry_.child_frame_id.empty()) {
      return detection.position_base_valid;
    }

    tf2::Vector3 child_point;
    if (!transformPoint(
        last_odometry_.child_frame_id, camera_frame, camera_point, child_point, "odometry child"))
    {
      if (!rotateCameraPointIntoOvChildFallback(camera_frame, camera_point, child_point)) {
        return detection.position_base_valid;
      }
    }

    const tf2::Transform odom_from_child = transformFromPose(last_odometry_.pose.pose);
    const tf2::Vector3 odom_point = odom_from_child * child_point;
    detection.position_odom_valid = true;
    detection.position_odom_m = pointToMsg(odom_point);
    return true;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_frame_id_ = msg->header.frame_id;
    if (camera_info_received_) {
      return;
    }
    fx_ = static_cast<float>(msg->k[0]);
    fy_ = static_cast<float>(msg->k[4]);
    cx_ = static_cast<float>(msg->k[2]);
    cy_ = static_cast<float>(msg->k[5]);
    camera_info_received_ = true;

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_matrix_.at<double>(row, col) = msg->k[row * 3 + col];
      }
    }
    dist_coeffs_.assign(msg->d.begin(), msg->d.end());

    RCLCPP_INFO(
      this->get_logger(),
      "CameraInfo received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f frame_id=%s",
      fx_, fy_, cx_, cy_,
      camera_info_frame_id_.empty() ? "<unset>" : camera_info_frame_id_.c_str());
  }

  void publishDebugImage(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    const cv::Mat & gray_frame,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    int target_index,
    const uav_visual_landing::msg::TargetObservation & observation,
    const uav_visual_landing::msg::AprilTagDetection & detection)
  {
    if (debug_image_pub_->get_subscription_count() == 0U) {
      return;
    }

    cv::Mat debug_bgr;
    cv::cvtColor(gray_frame, debug_bgr, cv::COLOR_GRAY2BGR);
    if (!corners.empty()) {
      cv::aruco::drawDetectedMarkers(debug_bgr, corners, ids);
    }

    const cv::Point image_center(
      static_cast<int>(std::lround(cx_)),
      static_cast<int>(std::lround(cy_)));
    cv::drawMarker(debug_bgr, image_center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 28, 2);

    if (target_index >= 0) {
      const auto & target_corners = corners[static_cast<size_t>(target_index)];
      cv::Point target_center(0, 0);
      for (const auto & point : target_corners) {
        target_center.x += static_cast<int>(std::lround(point.x));
        target_center.y += static_cast<int>(std::lround(point.y));
      }
      target_center.x /= 4;
      target_center.y /= 4;
      cv::circle(debug_bgr, target_center, 6, cv::Scalar(0, 255, 0), 2);
      cv::line(debug_bgr, image_center, target_center, cv::Scalar(255, 255, 0), 2);
    }

    int y = 26;
    const int line_height = 22;
    const double font_scale = 0.60;
    const int thickness = 2;
    auto drawLine = [&](const std::string & text)
      {
        cv::putText(
          debug_bgr, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX,
          font_scale, cv::Scalar(16, 16, 16), thickness + 2, cv::LINE_AA);
        cv::putText(
          debug_bgr, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX,
          font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
        y += line_height;
      };

    drawLine("VISUAL LANDING DEBUG");
    drawLine("PHASE: " + telemetry_.phase);
    drawLine(observation.detected ? "TAG: DETECTED" : "TAG: MISSING");
    drawLine(
      cv::format(
        "TAG id=%d family=%s size=%.3f",
        detection.tag_id,
        detection.tag_family.c_str(),
        detection.tag_size_m));
    drawLine(
      cv::format(
        "TAG_Z=%.2f src=%s conf=%.2f",
        observation.tag_depth_m,
        observation.tag_depth_source.c_str(),
        observation.tag_depth_confidence));
    if (detection.pose_valid) {
      drawLine(
        cv::format(
          "CAM xyz=(%.2f %.2f %.2f)",
          detection.position_camera_m.x,
          detection.position_camera_m.y,
          detection.position_camera_m.z));
      if (detection.position_base_valid) {
        drawLine(
          cv::format(
            "BASE xyz=(%.2f %.2f %.2f)",
            detection.position_base_m.x,
            detection.position_base_m.y,
            detection.position_base_m.z));
      }
      if (detection.position_odom_valid) {
        drawLine(
          cv::format(
            "OV xyz=(%.2f %.2f %.2f) dt=%.3f",
            detection.position_odom_m.x,
            detection.position_odom_m.y,
            detection.position_odom_m.z,
            detection.odometry_sample_age_s));
      }
    }
    drawLine(
      cv::format(
        "H_CTRL=%.2f src=%s",
        telemetry_.control_height_m,
        telemetry_.height_source.c_str()));
    drawLine(
      cv::format(
        "RANGE=%.2f src=%s valid=%s",
        telemetry_.height_measurement_m,
        telemetry_.height_measurement_source.c_str(),
        telemetry_.height_valid ? "Y" : "N"));
    drawLine(
      cv::format(
        "HFRESH samp=%s recv=%s basis=%s%s",
        telemetry_.height_measurement_sample_fresh ? "Y" : "N",
        telemetry_.height_measurement_receive_fresh ? "Y" : "N",
        telemetry_.height_measurement_time_basis.c_str(),
        legacyTimeBasisSuffix(telemetry_.height_measurement_time_basis).c_str()));
    drawLine("TERM: " + telemetry_.terminal_trigger_source);
    drawLine(cv::format("ERR n=(%.4f, %.4f)", observation.err_u_norm, observation.err_v_norm));
    drawLine(
      cv::format(
        "ERR_F=(%.4f, %.4f)", telemetry_.err_u_norm_filtered,
        telemetry_.err_v_norm_filtered));
    drawLine(
      cv::format(
        "DERR=(%.4f, %.4f)", telemetry_.err_u_rate_norm_s,
        telemetry_.err_v_rate_norm_s));
    drawLine(
      cv::format(
        "LERR=%.3f xy=(%.3f, %.3f)",
        telemetry_.lateral_error_m,
        telemetry_.lateral_error_x_m,
        telemetry_.lateral_error_y_m));
    drawLine(
      cv::format(
        "TH=%.2f/%.2f VXY_MAX=%.3f",
        telemetry_.align_enter_lateral_m,
        telemetry_.align_exit_lateral_m,
        telemetry_.active_max_vxy));
    drawLine(cv::format("ZTG=%.2f ZE=%.2f", telemetry_.z_target_height_m, telemetry_.z_error_m));
    drawLine(
      cv::format(
        "YAW=%.3f rep=%.2f span=%.1f", observation.yaw_err_rad,
        observation.reproj_err_px, observation.marker_span_px));

    const auto output_msg = cv_bridge::CvImage(
      msg->header, sensor_msgs::image_encodings::BGR8,
      debug_bgr).toImageMsg();
    debug_image_pub_->publish(*output_msg);
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for CameraInfo on %s before publishing target observations",
        camera_info_topic_.c_str());
      return;
    }

    cv::Mat gray_frame;
    try {
      gray_frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "cv_bridge failed: %s", e.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray_frame, tag_dict_, corners, ids, tag_params_);

    const std::string camera_frame = resolveCameraFrameId(msg->header.frame_id);

    uav_visual_landing::msg::TargetObservation observation;
    observation.header = msg->header;

    uav_visual_landing::msg::AprilTagDetection detection;
    detection.header = msg->header;
    detection.tag_id = configuredOrUnknownMarkerId();
    detection.tag_family = tag_family_;
    detection.tag_size_m = tag_size_m_;
    detection.camera_frame_id = camera_frame;
    detection.base_frame_id = base_frame_id_;

    const int target_index = findTargetIndex(ids);
    if (target_index < 0) {
      observation_pub_->publish(observation);
      tag_detection_pub_->publish(detection);
      publishDeleteMarker(detection.header.stamp);
      publishDebugImage(msg, gray_frame, corners, ids, target_index, observation, detection);
      return;
    }

    detection.detected = true;
    detection.tag_id = ids[static_cast<size_t>(target_index)];

    const auto & target_corners = corners[static_cast<size_t>(target_index)];
    const cv::Point2f center_px(
      (target_corners[0].x + target_corners[1].x + target_corners[2].x + target_corners[3].x) *
      0.25f,
      (target_corners[0].y + target_corners[1].y + target_corners[2].y + target_corners[3].y) *
      0.25f);

    observation.detected = true;
    observation.pixel_err_u = center_px.x - cx_;
    observation.pixel_err_v = center_px.y - cy_;
    observation.err_u_norm = observation.pixel_err_u / std::max(fx_, 1.0e-6f);
    observation.err_v_norm = observation.pixel_err_v / std::max(fy_, 1.0e-6f);
    observation.marker_span_px = computeMarkerSpanPx(target_corners);

    const float edge_dx = target_corners[1].x - target_corners[0].x;
    const float edge_dy = target_corners[1].y - target_corners[0].y;
    observation.yaw_err_rad = normalizeAngle(std::atan2(edge_dy, edge_dx));

    const float span_conf = computeSpanConfidence(observation.marker_span_px);
    const float span_tag_depth_m = computeTagDepthFromSpanPx(observation.marker_span_px);

    std::vector<PoseCandidate> candidates;
    if (estimatePose(target_corners, candidates)) {
      observation.yaw_err_rad = candidates.front().yaw_rad;
      observation.reproj_err_px = candidates.front().reproj_err_px;
      if (isPositiveFiniteDepth(candidates.front().tag_depth_m)) {
        observation.pose_valid = true;
        observation.tag_depth_valid = true;
        observation.tag_depth_m = candidates.front().tag_depth_m;
        observation.tag_depth_source = "PNP_Z";
        observation.tag_depth_confidence = computeDetectionConfidence(
          true,
          observation.reproj_err_px,
          observation.marker_span_px);
        detection.pose_valid = true;
        fillDetectionTransforms(msg, camera_frame, candidates.front().tvec, detection);
      } else {
        observation.pose_valid = false;
        observation.tag_depth_valid = false;
        observation.tag_depth_m = 0.0f;
        observation.tag_depth_source = "NONE";
        observation.tag_depth_confidence = 0.0f;
      }
    } else if (span_tag_depth_m > 0.0f) {
      observation.tag_depth_valid = true;
      observation.tag_depth_m = span_tag_depth_m;
      observation.tag_depth_source = "SPAN_Z";
      observation.tag_depth_confidence = 0.60f * span_conf;
    } else {
      observation.tag_depth_valid = false;
      observation.tag_depth_m = 0.0f;
      observation.tag_depth_source = "NONE";
      observation.tag_depth_confidence = 0.0f;
    }

    observation.confidence = computeDetectionConfidence(
      observation.pose_valid, observation.reproj_err_px, observation.marker_span_px);
    detection.confidence = observation.confidence;
    detection.reproj_err_px = observation.reproj_err_px;

    observation_pub_->publish(observation);
    tag_detection_pub_->publish(detection);
    publishPoseAndMarker(detection);
    publishTagTransforms(detection);
    publishDebugImage(msg, gray_frame, corners, ids, target_index, observation, detection);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string target_observation_topic_;
  std::string tag_detection_topic_;
  std::string tag_pose_topic_;
  std::string tag_marker_topic_;
  std::string controller_state_topic_;
  std::string debug_image_topic_;
  std::string configured_camera_frame_id_;
  std::string camera_info_frame_id_;
  std::string base_frame_id_;
  std::string odometry_topic_;
  std::string tag_family_;
  int target_marker_id_{0};
  float tag_size_m_{0.20f};
  float fx_{0.0f};
  float fy_{0.0f};
  float cx_{0.0f};
  float cy_{0.0f};
  double odometry_timeout_s_{0.20};
  double mono_in_ov_x_m_{0.0};
  double mono_in_ov_y_m_{0.0};
  double mono_in_ov_z_m_{0.0};
  bool publish_tag_base_tf_{true};
  bool publish_tag_odom_tf_{true};
  std::string tag_tf_frame_prefix_{"apriltag"};
  float confidence_reproj_scale_px_{4.0f};
  float confidence_min_marker_span_px_{18.0f};
  float confidence_full_marker_span_px_{72.0f};
  bool camera_info_received_{false};
  bool has_odometry_{false};
  bool marker_visible_{false};
  int last_marker_id_{0};
  cv::Mat camera_matrix_;
  std::vector<double> dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> tag_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> tag_params_;
  ControllerTelemetry telemetry_{};
  nav_msgs::msg::Odometry last_odometry_{};
  std::string last_marker_frame_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tag_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<uav_visual_landing::msg::LandingControllerState>::SharedPtr
    controller_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<uav_visual_landing::msg::TargetObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<uav_visual_landing::msg::AprilTagDetection>::SharedPtr tag_detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tag_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

}  // namespace uav_visual_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_visual_landing::ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
