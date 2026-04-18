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
#include <tf2/LinearMath/Matrix3x3.h>
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
    this->declare_parameter<std::string>("tag_output_frame_mode", "base");
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
    this->declare_parameter<double>("tag_camera_roll_offset_rad", 0.0);
    this->declare_parameter<double>("tag_camera_pitch_offset_rad", 0.0);
    this->declare_parameter<double>("tag_camera_yaw_offset_rad", 0.0);
    this->declare_parameter<double>("tag_frame_yaw_offset_rad", 0.0);
    this->declare_parameter<double>("mono_in_ov_x_m", 0.0);
    this->declare_parameter<double>("mono_in_ov_y_m", 0.0);
    this->declare_parameter<double>("mono_in_ov_z_m", 0.0);
    this->declare_parameter<double>("mono_in_ov_roll_rad", 0.0);
    this->declare_parameter<double>("mono_in_ov_pitch_rad", 0.0);
    this->declare_parameter<double>("mono_in_ov_yaw_rad", 0.0);
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
    tag_output_frame_mode_ = normalizeOutputFrameMode(
      this->get_parameter("tag_output_frame_mode").as_string());
    controller_state_topic_ = this->get_parameter("controller_state_topic").as_string();
    debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
    configured_camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    target_marker_id_ = this->get_parameter("target_marker_id").as_int();
    tag_size_m_ = static_cast<float>(this->get_parameter("tag_size_m").as_double());
    odometry_timeout_s_ = std::max(0.0, this->get_parameter("odometry_timeout_s").as_double());
    tag_camera_roll_offset_rad_ =
      this->get_parameter("tag_camera_roll_offset_rad").as_double();
    tag_camera_pitch_offset_rad_ =
      this->get_parameter("tag_camera_pitch_offset_rad").as_double();
    tag_camera_yaw_offset_rad_ =
      this->get_parameter("tag_camera_yaw_offset_rad").as_double();
    tag_frame_yaw_offset_rad_ =
      this->get_parameter("tag_frame_yaw_offset_rad").as_double();
    mono_in_ov_x_m_ = this->get_parameter("mono_in_ov_x_m").as_double();
    mono_in_ov_y_m_ = this->get_parameter("mono_in_ov_y_m").as_double();
    mono_in_ov_z_m_ = this->get_parameter("mono_in_ov_z_m").as_double();
    mono_in_ov_roll_rad_ = this->get_parameter("mono_in_ov_roll_rad").as_double();
    mono_in_ov_pitch_rad_ = this->get_parameter("mono_in_ov_pitch_rad").as_double();
    mono_in_ov_yaw_rad_ = this->get_parameter("mono_in_ov_yaw_rad").as_double();
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
      "tag_pose=%s tag_marker=%s camera_frame=%s base_frame=%s output_frame=%s debug_image=%s "
      "marker_id=%d family=%s tag_size=%.3f "
      "odom=%s tag_camera_rpy_offset=(%.3f, %.3f, %.3f) tag_frame_yaw_offset=%.3f "
      "mono_in_ov=(xyz=%.3f, %.3f, %.3f rpy=%.3f, %.3f, %.3f) "
      "tag_tf(base=%s odom=%s prefix=%s)",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      target_observation_topic_.c_str(),
      tag_detection_topic_.c_str(),
      tag_pose_topic_.c_str(),
      tag_marker_topic_.c_str(),
      configured_camera_frame_id_.empty() ? "<auto>" : configured_camera_frame_id_.c_str(),
      base_frame_id_.c_str(),
      tag_output_frame_mode_.c_str(),
      debug_image_topic_.c_str(),
      target_marker_id_,
      tag_family_.c_str(),
      tag_size_m_,
      odometry_topic_.empty() ? "<disabled>" : odometry_topic_.c_str(),
      tag_camera_roll_offset_rad_,
      tag_camera_pitch_offset_rad_,
      tag_camera_yaw_offset_rad_,
      tag_frame_yaw_offset_rad_,
      mono_in_ov_x_m_,
      mono_in_ov_y_m_,
      mono_in_ov_z_m_,
      mono_in_ov_roll_rad_,
      mono_in_ov_pitch_rad_,
      mono_in_ov_yaw_rad_,
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
    float normal_dot_view{std::numeric_limits<float>::infinity()};
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

  struct ResolvedTagPose
  {
    bool camera_valid{false};
    tf2::Transform camera_from_tag;
    bool base_valid{false};
    tf2::Transform base_from_tag;
    bool odom_valid{false};
    tf2::Transform odom_from_tag;
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

  static std::string normalizeOutputFrameMode(const std::string & mode)
  {
    std::string normalized;
    normalized.reserve(mode.size());
    for (const unsigned char ch : mode) {
      if (std::isalnum(ch)) {
        normalized.push_back(static_cast<char>(std::tolower(ch)));
      }
    }
    if (normalized == "base" || normalized == "odom" || normalized == "camera") {
      return normalized;
    }
    return "base";
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

  static float computeTransformYawRad(const tf2::Transform & transform)
  {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    return normalizeAngle(static_cast<float>(yaw));
  }

  static float computeTagNormalDotView(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    const cv::Vec3d normal_cam(
      rotation_matrix.at<double>(0, 2),
      rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 2));
    return static_cast<float>(normal_cam.dot(tvec));
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

  static tf2::Transform transformFromCvPose(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    const tf2::Matrix3x3 basis(
      rotation_matrix.at<double>(0, 0),
      rotation_matrix.at<double>(0, 1),
      rotation_matrix.at<double>(0, 2),
      rotation_matrix.at<double>(1, 0),
      rotation_matrix.at<double>(1, 1),
      rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0),
      rotation_matrix.at<double>(2, 1),
      rotation_matrix.at<double>(2, 2));

    tf2::Quaternion rotation;
    basis.getRotation(rotation);
    if (rotation.length2() <= 1.0e-12) {
      rotation.setRPY(0.0, 0.0, 0.0);
    } else {
      rotation.normalize();
    }

    return tf2::Transform(rotation, tf2::Vector3(tvec[0], tvec[1], tvec[2]));
  }

  static tf2::Transform rotateDetectedPoseInCameraFrame(
    const tf2::Transform & camera_from_tag,
    double roll_offset_rad,
    double pitch_offset_rad,
    double yaw_offset_rad)
  {
    const bool apply_roll =
      std::isfinite(roll_offset_rad) && std::abs(roll_offset_rad) > 1.0e-12;
    const bool apply_pitch =
      std::isfinite(pitch_offset_rad) && std::abs(pitch_offset_rad) > 1.0e-12;
    const bool apply_yaw =
      std::isfinite(yaw_offset_rad) && std::abs(yaw_offset_rad) > 1.0e-12;
    if (!(apply_roll || apply_pitch || apply_yaw)) {
      return camera_from_tag;
    }

    tf2::Quaternion camera_rotation_offset;
    camera_rotation_offset.setRPY(
      apply_roll ? roll_offset_rad : 0.0,
      apply_pitch ? pitch_offset_rad : 0.0,
      apply_yaw ? yaw_offset_rad : 0.0);
    camera_rotation_offset.normalize();
    // These launch-level offsets are intended to redefine the reported tag-frame
    // orientation, not to rotate the measured tag-center translation in camera
    // space. Post-multiply so the tag origin stays where solvePnP measured it.
    return camera_from_tag * tf2::Transform(
      camera_rotation_offset,
      tf2::Vector3(0.0, 0.0, 0.0));
  }

  static tf2::Transform rotateTagFrameInPlane(
    const tf2::Transform & parent_from_tag,
    double yaw_offset_rad)
  {
    if (!std::isfinite(yaw_offset_rad) || std::abs(yaw_offset_rad) <= 1.0e-12) {
      return parent_from_tag;
    }

    tf2::Quaternion tag_rotation_offset;
    tag_rotation_offset.setRPY(0.0, 0.0, yaw_offset_rad);
    tag_rotation_offset.normalize();
    return parent_from_tag * tf2::Transform(tag_rotation_offset, tf2::Vector3(0.0, 0.0, 0.0));
  }

  static geometry_msgs::msg::Point pointToMsg(const tf2::Vector3 & point)
  {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
    return msg;
  }

  static geometry_msgs::msg::Quaternion quaternionToMsg(const tf2::Quaternion & rotation)
  {
    geometry_msgs::msg::Quaternion msg;
    msg.x = rotation.x();
    msg.y = rotation.y();
    msg.z = rotation.z();
    msg.w = rotation.w();
    return msg;
  }

  static geometry_msgs::msg::Transform transformToMsg(const tf2::Transform & transform)
  {
    geometry_msgs::msg::Transform msg;
    msg.translation.x = transform.getOrigin().x();
    msg.translation.y = transform.getOrigin().y();
    msg.translation.z = transform.getOrigin().z();
    msg.rotation = quaternionToMsg(transform.getRotation());
    return msg;
  }

  static geometry_msgs::msg::Pose poseToMsg(const tf2::Transform & transform)
  {
    geometry_msgs::msg::Pose msg;
    msg.position = pointToMsg(transform.getOrigin());
    msg.orientation = quaternionToMsg(transform.getRotation());
    return msg;
  }

  tf2::Transform monoInOvFallbackTransform() const
  {
    tf2::Quaternion rotation;
    rotation.setRPY(
      mono_in_ov_roll_rad_,
      mono_in_ov_pitch_rad_,
      mono_in_ov_yaw_rad_);
    rotation.normalize();
    return tf2::Transform(
      rotation,
      tf2::Vector3(mono_in_ov_x_m_, mono_in_ov_y_m_, mono_in_ov_z_m_));
  }

  bool lookupFrameTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    tf2::Transform & target_from_source,
    const char * context)
  {
    if (target_frame.empty() || source_frame.empty()) {
      return false;
    }
    if (target_frame == source_frame) {
      target_from_source.setIdentity();
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      target_from_source = transformFromMsg(transform.transform);
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
    const tf2::Transform & parent_from_tag,
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
    transform.transform = transformToMsg(parent_from_tag);
    tag_tf_broadcaster_->sendTransform(transform);
  }

  void publishTagTransforms(
    const uav_visual_landing::msg::AprilTagDetection & detection,
    const ResolvedTagPose & resolved_pose)
  {
    if (publish_tag_base_tf_ && resolved_pose.base_valid && !detection.base_frame_id.empty()) {
      publishTagTransform(
        detection.header.stamp,
        detection.base_frame_id,
        resolved_pose.base_from_tag,
        detection.tag_id,
        "");
    }

    if (publish_tag_odom_tf_ && resolved_pose.odom_valid && !detection.odom_frame_id.empty()) {
      publishTagTransform(
        detection.header.stamp,
        detection.odom_frame_id,
        resolved_pose.odom_from_tag,
        detection.tag_id,
        "odom");
    }
  }

  bool resolveOutputPose(
    const uav_visual_landing::msg::AprilTagDetection & detection,
    const ResolvedTagPose & resolved_pose,
    std::string & frame_id,
    tf2::Transform & frame_from_tag) const
  {
    const auto try_mode = [&](const std::string & mode) -> bool {
        if (mode == "base") {
          if (resolved_pose.base_valid && !detection.base_frame_id.empty()) {
            frame_id = detection.base_frame_id;
            frame_from_tag = resolved_pose.base_from_tag;
            return true;
          }
          return false;
        }
        if (mode == "odom") {
          if (resolved_pose.odom_valid && !detection.odom_frame_id.empty()) {
            frame_id = detection.odom_frame_id;
            frame_from_tag = resolved_pose.odom_from_tag;
            return true;
          }
          return false;
        }
        if (mode == "camera") {
          if (resolved_pose.camera_valid && !detection.camera_frame_id.empty()) {
            frame_id = detection.camera_frame_id;
            frame_from_tag = resolved_pose.camera_from_tag;
            return true;
          }
          return false;
        }
        return false;
      };

    if (try_mode(tag_output_frame_mode_)) {
      return true;
    }

    if (tag_output_frame_mode_ == "base") {
      return try_mode("odom") || try_mode("camera");
    }
    if (tag_output_frame_mode_ == "odom") {
      return try_mode("base") || try_mode("camera");
    }
    return try_mode("base") || try_mode("odom");
  }

  void publishPoseAndMarker(
    const uav_visual_landing::msg::AprilTagDetection & detection,
    const ResolvedTagPose & resolved_pose)
  {
    std::string output_frame_id;
    tf2::Transform output_from_tag;
    if (!resolveOutputPose(detection, resolved_pose, output_frame_id, output_from_tag)) {
      publishDeleteMarker(detection.header.stamp);
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = detection.header;
    pose.header.frame_id = output_frame_id;
    pose.pose = poseToMsg(output_from_tag);
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
      candidate.normal_dot_view = computeTagNormalDotView(candidate.rvec, candidate.tvec);
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
      candidate.normal_dot_view = computeTagNormalDotView(candidate.rvec, candidate.tvec);
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
        const bool lhs_facing_camera =
          std::isfinite(lhs.normal_dot_view) && (lhs.normal_dot_view < 0.0f);
        const bool rhs_facing_camera =
          std::isfinite(rhs.normal_dot_view) && (rhs.normal_dot_view < 0.0f);
        if (lhs_facing_camera != rhs_facing_camera) {
          return lhs_facing_camera > rhs_facing_camera;
        }
        if (lhs_facing_camera && rhs_facing_camera) {
          if (std::abs(lhs.normal_dot_view - rhs.normal_dot_view) > 1.0e-6f) {
            return lhs.normal_dot_view < rhs.normal_dot_view;
          }
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
    // When a camera frame is configured from launch, prefer it over the
    // incoming image header so the detector follows the same TF that the
    // static camera mount publishes. This avoids silently bypassing the mono
    // mount TF when an external driver uses a different frame_id.
    if (!configured_camera_frame_id_.empty()) {
      if (!image_frame_id.empty() && image_frame_id != configured_camera_frame_id_) {
        RCLCPP_WARN(
          this->get_logger(),
          "image frame_id '%s' differs from configured camera_frame_id '%s'; "
          "using configured frame for TF lookups",
          image_frame_id.c_str(),
          configured_camera_frame_id_.c_str());
      }
      return configured_camera_frame_id_;
    }
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

  bool fillDetectionTransforms(
    const sensor_msgs::msg::Image::SharedPtr & image_msg,
    const std::string & camera_frame,
    const cv::Vec3d & camera_rvec,
    const cv::Vec3d & camera_tvec,
    uav_visual_landing::msg::AprilTagDetection & detection,
    ResolvedTagPose & resolved_pose)
  {
    if (!detection.pose_valid) {
      return false;
    }

    resolved_pose = ResolvedTagPose{};
    resolved_pose.camera_valid = true;
    resolved_pose.camera_from_tag = rotateTagFrameInPlane(
      rotateDetectedPoseInCameraFrame(
        transformFromCvPose(camera_rvec, camera_tvec),
        tag_camera_roll_offset_rad_,
        tag_camera_pitch_offset_rad_,
        tag_camera_yaw_offset_rad_),
      tag_frame_yaw_offset_rad_);
    detection.position_camera_m = pointToMsg(resolved_pose.camera_from_tag.getOrigin());

    tf2::Transform base_from_camera;
    if (lookupFrameTransform(base_frame_id_, camera_frame, base_from_camera, "base")) {
      resolved_pose.base_valid = true;
      resolved_pose.base_from_tag = base_from_camera * resolved_pose.camera_from_tag;
      detection.position_base_valid = true;
      detection.position_base_m = pointToMsg(resolved_pose.base_from_tag.getOrigin());
    }

    if (!has_odometry_ || odometry_topic_.empty()) {
      return resolved_pose.base_valid;
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
      return resolved_pose.base_valid;
    }
    if (last_odometry_.header.frame_id.empty() || last_odometry_.child_frame_id.empty()) {
      return resolved_pose.base_valid;
    }

    tf2::Transform child_from_camera;
    if (!lookupFrameTransform(
        last_odometry_.child_frame_id, camera_frame, child_from_camera, "odometry child"))
    {
      child_from_camera = monoInOvFallbackTransform();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000,
        "using mono_in_ov fallback transform for %s <- %s",
        last_odometry_.child_frame_id.c_str(),
        camera_frame.empty() ? "<unset>" : camera_frame.c_str());
    }

    const tf2::Transform odom_from_child = transformFromPose(last_odometry_.pose.pose);
    resolved_pose.odom_valid = true;
    resolved_pose.odom_from_tag =
      odom_from_child * child_from_camera * resolved_pose.camera_from_tag;
    detection.position_odom_valid = true;
    detection.position_odom_m = pointToMsg(resolved_pose.odom_from_tag.getOrigin());
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
    ResolvedTagPose resolved_pose;
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
    observation.yaw_err_rad = normalizeAngle(
      std::atan2(edge_dy, edge_dx) +
      static_cast<float>(tag_camera_yaw_offset_rad_ + tag_frame_yaw_offset_rad_));

    const float span_conf = computeSpanConfidence(observation.marker_span_px);
    const float span_tag_depth_m = computeTagDepthFromSpanPx(observation.marker_span_px);

    std::vector<PoseCandidate> candidates;
    if (estimatePose(target_corners, candidates)) {
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
        fillDetectionTransforms(
          msg,
          camera_frame,
          candidates.front().rvec,
          candidates.front().tvec,
          detection,
          resolved_pose);
        if (resolved_pose.camera_valid) {
          observation.yaw_err_rad = computeTransformYawRad(resolved_pose.camera_from_tag);
        }
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
    publishPoseAndMarker(detection, resolved_pose);
    publishTagTransforms(detection, resolved_pose);
    publishDebugImage(msg, gray_frame, corners, ids, target_index, observation, detection);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string target_observation_topic_;
  std::string tag_detection_topic_;
  std::string tag_pose_topic_;
  std::string tag_marker_topic_;
  std::string tag_output_frame_mode_;
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
  double tag_camera_roll_offset_rad_{0.0};
  double tag_camera_pitch_offset_rad_{0.0};
  double tag_camera_yaw_offset_rad_{0.0};
  double tag_frame_yaw_offset_rad_{0.0};
  double mono_in_ov_roll_rad_{0.0};
  double mono_in_ov_pitch_rad_{0.0};
  double mono_in_ov_yaw_rad_{0.0};
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
