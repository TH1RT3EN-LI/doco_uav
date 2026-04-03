#include "uav_bridge/sim_vio_odometry_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace uav_bridge
{

namespace
{

constexpr double kMinQuaternionNormSquared = 1.0e-12;

double covarianceOrDefault(double value, double fallback)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return std::max(value, fallback);
}

double normalizeAngleLocal(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clampAbs(double value, double limit)
{
  if (!std::isfinite(limit) || limit < 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

void clampPlanarNorm(double * x, double * y, double limit)
{
  if (x == nullptr || y == nullptr || !std::isfinite(limit) || limit < 0.0) {
    return;
  }

  const double norm = std::hypot(*x, *y);
  if (norm <= limit || norm <= 1.0e-9) {
    return;
  }

  const double scale = limit / norm;
  *x *= scale;
  *y *= scale;
}

}  // namespace

SimVioOdometryLogic::SimVioOdometryLogic(const SimVioOdometryConfig & config)
: config_(config),
  random_engine_(config.random_seed)
{
  reset(0);
}

void SimVioOdometryLogic::reset(int64_t start_time_ns)
{
  buffered_ground_truth_.clear();
  last_emitted_stamp_ns_.reset();
  position_random_walk_m_ = {0.0, 0.0, 0.0};
  yaw_random_walk_rad_ = 0.0;
  in_dropout_ = false;
  dropout_end_ns_ = 0;
  warmup_until_ns_ = start_time_ns + secondsToNanoseconds(config_.warmup_duration_s);
  random_engine_.seed(config_.random_seed);
  scheduleNextDropout(warmup_until_ns_);
}

void SimVioOdometryLogic::observeGroundTruth(const SimVioOdometrySample & sample)
{
  if (!buffered_ground_truth_.empty() && sample.stamp_ns <= buffered_ground_truth_.back().stamp_ns) {
    return;
  }

  buffered_ground_truth_.push_back(sample);
}

std::optional<SimVioOdometrySample> SimVioOdometryLogic::step(int64_t now_ns)
{
  if (now_ns < warmup_until_ns_) {
    return std::nullopt;
  }

  if (in_dropout_) {
    if (now_ns < dropout_end_ns_) {
      return std::nullopt;
    }
    in_dropout_ = false;
    scheduleNextDropout(now_ns);
  }

  if (now_ns >= next_dropout_start_ns_) {
    if (secondsToNanoseconds(config_.dropout_duration_s) > 0) {
      in_dropout_ = true;
      dropout_end_ns_ = now_ns + secondsToNanoseconds(config_.dropout_duration_s);
    } else {
      scheduleNextDropout(now_ns);
    }
    return std::nullopt;
  }

  const int64_t target_stamp_ns = now_ns - secondsToNanoseconds(config_.output_delay_s);
  if (target_stamp_ns < 0) {
    return std::nullopt;
  }

  while (buffered_ground_truth_.size() >= 2 &&
    buffered_ground_truth_[1].stamp_ns <= target_stamp_ns)
  {
    buffered_ground_truth_.pop_front();
  }

  if (buffered_ground_truth_.empty() || buffered_ground_truth_.front().stamp_ns > target_stamp_ns) {
    return std::nullopt;
  }

  const auto & sample = buffered_ground_truth_.front();
  const double dt_seconds = last_emitted_stamp_ns_.has_value() ?
    std::max(0.0, static_cast<double>(sample.stamp_ns - *last_emitted_stamp_ns_) * 1.0e-9) :
    0.0;
  applyRandomWalk(dt_seconds);
  last_emitted_stamp_ns_ = sample.stamp_ns;
  return degradeSample(sample);
}

int64_t SimVioOdometryLogic::secondsToNanoseconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return 0;
  }

  return static_cast<int64_t>(std::llround(seconds * 1.0e9));
}

double SimVioOdometryLogic::square(double value)
{
  return value * value;
}

double SimVioOdometryLogic::sampleGaussian(double stddev)
{
  if (!std::isfinite(stddev) || stddev <= 0.0) {
    return 0.0;
  }

  std::normal_distribution<double> distribution(0.0, stddev);
  return distribution(random_engine_);
}

double SimVioOdometryLogic::sampleDropoutIntervalSeconds()
{
  if (!std::isfinite(config_.dropout_interval_mean_s) || config_.dropout_interval_mean_s <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  std::exponential_distribution<double> distribution(1.0 / config_.dropout_interval_mean_s);
  return distribution(random_engine_);
}

void SimVioOdometryLogic::scheduleNextDropout(int64_t reference_time_ns)
{
  const double interval_seconds = sampleDropoutIntervalSeconds();
  if (!std::isfinite(interval_seconds)) {
    next_dropout_start_ns_ = std::numeric_limits<int64_t>::max();
    return;
  }

  next_dropout_start_ns_ = reference_time_ns + std::max<int64_t>(
    1,
    secondsToNanoseconds(interval_seconds));
}

void SimVioOdometryLogic::applyRandomWalk(double dt_seconds)
{
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    return;
  }

  const double sqrt_dt = std::sqrt(dt_seconds);
  position_random_walk_m_[0] += sampleGaussian(config_.position_random_walk_xy_m_per_sqrt_s * sqrt_dt);
  position_random_walk_m_[1] += sampleGaussian(config_.position_random_walk_xy_m_per_sqrt_s * sqrt_dt);
  position_random_walk_m_[2] += sampleGaussian(config_.position_random_walk_z_m_per_sqrt_s * sqrt_dt);
  yaw_random_walk_rad_ = normalizeAngleLocal(
    yaw_random_walk_rad_ +
    sampleGaussian(config_.yaw_random_walk_rad_per_sqrt_s * sqrt_dt));
  clampPlanarNorm(
    &position_random_walk_m_[0],
    &position_random_walk_m_[1],
    config_.position_error_limit_xy_m);
  position_random_walk_m_[2] = clampAbs(
    position_random_walk_m_[2],
    config_.position_error_limit_z_m);
}

SimVioOdometrySample SimVioOdometryLogic::degradeSample(const SimVioOdometrySample & sample)
{
  SimVioOdometrySample output = sample;
  output.frame_id = config_.output_frame_id;
  output.child_frame_id = config_.output_child_frame_id;

  double position_error_x =
    position_random_walk_m_[0] + sampleGaussian(config_.position_noise_stddev_xy_m);
  double position_error_y =
    position_random_walk_m_[1] + sampleGaussian(config_.position_noise_stddev_xy_m);
  double position_error_z =
    position_random_walk_m_[2] + sampleGaussian(config_.position_noise_stddev_z_m);
  clampPlanarNorm(
    &position_error_x,
    &position_error_y,
    config_.position_error_limit_xy_m);
  position_error_z = clampAbs(position_error_z, config_.position_error_limit_z_m);

  output.position[0] += position_error_x;
  output.position[1] += position_error_y;
  output.position[2] += position_error_z;

  tf2::Quaternion orientation(
    sample.orientation_xyzw[0],
    sample.orientation_xyzw[1],
    sample.orientation_xyzw[2],
    sample.orientation_xyzw[3]);
  if (orientation.length2() <= kMinQuaternionNormSquared) {
    orientation = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  } else {
    orientation.normalize();
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  roll += sampleGaussian(config_.roll_pitch_noise_stddev_rad);
  pitch += sampleGaussian(config_.roll_pitch_noise_stddev_rad);
  yaw = normalizeAngleLocal(yaw + yaw_random_walk_rad_ + sampleGaussian(config_.yaw_noise_stddev_rad));

  tf2::Quaternion degraded_orientation;
  degraded_orientation.setRPY(roll, pitch, yaw);
  degraded_orientation.normalize();
  output.orientation_xyzw = {
    degraded_orientation.x(),
    degraded_orientation.y(),
    degraded_orientation.z(),
    degraded_orientation.w()};

  output.linear_velocity[0] += sampleGaussian(config_.linear_velocity_noise_stddev_mps);
  output.linear_velocity[1] += sampleGaussian(config_.linear_velocity_noise_stddev_mps);
  output.linear_velocity[2] += sampleGaussian(config_.linear_velocity_noise_stddev_mps);

  output.pose_covariance[0] = covarianceOrDefault(
    sample.pose_covariance[0],
    square(config_.position_noise_stddev_xy_m));
  output.pose_covariance[7] = covarianceOrDefault(
    sample.pose_covariance[7],
    square(config_.position_noise_stddev_xy_m));
  output.pose_covariance[14] = covarianceOrDefault(
    sample.pose_covariance[14],
    square(config_.position_noise_stddev_z_m));
  output.pose_covariance[21] = covarianceOrDefault(
    sample.pose_covariance[21],
    square(config_.roll_pitch_noise_stddev_rad));
  output.pose_covariance[28] = covarianceOrDefault(
    sample.pose_covariance[28],
    square(config_.roll_pitch_noise_stddev_rad));
  output.pose_covariance[35] = covarianceOrDefault(
    sample.pose_covariance[35],
    square(config_.yaw_noise_stddev_rad));

  output.twist_covariance[0] = covarianceOrDefault(
    sample.twist_covariance[0],
    square(config_.linear_velocity_noise_stddev_mps));
  output.twist_covariance[7] = covarianceOrDefault(
    sample.twist_covariance[7],
    square(config_.linear_velocity_noise_stddev_mps));
  output.twist_covariance[14] = covarianceOrDefault(
    sample.twist_covariance[14],
    square(config_.linear_velocity_noise_stddev_mps));

  return output;
}

}  // namespace uav_bridge
