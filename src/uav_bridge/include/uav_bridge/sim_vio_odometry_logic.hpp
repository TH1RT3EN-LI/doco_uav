#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <random>
#include <string>

namespace uav_bridge
{

struct SimVioOdometryConfig
{
  double output_delay_s{0.04};
  double warmup_duration_s{0.5};
  double position_noise_stddev_xy_m{0.03};
  double position_noise_stddev_z_m{0.02};
  double position_error_limit_xy_m{std::numeric_limits<double>::infinity()};
  double position_error_limit_z_m{std::numeric_limits<double>::infinity()};
  double roll_pitch_noise_stddev_rad{0.003};
  double yaw_noise_stddev_rad{0.01};
  double linear_velocity_noise_stddev_mps{0.05};
  double position_random_walk_xy_m_per_sqrt_s{0.002};
  double position_random_walk_z_m_per_sqrt_s{0.001};
  double yaw_random_walk_rad_per_sqrt_s{0.0005};
  double dropout_interval_mean_s{25.0};
  double dropout_duration_s{0.3};
  uint32_t random_seed{1U};
  std::string output_frame_id{"global"};
  std::string output_child_frame_id{"imu"};
};

struct SimVioOdometrySample
{
  int64_t stamp_ns{0};
  std::string frame_id;
  std::string child_frame_id;
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> linear_velocity{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 36> pose_covariance{};
  std::array<double, 36> twist_covariance{};
};

class SimVioOdometryLogic
{
public:
  explicit SimVioOdometryLogic(const SimVioOdometryConfig & config = {});

  void reset(int64_t start_time_ns = 0);
  void observeGroundTruth(const SimVioOdometrySample & sample);
  std::optional<SimVioOdometrySample> step(int64_t now_ns);

private:
  static int64_t secondsToNanoseconds(double seconds);
  static double square(double value);

  double sampleGaussian(double stddev);
  double sampleDropoutIntervalSeconds();
  void scheduleNextDropout(int64_t reference_time_ns);
  void applyRandomWalk(double dt_seconds);
  SimVioOdometrySample degradeSample(const SimVioOdometrySample & sample);

  SimVioOdometryConfig config_;
  std::mt19937 random_engine_;
  std::deque<SimVioOdometrySample> buffered_ground_truth_;
  std::optional<int64_t> last_emitted_stamp_ns_;
  int64_t warmup_until_ns_{0};
  int64_t next_dropout_start_ns_{std::numeric_limits<int64_t>::max()};
  int64_t dropout_end_ns_{0};
  bool in_dropout_{false};
  std::array<double, 3> position_random_walk_m_{{0.0, 0.0, 0.0}};
  double yaw_random_walk_rad_{0.0};
};

}  // namespace uav_bridge
