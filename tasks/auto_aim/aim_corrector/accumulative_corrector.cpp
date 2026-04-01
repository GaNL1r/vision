#include "accumulative_corrector.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

AccumulativeCorrector::AccumulativeCorrector(const std::string & config_path)
: enabled_(true),
  q_vec_({0.1}),
  r_vec_({1.0}),
  max_history_(15),
  current_distance_(0.0),
  current_yaw_(0.0),
  current_pitch_(0.0),
  bullet_speed_(0.0)
{
  start_time_ = std::chrono::steady_clock::now();
  yaw_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  pitch_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  load_config(config_path);
}

void AccumulativeCorrector::load_config(const std::string & config_path)
{
  try {
    auto yaml = tools::load(config_path);

    if (yaml["aim_corrector"]) {
      auto corrector_yaml = yaml["aim_corrector"];
      enabled_ = tools::read_or(corrector_yaml, "enabled", true);

      if (corrector_yaml["accumulative"]) {
        auto acc_yaml = corrector_yaml["accumulative"];
        
        if (acc_yaml["q_vec"]) {
          q_vec_ = tools::read<std::vector<double>>(acc_yaml, "q_vec");
        }
        if (acc_yaml["r_vec"]) {
          r_vec_ = tools::read<std::vector<double>>(acc_yaml, "r_vec");
        }
        max_history_ = tools::read_or(acc_yaml, "max_history", static_cast<std::size_t>(15));
      }
    }

    tools::logger()->info("AccumulativeCorrector: initialized with q_vec size={}, r_vec size={}, max_history={}",
      q_vec_.size(), r_vec_.size(), max_history_);
  } catch (const std::exception & e) {
    tools::logger()->error("AccumulativeCorrector: Failed to load config: {}", e.what());
    enabled_ = false;
  }
}

double AccumulativeCorrector::get_current_timestamp() const
{
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
  return duration.count() / 1e6;
}

Eigen::Vector2d AccumulativeCorrector::get_correction() const
{
  if (!enabled_) {
    return Eigen::Vector2d::Zero();
  }

  if (!yaw_filter_.is_initialized() || !pitch_filter_.is_initialized()) {
    return Eigen::Vector2d::Zero();
  }

  return Eigen::Vector2d(yaw_filter_.get_x()(0), pitch_filter_.get_x()(0));
}

void AccumulativeCorrector::update(double distance, double yaw, double pitch)
{
  current_distance_ = distance;
  current_yaw_ = yaw;
  current_pitch_ = pitch;
}

void AccumulativeCorrector::add_shoot_event(int id, const AimInfo & aim)
{
}

void AccumulativeCorrector::set_image(const cv::Mat & img)
{
}

void AccumulativeCorrector::set_gimbal_state(double yaw, double pitch, double bullet_speed)
{
  current_yaw_ = yaw;
  current_pitch_ = pitch;
  bullet_speed_ = bullet_speed;
}

void AccumulativeCorrector::add_manual_sample(double yaw_error, double pitch_error, double distance)
{
  if (!enabled_) return;

  double t = get_current_timestamp();

  yaw_filter_.update(yaw_error, t, q_vec_, r_vec_);
  pitch_filter_.update(pitch_error, t, q_vec_, r_vec_);

  ErrorSample sample;
  sample.yaw_error = yaw_error;
  sample.pitch_error = pitch_error;
  sample.distance = distance;
  sample.timestamp = std::chrono::steady_clock::now();

  samples_.push_back(sample);
  while (samples_.size() > max_history_) {
    samples_.pop_front();
  }
}

void AccumulativeCorrector::add_hit_feedback(bool hit, double distance)
{
  if (!enabled_ || !hit) return;

  double t = get_current_timestamp();
  double base_yaw_corr = 0.0;
  double base_pitch_corr = 0.02;

  yaw_filter_.update(base_yaw_corr, t, q_vec_, r_vec_);
  pitch_filter_.update(base_pitch_corr, t, q_vec_, r_vec_);
}

void AccumulativeCorrector::reset()
{
  yaw_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  pitch_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  samples_.clear();
  current_distance_ = 0.0;
  current_yaw_ = 0.0;
  current_pitch_ = 0.0;
  start_time_ = std::chrono::steady_clock::now();
}

}  // namespace auto_aim