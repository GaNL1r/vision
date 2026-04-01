#ifndef AUTO_AIM__AIM_CORRECTOR__ACCUMULATIVE_CORRECTOR_HPP
#define AUTO_AIM__AIM_CORRECTOR__ACCUMULATIVE_CORRECTOR_HPP

#include "aim_corrector.hpp"
#include "single_filter.hpp"
#include <deque>
#include <chrono>

namespace auto_aim
{

struct ErrorSample
{
  double yaw_error;
  double pitch_error;
  double distance;
  std::chrono::steady_clock::time_point timestamp;
};

class AccumulativeCorrector : public AimCorrector
{
public:
  explicit AccumulativeCorrector(const std::string & config_path);

  Eigen::Vector2d get_correction() const override;

  void update(double distance, double yaw, double pitch) override;

  void add_shoot_event(int id, const AimInfo & aim) override;

  void set_image(const cv::Mat & img) override;

  void set_gimbal_state(double yaw, double pitch, double bullet_speed) override;

  bool is_enabled() const override { return enabled_; }

  void reset() override;

  std::string type_name() const override { return "accumulative"; }

  void add_manual_sample(double yaw_error, double pitch_error, double distance);

  void add_hit_feedback(bool hit, double distance);

private:
  void load_config(const std::string & config_path);
  double get_current_timestamp() const;

  bool enabled_;
  std::vector<double> q_vec_;
  std::vector<double> r_vec_;
  std::size_t max_history_;

  SingleFilter<1> yaw_filter_;
  SingleFilter<1> pitch_filter_;

  std::deque<ErrorSample> samples_;

  double current_distance_;
  double current_yaw_;
  double current_pitch_;
  double bullet_speed_;

  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIM_CORRECTOR__ACCUMULATIVE_CORRECTOR_HPP