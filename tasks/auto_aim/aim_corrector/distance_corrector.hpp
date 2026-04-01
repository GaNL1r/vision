#ifndef AUTO_AIM__AIM_CORRECTOR__DISTANCE_CORRECTOR_HPP
#define AUTO_AIM__AIM_CORRECTOR__DISTANCE_CORRECTOR_HPP

#include "aim_corrector.hpp"
#include <algorithm>
#include <vector>

namespace auto_aim
{

struct DistanceSegment
{
  double min_dist;
  double max_dist;
  double yaw_correction;
  double pitch_correction;
};

class DistanceCorrector : public AimCorrector
{
public:
  explicit DistanceCorrector(const std::string & config_path);

  Eigen::Vector2d get_correction() const override;

  void update(double distance, double yaw, double pitch) override;

  void add_shoot_event(int id, const AimInfo & aim) override;

  void set_image(const cv::Mat & img) override;

  void set_gimbal_state(double yaw, double pitch, double bullet_speed) override;

  bool is_enabled() const override { return enabled_; }

  void reset() override;

  std::string type_name() const override { return "distance"; }

private:
  void load_config(const std::string & config_path);
  Eigen::Vector2d interpolate(double distance) const;

  bool enabled_;
  std::vector<DistanceSegment> segments_;
  double current_distance_;
  double current_yaw_;
  double current_pitch_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIM_CORRECTOR__DISTANCE_CORRECTOR_HPP