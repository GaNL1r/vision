#include "distance_corrector.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

DistanceCorrector::DistanceCorrector(const std::string & config_path)
: enabled_(true), current_distance_(0.0), current_yaw_(0.0), current_pitch_(0.0)
{
  load_config(config_path);
}

void DistanceCorrector::load_config(const std::string & config_path)
{
  try {
    auto yaml = tools::load(config_path);

    if (yaml["aim_corrector"]) {
      auto corrector_yaml = yaml["aim_corrector"];
      enabled_ = tools::read_or(corrector_yaml, "enabled", true);

      if (corrector_yaml["distance_segments"]) {
        auto segments_yaml = corrector_yaml["distance_segments"];
        for (std::size_t i = 0; i < segments_yaml.size(); ++i) {
          auto seg_yaml = segments_yaml[i];
          DistanceSegment seg;
          seg.min_dist = tools::read_or(seg_yaml, "min_dist", 0.0);
          seg.max_dist = tools::read_or(seg_yaml, "max_dist", 10.0);
          seg.yaw_correction = tools::read_or(seg_yaml, "yaw_correction", 0.0) / 57.3;
          seg.pitch_correction = tools::read_or(seg_yaml, "pitch_correction", 0.0) / 57.3;
          segments_.push_back(seg);
        }
      }
    }

    if (segments_.empty()) {
      tools::logger()->warn("DistanceCorrector: No segments loaded, using default");
      segments_ = {
        {0.0, 2.0, 0.0, 0.0},
        {2.0, 4.0, 0.0, 0.02},
        {4.0, 6.0, 0.0, 0.05},
        {6.0, 8.0, 0.01, 0.08},
        {8.0, 10.0, 0.02, 0.10}
      };
    }

    std::sort(segments_.begin(), segments_.end(),
      [](const DistanceSegment & a, const DistanceSegment & b) {
        return a.min_dist < b.min_dist;
      });

    tools::logger()->info("DistanceCorrector: Loaded {} segments", segments_.size());
  } catch (const std::exception & e) {
    tools::logger()->error("DistanceCorrector: Failed to load config: {}", e.what());
    enabled_ = false;
  }
}

Eigen::Vector2d DistanceCorrector::interpolate(double distance) const
{
  if (segments_.empty()) {
    return Eigen::Vector2d::Zero();
  }

  if (distance <= segments_.front().min_dist) {
    return Eigen::Vector2d(segments_.front().yaw_correction, segments_.front().pitch_correction);
  }

  if (distance >= segments_.back().max_dist) {
    return Eigen::Vector2d(segments_.back().yaw_correction, segments_.back().pitch_correction);
  }

  for (std::size_t i = 0; i < segments_.size() - 1; ++i) {
    if (distance >= segments_[i].min_dist && distance < segments_[i + 1].min_dist) {
      double t = (distance - segments_[i].min_dist) / (segments_[i + 1].min_dist - segments_[i].min_dist);
      double yaw = segments_[i].yaw_correction * (1 - t) + segments_[i + 1].yaw_correction * t;
      double pitch = segments_[i].pitch_correction * (1 - t) + segments_[i + 1].pitch_correction * t;
      return Eigen::Vector2d(yaw, pitch);
    }
  }

  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d DistanceCorrector::get_correction() const
{
  if (!enabled_) {
    return Eigen::Vector2d::Zero();
  }
  return interpolate(current_distance_);
}

void DistanceCorrector::update(double distance, double yaw, double pitch)
{
  current_distance_ = distance;
  current_yaw_ = yaw;
  current_pitch_ = pitch;
}

void DistanceCorrector::add_shoot_event(int id, const AimInfo & aim)
{
}

void DistanceCorrector::set_image(const cv::Mat & img)
{
}

void DistanceCorrector::set_gimbal_state(double yaw, double pitch, double bullet_speed)
{
}

void DistanceCorrector::reset()
{
  current_distance_ = 0.0;
  current_yaw_ = 0.0;
  current_pitch_ = 0.0;
}

}  // namespace auto_aim