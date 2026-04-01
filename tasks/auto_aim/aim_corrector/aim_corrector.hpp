#ifndef AUTO_AIM__AIM_CORRECTOR__AIM_CORRECTOR_HPP
#define AUTO_AIM__AIM_CORRECTOR__AIM_CORRECTOR_HPP

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>

namespace auto_aim
{

struct AimInfo
{
  Eigen::Vector2d ypd;
  Eigen::Vector2d ypd_v;
  double shoot_param_v0;
  Eigen::Vector3d aim_xyz;
  int shoot_mode;

  static AimInfo idle()
  {
    return AimInfo{
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      0.0,
      Eigen::Vector3d::Zero(),
      0
    };
  }
};

struct CorrectionResult
{
  double yaw_correction;
  double pitch_correction;
  bool valid;
};

class AimCorrector
{
public:
  virtual ~AimCorrector() = default;

  virtual Eigen::Vector2d get_correction() const = 0;

  virtual void update(double distance, double yaw, double pitch) = 0;

  virtual void add_shoot_event(int id, const AimInfo & aim) = 0;

  virtual void set_image(const cv::Mat & img) = 0;

  virtual void set_gimbal_state(double yaw, double pitch, double bullet_speed) = 0;

  virtual bool is_enabled() const = 0;

  virtual void reset() = 0;

  virtual std::string type_name() const = 0;
};

class AimCorrectorFactory
{
public:
  enum class Type
  {
    NONE,
    DISTANCE,
    ACCUMULATIVE,
    BULLET_TRACKING
  };

  static std::shared_ptr<AimCorrector> create(
    Type type,
    const std::string & config_path);

  static Type type_from_string(const std::string & str);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIM_CORRECTOR__AIM_CORRECTOR_HPP