#ifndef AUTO_AIM__AIM_CORRECTOR__BULLET_TRACKING_CORRECTOR_HPP
#define AUTO_AIM__AIM_CORRECTOR__BULLET_TRACKING_CORRECTOR_HPP

#include "aim_corrector.hpp"
#include "single_filter.hpp"
#include "detect_bullet.hpp"
#include <deque>
#include <list>
#include <queue>
#include <chrono>

namespace auto_aim
{

constexpr std::size_t AIM_HISTORY_MAX_SZ = 200u;
constexpr std::size_t BULLETS_MAX_SZ = 200u;
constexpr std::size_t PENDING_IDS_MAX_SZ = 200u;
constexpr std::size_t ERROR_ANGLES_MAX_SZ = 15u;
constexpr double AIM_TIME_MAX_ERROR = 50e-3;
constexpr double CATCH_CIRCLE_DIS_MAX_RATIO = 4.0;
constexpr double CATCH_CIRCLE_RADIUS_MIN_RATIO = 0.707;
constexpr double FIT_CIRCLE_MAX_T = 1.0;
constexpr int FIT_CIRCLE_ITERATIONS_NUM = 15;
constexpr double DEFAULT_BULLET_RADIUS = 0.0085;
constexpr double DEFAULT_RESISTANCE_K = 0.01;
constexpr double GRAVITY = 9.8;

struct IdTLatencyAimCorrection
{
  enum
  {
    INVALID_ID = -1
  };
  int id;
  double img_t;
  double img_to_predict_latency;
  AimInfo aim;
  Eigen::Vector2d correction;

  static IdTLatencyAimCorrection invalid()
  {
    return IdTLatencyAimCorrection{
      IdTLatencyAimCorrection::INVALID_ID,
      0.0,
      0.0,
      AimInfo::idle(),
      Eigen::Vector2d::Zero()
    };
  }
};

class AimHistory
{
public:
  explicit AimHistory(std::size_t max_sz = AIM_HISTORY_MAX_SZ);

  void add_aim(const IdTLatencyAimCorrection & aim);
  int get_id_cnt() const;
  IdTLatencyAimCorrection find_by_img_t(double img_t) const;
  IdTLatencyAimCorrection find_by_id(int id) const;

private:
  std::size_t max_sz_;
  int id_cnt_ = 0;
  std::deque<IdTLatencyAimCorrection> aims_;
};

struct HitPos
{
  bool hit;
  Eigen::Vector3d pos;
};

struct HitCircle
{
  bool hit;
  cv::Point2f center;
  float radius;
};

struct CaughtCost
{
  bool caught;
  double cost;
};

class ProjectileSimulator
{
public:
  ProjectileSimulator(
    const IdTLatencyAimCorrection & aim,
    double fire_t,
    double v0,
    const Eigen::Vector3d & target_xyz,
    const Eigen::Matrix3d & camera_matrix,
    const cv::Mat & distort_coeffs);

  const IdTLatencyAimCorrection & get_aim_ref() const { return aim_; }
  double get_fire_t() const { return fire_t_; }

  HitPos get_pos_by_t(double t) const;
  HitPos get_pos() const;
  HitCircle get_circle_by_t(double t) const;
  HitCircle get_circle() const;

  CaughtCost catch_circle(const cv::Point2f & center, float radius) const;
  cv::Point2f fit_circle(const cv::Point2f & center, float radius) const;

private:
  double get_param_k() const;
  Eigen::Vector3d project_to_camera(const Eigen::Vector3d & xyz) const;
  cv::Point2f project_to_image(const Eigen::Vector3d & xyz) const;

  IdTLatencyAimCorrection aim_;
  double fire_t_;
  double v0_;
  Eigen::Vector3d target_xyz_;
  Eigen::Vector3d barrel_offset_;
  Eigen::Matrix3d camera_matrix_;
  cv::Mat distort_coeffs_;
  double bullet_radius_;
  double resistance_k_;
};

struct IdProj
{
  int id;
  std::shared_ptr<ProjectileSimulator> proj;
};

class BulletTrackingCorrector : public AimCorrector
{
public:
  explicit BulletTrackingCorrector(const std::string & config_path);

  Eigen::Vector2d get_correction() const override;

  void update(double distance, double yaw, double pitch) override;

  void add_shoot_event(int id, const AimInfo & aim) override;

  void set_image(const cv::Mat & img) override;

  void set_gimbal_state(double yaw, double pitch, double bullet_speed) override;

  bool is_enabled() const override { return enabled_; }

  void reset() override;

  std::string type_name() const override { return "bullet_tracking"; }

  void update_bullet_id(int last_shoot_id);
  std::vector<std::pair<int, Eigen::Vector3d>> get_bullets();
  void sample_aim_errors();

private:
  void load_config(const std::string & config_path);
  cv::Point2f undistort_point(const cv::Point2f & pt) const;
  double get_current_time() const;

  bool enabled_;
  bool sample_enabled_;
  bool correction_enabled_;

  SingleFilter<1> yaw_filter_;
  SingleFilter<1> pitch_filter_;

  AimHistory aim_history_;
  std::list<IdProj> bullets_;
  std::queue<int> pending_ids_;
  std::deque<Eigen::Vector2d> error_angles_;

  DetectBullet bullet_detector_;
  cv::Mat current_image_;
  Eigen::Quaterniond current_q_;
  Eigen::Matrix3d camera_matrix_;
  cv::Mat distort_coeffs_;

  int last_shoot_id_ = 0;
  double current_yaw_ = 0.0;
  double current_pitch_ = 0.0;
  double current_bullet_speed_ = 0.0;
  double bullet_radius_ = DEFAULT_BULLET_RADIUS;
  double resistance_k_ = DEFAULT_RESISTANCE_K;
  double control_to_fire_latency_ = 0.05;
  double img_to_control_latency_ = 0.03;

  std::vector<double> q_vec_{0.1};
  std::vector<double> r_vec_{1.0};

  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIM_CORRECTOR__BULLET_TRACKING_CORRECTOR_HPP