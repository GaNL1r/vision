#include "bullet_tracking_corrector.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/core/eigen.hpp>

namespace auto_aim
{

AimHistory::AimHistory(std::size_t max_sz) : max_sz_(max_sz) {}

void AimHistory::add_aim(const IdTLatencyAimCorrection & aim)
{
  if (aims_.size() >= max_sz_ && !aims_.empty()) {
    aims_.pop_front();
  }
  if (aims_.size() + 1 <= max_sz_) {
    aims_.push_back(aim);
  }
}

int AimHistory::get_id_cnt() const { return static_cast<int>(aims_.size()); }

IdTLatencyAimCorrection AimHistory::find_by_img_t(double img_t) const
{
  auto it = std::lower_bound(aims_.begin(), aims_.end(), img_t,
    [](const IdTLatencyAimCorrection & a, double t) { return a.img_t < t; });
  if (it == aims_.end()) {
    return IdTLatencyAimCorrection::invalid();
  }
  return *it;
}

IdTLatencyAimCorrection AimHistory::find_by_id(int id) const
{
  auto it = std::lower_bound(aims_.begin(), aims_.end(), id,
    [](const IdTLatencyAimCorrection & a, int id) { return a.id < id; });
  if (it == aims_.end() || it->id != id) {
    return IdTLatencyAimCorrection::invalid();
  }
  return *it;
}

ProjectileSimulator::ProjectileSimulator(
  const IdTLatencyAimCorrection & aim,
  double fire_t,
  double v0,
  const Eigen::Vector3d & target_xyz,
  const Eigen::Matrix3d & camera_matrix,
  const cv::Mat & distort_coeffs)
: aim_(aim),
  fire_t_(fire_t),
  v0_(v0),
  target_xyz_(target_xyz),
  camera_matrix_(camera_matrix),
  distort_coeffs_(distort_coeffs.clone()),
  bullet_radius_(DEFAULT_BULLET_RADIUS),
  resistance_k_(DEFAULT_RESISTANCE_K)
{
  barrel_offset_ = Eigen::Vector3d::Zero();
}

double ProjectileSimulator::get_param_k() const { return resistance_k_; }

Eigen::Vector3d ProjectileSimulator::project_to_camera(const Eigen::Vector3d & xyz) const
{
  return xyz;
}

cv::Point2f ProjectileSimulator::project_to_image(const Eigen::Vector3d & xyz) const
{
  std::vector<cv::Point2f> distorted;
  std::vector<cv::Point3f> object_points = {
    cv::Point3f(static_cast<float>(xyz.x()), static_cast<float>(xyz.y()), static_cast<float>(xyz.z()))};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat camera_mat;
  cv::eigen2cv(camera_matrix_, camera_mat);
  cv::projectPoints(object_points, rvec, tvec, camera_mat, distort_coeffs_, distorted);
  return distorted.empty() ? cv::Point2f() : distorted[0];
}

HitPos ProjectileSimulator::get_pos_by_t(double t) const
{
  double k = get_param_k();
  double aim_angle = aim_.aim.ypd.y();

  double w = (t - fire_t_) * v0_ * std::cos(aim_angle);
  double numerator = (k * v0_ * std::sin(aim_angle) + GRAVITY) * k * w;
  double denominator = k * k * v0_ * std::cos(aim_angle);
  double h = numerator / denominator + GRAVITY * std::log(1.0 - (k * w) / (v0_ * std::cos(aim_angle))) / (k * k);

  Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_.x(), target_xyz_.y(), 0).normalized();
  Eigen::Vector3d h_norm(0, 0, 1);
  Eigen::Vector3d bullet_xyz = w * w_norm + h * h_norm;

  Eigen::Vector2d bullet_xy(target_xyz_.x(), target_xyz_.y());
  double target_dist = bullet_xy.norm();

  return HitPos{w >= target_dist, bullet_xyz};
}

HitPos ProjectileSimulator::get_pos() const { return get_pos_by_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6); }

HitCircle ProjectileSimulator::get_circle_by_t(double t) const
{
  HitPos pos = get_pos_by_t(t);
  if (!pos.hit) {
    cv::Point2f center = project_to_image(pos.pos);
    float radius = static_cast<float>(bullet_radius_ * camera_matrix_(0, 0) / pos.pos.z());
    return HitCircle{false, center, radius};
  }
  return HitCircle{true, cv::Point2f(), 0.f};
}

HitCircle ProjectileSimulator::get_circle() const { return get_circle_by_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6); }

CaughtCost ProjectileSimulator::catch_circle(const cv::Point2f & center, float radius) const
{
  HitCircle hit_circle = get_circle();
  double dis = cv::norm(hit_circle.center - center);
  double r_ratio = hit_circle.radius > 0 ? radius / hit_circle.radius : 0.0;

  bool caught = dis <= CATCH_CIRCLE_DIS_MAX_RATIO * hit_circle.radius && r_ratio >= CATCH_CIRCLE_RADIUS_MIN_RATIO;
  double cost = dis / (CATCH_CIRCLE_DIS_MAX_RATIO * hit_circle.radius) + (1.0 - r_ratio) / (1.0 - CATCH_CIRCLE_RADIUS_MIN_RATIO);

  return CaughtCost{caught, cost};
}

cv::Point2f ProjectileSimulator::fit_circle(const cv::Point2f & center, float radius) const
{
  double low = fire_t_;
  double high = fire_t_ + FIT_CIRCLE_MAX_T;

  for (int i = 0; i < FIT_CIRCLE_ITERATIONS_NUM; ++i) {
    double mid = (low + high) / 2.0;
    HitCircle circle = get_circle_by_t(mid);
    if (circle.radius < radius) {
      low = mid;
    } else {
      high = mid;
    }
  }

  HitCircle result = get_circle_by_t((low + high) / 2.0);
  return result.center;
}

BulletTrackingCorrector::BulletTrackingCorrector(const std::string & config_path)
: enabled_(true),
  sample_enabled_(false),
  correction_enabled_(true),
  aim_history_(AIM_HISTORY_MAX_SZ),
  last_shoot_id_(0),
  current_yaw_(0.0),
  current_pitch_(0.0),
  current_bullet_speed_(0.0),
  bullet_radius_(DEFAULT_BULLET_RADIUS),
  resistance_k_(DEFAULT_RESISTANCE_K),
  control_to_fire_latency_(0.05),
  img_to_control_latency_(0.03)
{
  start_time_ = std::chrono::steady_clock::now();
  yaw_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  pitch_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  load_config(config_path);
}

void BulletTrackingCorrector::load_config(const std::string & config_path)
{
  try {
    auto yaml = tools::load(config_path);

    if (yaml["aim_corrector"]) {
      auto corrector_yaml = yaml["aim_corrector"];
      enabled_ = tools::read_or(corrector_yaml, "enabled", true);

      if (corrector_yaml["bullet_tracking"]) {
        auto bt_yaml = corrector_yaml["bullet_tracking"];
        sample_enabled_ = tools::read_or(bt_yaml, "sample_enabled", false);
        correction_enabled_ = tools::read_or(bt_yaml, "correction_enabled", true);
        bullet_radius_ = tools::read_or(bt_yaml, "bullet_radius", DEFAULT_BULLET_RADIUS);
        resistance_k_ = tools::read_or(bt_yaml, "resistance_k", DEFAULT_RESISTANCE_K);
        control_to_fire_latency_ = tools::read_or(bt_yaml, "control_to_fire_latency", 0.05);

        if (bt_yaml["q_vec"]) {
          q_vec_ = tools::read<std::vector<double>>(bt_yaml, "q_vec");
        }
        if (bt_yaml["r_vec"]) {
          r_vec_ = tools::read<std::vector<double>>(bt_yaml, "r_vec");
        }
      }
    }

    if (yaml["camera_matrix"]) {
      auto cm = tools::read<std::vector<double>>(yaml, "camera_matrix");
      if (cm.size() >= 9) {
        camera_matrix_ << cm[0], cm[1], cm[2], cm[3], cm[4], cm[5], cm[6], cm[7], cm[8];
      }
    }

    if (yaml["distort_coeffs"]) {
      auto dc = tools::read<std::vector<double>>(yaml, "distort_coeffs");
      distort_coeffs_ = cv::Mat(1, static_cast<int>(dc.size()), CV_64F);
      for (std::size_t i = 0; i < dc.size(); ++i) {
        distort_coeffs_.at<double>(0, static_cast<int>(i)) = dc[i];
      }
    }

    Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_gimbal2imubody = Eigen::Matrix3d::Identity();

    if (yaml["R_camera2gimbal"]) {
      auto rc2g = tools::read<std::vector<double>>(yaml, "R_camera2gimbal");
      if (rc2g.size() >= 9) {
        R_camera2gimbal << rc2g[0], rc2g[1], rc2g[2], rc2g[3], rc2g[4], rc2g[5], rc2g[6], rc2g[7], rc2g[8];
      }
    }

    if (yaml["R_gimbal2imubody"]) {
      auto rg2i = tools::read<std::vector<double>>(yaml, "R_gimbal2imubody");
      if (rg2i.size() >= 9) {
        R_gimbal2imubody << rg2i[0], rg2i[1], rg2i[2], rg2i[3], rg2i[4], rg2i[5], rg2i[6], rg2i[7], rg2i[8];
      }
    }

    R_imu2camera_ = (R_camera2gimbal * R_gimbal2imubody).transpose();

    cv::Mat cam_mat, imu_mat;
    cv::eigen2cv(camera_matrix_, cam_mat);
    cv::eigen2cv(R_imu2camera_, imu_mat);
    bullet_detector_.init(DoReproj(cam_mat, imu_mat));

    tools::logger()->info("BulletTrackingCorrector: initialized");
  } catch (const std::exception & e) {
    tools::logger()->error("BulletTrackingCorrector: Failed to load config: {}", e.what());
    enabled_ = false;
  }
}

double BulletTrackingCorrector::get_current_time() const
{
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
  return duration.count() / 1e6;
}

Eigen::Vector2d BulletTrackingCorrector::get_correction() const
{
  if (!enabled_ || !correction_enabled_) {
    return Eigen::Vector2d::Zero();
  }

  if (!yaw_filter_.is_initialized() || !pitch_filter_.is_initialized()) {
    return Eigen::Vector2d::Zero();
  }

  double t = get_current_time();
  return Eigen::Vector2d(yaw_filter_.predict(t)(0), pitch_filter_.predict(t)(0));
}

void BulletTrackingCorrector::update(double distance, double yaw, double pitch)
{
  current_yaw_ = yaw;
  current_pitch_ = pitch;
  sample_aim_errors();
}

void BulletTrackingCorrector::add_shoot_event(int id, const AimInfo & aim)
{
  if (!enabled_) return;

  double t = get_current_time();
  IdTLatencyAimCorrection correction;
  correction.id = id;
  correction.img_t = t;
  correction.img_to_predict_latency = 0.03;
  correction.aim = aim;
  correction.correction = get_correction();

  aim_history_.add_aim(correction);
}

void BulletTrackingCorrector::set_image(const cv::Mat & img) { current_image_ = img.clone(); }

void BulletTrackingCorrector::set_gimbal_state(double yaw, double pitch, double bullet_speed)
{
  current_yaw_ = yaw;
  current_pitch_ = pitch;
  current_bullet_speed_ = bullet_speed;
}

void BulletTrackingCorrector::set_quaternion(const Eigen::Quaterniond & q) { current_q_ = q; }

void BulletTrackingCorrector::update_bullet_id(int last_shoot_id)
{
  if (last_shoot_id != last_shoot_id_) {
    last_shoot_id_ = last_shoot_id;
    if (pending_ids_.size() + 1 <= PENDING_IDS_MAX_SZ) {
      pending_ids_.push(last_shoot_id);
    }
  }

  while (!pending_ids_.empty()) {
    int id = pending_ids_.front();
    IdTLatencyAimCorrection origin_aim = aim_history_.find_by_id(id);

    if (origin_aim.id == IdTLatencyAimCorrection::INVALID_ID) {
      pending_ids_.pop();
      continue;
    }

    double fire_controlling_t = origin_aim.img_t + control_to_fire_latency_;
    IdTLatencyAimCorrection fire_controlling_aim = aim_history_.find_by_img_t(fire_controlling_t);

    if (fire_controlling_aim.id == IdTLatencyAimCorrection::INVALID_ID) {
      break;
    }

    if (fire_controlling_t + AIM_TIME_MAX_ERROR < fire_controlling_aim.img_t) {
      pending_ids_.pop();
      continue;
    }

    if (bullets_.size() + 1 <= BULLETS_MAX_SZ) {
      auto proj = std::make_shared<ProjectileSimulator>(
        fire_controlling_aim, origin_aim.img_t + origin_aim.img_to_predict_latency + img_to_control_latency_ + control_to_fire_latency_,
        current_bullet_speed_ > 0 ? current_bullet_speed_ : 22.0,
        origin_aim.aim.aim_xyz,
        camera_matrix_,
        distort_coeffs_);
      bullets_.push_back({id, proj});
    }
    pending_ids_.pop();
  }
}

std::vector<std::pair<int, Eigen::Vector3d>> BulletTrackingCorrector::get_bullets()
{
  std::vector<std::pair<int, Eigen::Vector3d>> result;
  double t = get_current_time();

  for (auto it = bullets_.begin(); it != bullets_.end();) {
    if (t < it->proj->get_fire_t()) {
      ++it;
      continue;
    }

    HitPos hit_pos = it->proj->get_pos_by_t(t);
    if (hit_pos.hit) {
      it = bullets_.erase(it);
    } else {
      result.push_back({it->id, hit_pos.pos});
      ++it;
    }
  }
  return result;
}

std::vector<AimCorrector::BulletCircle> BulletTrackingCorrector::get_bullet_circles()
{
  std::vector<BulletCircle> result;
  double t = get_current_time();

  for (auto it = bullets_.begin(); it != bullets_.end();) {
    if (t < it->proj->get_fire_t()) {
      ++it;
      continue;
    }

    HitCircle hit_circle = it->proj->get_circle_by_t(t);
    if (hit_circle.hit) {
      it = bullets_.erase(it);
    } else {
      result.push_back({it->id, hit_circle.center, hit_circle.radius});
      ++it;
    }
  }
  return result;
}

void BulletTrackingCorrector::sample_aim_errors()
{
  if (!enabled_ || !sample_enabled_ || current_image_.empty()) {
    return;
  }

  bullet_detector_.process_new_frame(current_image_, current_q_);

  std::vector<ImageBullet> detected = bullet_detector_.get_bullets();
  std::vector<std::pair<cv::Point2f, float>> undistorted_detected;
  for (const auto & d : detected) {
    cv::Point2f undistorted_center = undistort_point(d.center);
    undistorted_detected.push_back({undistorted_center, d.radius});
  }

  double t = get_current_time();
  for (auto & bullet : bullets_) {
    auto best_it = undistorted_detected.end();
    CaughtCost best_caught{false, 0.0};

    for (auto it = undistorted_detected.begin(); it != undistorted_detected.end(); ++it) {
      HitCircle hit_circle = bullet.proj->get_circle_by_t(t);
      if (!hit_circle.hit) {
        double dis = cv::norm(hit_circle.center - it->first);
        double r_ratio = hit_circle.radius > 0 ? it->second / hit_circle.radius : 0.0;
        bool caught = dis <= CATCH_CIRCLE_DIS_MAX_RATIO * hit_circle.radius && r_ratio >= CATCH_CIRCLE_RADIUS_MIN_RATIO;
        double cost = dis / (CATCH_CIRCLE_DIS_MAX_RATIO * hit_circle.radius) + (1.0 - r_ratio) / (1.0 - CATCH_CIRCLE_RADIUS_MIN_RATIO);
        if (caught && (static_cast<int>(caught) > static_cast<int>(best_caught.caught) || cost < best_caught.cost)) {
          best_it = it;
          best_caught = {caught, cost};
        }
      }
    }

    if (best_it != undistorted_detected.end()) {
      double yaw_error = (best_it->first.x - current_image_.cols / 2.0) / camera_matrix_(0, 0);
      double pitch_error = (best_it->first.y - camera_matrix_(1, 2)) / camera_matrix_(1, 1);

      Eigen::Vector2d correction = bullet.proj->get_aim_ref().correction;
      yaw_filter_.update(yaw_error + correction(0), t, q_vec_, r_vec_);
      pitch_filter_.update(pitch_error + correction(1), t, q_vec_, r_vec_);

      if (error_angles_.size() + 1 > ERROR_ANGLES_MAX_SZ && !error_angles_.empty()) {
        error_angles_.pop_front();
      }
      if (error_angles_.size() + 1 <= ERROR_ANGLES_MAX_SZ) {
        error_angles_.push_back(Eigen::Vector2d(yaw_error, pitch_error));
      }

      undistorted_detected.erase(best_it);
    }
  }
}

cv::Point2f BulletTrackingCorrector::undistort_point(const cv::Point2f & pt) const
{
  std::vector<cv::Point2f> src{pt};
  std::vector<cv::Point2f> dst;
  cv::Mat camera_mat;
  cv::eigen2cv(camera_matrix_, camera_mat);
  cv::undistortPoints(src, dst, camera_mat, distort_coeffs_);
  cv::Mat new_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  new_camera_matrix.at<double>(0, 0) = camera_matrix_(0, 0);
  new_camera_matrix.at<double>(1, 1) = camera_matrix_(1, 1);
  new_camera_matrix.at<double>(0, 2) = camera_matrix_(0, 2);
  new_camera_matrix.at<double>(1, 2) = camera_matrix_(1, 2);

  return dst.empty() ? pt : cv::Point2f(dst[0].x * camera_matrix_(0, 0) + camera_matrix_(0, 2),
                                         dst[0].y * camera_matrix_(1, 1) + camera_matrix_(1, 2));
}

void BulletTrackingCorrector::reset()
{
  yaw_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  pitch_filter_.init_x(Eigen::Matrix<double, 1, 1>::Zero());
  aim_history_ = AimHistory(AIM_HISTORY_MAX_SZ);
  bullets_.clear();
  while (!pending_ids_.empty()) {
    pending_ids_.pop();
  }
  error_angles_.clear();
  last_shoot_id_ = 0;
  start_time_ = std::chrono::steady_clock::now();
}

}  // namespace auto_aim