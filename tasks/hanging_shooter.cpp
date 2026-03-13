#include "hanging_shooter.hpp"

namespace auto_aim {
HangingShooter::HangingShooter(const std::string &config_path) {
  auto yaml = YAML::LoadFile(config_path);
  g_ = yaml["g"].as<double>();
  m_ = yaml["m"].as<double>();
  C_ = yaml["C"].as<double>();
  rho_ = yaml["rho"].as<double>();
  BigBulletRadius_ = yaml["BigBulletRadius"].as<double>();
  PI_ = yaml["PI"].as<double>();
  target_x_ = yaml["target_x"].as<double>();
  target_y_ = yaml["target_y"].as<double>();
  target_z_ = yaml["target_z"].as<double>();
  location_ = {0.0, 0.0, 0.0};
  best_angle_ = 0.0;

  trajectory_config_.enable_visualization =
      yaml["trajectory_enable_visualization"].as<bool>(false);
  trajectory_config_.trajectory_color_b =
      yaml["trajectory_color_b"].as<int>(255);
  trajectory_config_.trajectory_color_g = yaml["trajectory_color_g"].as<int>(0);
  trajectory_config_.trajectory_color_r = yaml["trajectory_color_r"].as<int>(0);
  trajectory_config_.circle_thickness = yaml["circle_thickness"].as<int>(2);
  trajectory_config_.sample_interval =
      yaml["trajectory_sample_interval"].as<int>(10);
}

/*
输入参数：
x_vals       打击距离数组
y_vals       打击高度数组
angle        发射角度     单位：度
bullet_speed 发射速度     单位：m/s
dt           步长        单位： s
max_time     极限时间     单位：s

作用：根据输入角度和速度，使用四阶龙格库塔进行弹道解算，获得离散的弹道映射关系，结果存储在数组当中
*/
void HangingShooter::RungeKutta_4(std::vector<double> &x_vals,
                                  std::vector<double> &y_vals, double angle,
                                  double bullet_speed, double height, double dt,
                                  double max_time) {
  angle = angle * PI_ / 180.0; // 先转化为弧度制
  double vx = bullet_speed * cos(angle);
  double vy = bullet_speed * sin(angle);
  // 显然初始位置为(0,0)
  x_vals.emplace_back(0);
  y_vals.emplace_back(0);
  double x = 0, y = 0;
  double t = 0;
  double S = PI_ * BigBulletRadius_ * BigBulletRadius_; // 大弹丸截面积
  double D = C_ * rho_ * S / 2; // 空气阻力的影响系数，推导请看知识库
  // 开始迭代求解
  bool is_fall = false;
  while (y >= 0 && t < max_time) // 弹丸落地和超出时间范围停止迭代
  {
    if (y >= height)
      is_fall = true;
    if (y < height && is_fall) // 开始下落且已到达目标高度则停止迭代
      break;
    double k1x = vx;
    double k1y = vy;
    double k1vx = -D / m_ * k1x * k1x;
    double k1vy = -g_ - D / m_ * k1y * k1y;

    double k2x = k1x + 0.5 * dt * k1vx;
    double k2y = k1y + 0.5 * dt * k1vy;
    double k2vx = k1vx + 0.5 * dt * (-D / m_ * k2x * k2x);
    double k2vy = k1vy + 0.5 * dt * (-g_ - D / m_ * k2y * k2y);

    double k3x = vx + 0.5 * dt * k2vx;
    double k3y = vy + 0.5 * dt * k2vy;
    double k3vx = k2vx + 0.5 * dt * (-D / m_ * k3x * k3x);
    double k3vy = k2vy + 0.5 * dt * (-g_ - D / m_ * k3y * k3y);

    double k4x = vx + dt * k3vx;
    double k4y = vy + dt * k3vy;
    double k4vx = k3vx + 0.5 * dt * (-D / m_ * k4x * k4x);
    double k4vy = k3vy + 0.5 * dt * (-g_ - D / m_ * k4y * k4y);

    // 更新状态
    x += dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
    y += dt / 6 * (k1y + 2 * k2y + 2 * k3y + k4y);
    vx += dt / 6 * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
    vy += dt / 6 * (k1vy + 2 * k2vy + 2 * k3vy + k4vy);
    // 存储轨迹
    x_vals.emplace_back(x);
    y_vals.emplace_back(y);
    // 计算下一时间点x和y的值
    t += dt;
  }
}

double HangingShooter::calculate_pitch(double current_x, double current_y,
                                       double current_z, double bullet_speed) {
  double distance = sqrt((current_x - target_x_) * (current_x - target_x_) +
                         (current_y - target_y_) * (current_y - target_y_));
  double height = target_z_ - current_z;
  std::vector<double> errors;
  for (double angle = 5.0; angle < 45.0; angle += 0.01) {
    std::vector<double> x_vals, y_vals;
    RungeKutta_4(x_vals, y_vals, angle, bullet_speed, height);
    double x_simple = x_vals.back(), y_simple = y_vals.back();
    double error = sqrt((x_simple - distance) * (x_simple - distance) +
                        (y_simple - height) * (y_simple - height));
    errors.emplace_back(error);
  }

  auto min_iter = std::min_element(errors.begin(), errors.end());
  best_angle_ = (min_iter - errors.begin()) / 100.0 + 5.0;

  trajectory_points_.clear();
  std::vector<double> x_vals, y_vals;
  RungeKutta_4(x_vals, y_vals, best_angle_, bullet_speed, height);

  double dir_x = (target_x_ - current_x) / distance;
  double dir_y = (target_y_ - current_y) / distance;

  for (size_t i = 0; i < x_vals.size();
       i += trajectory_config_.sample_interval) {
    TrajectoryPoint point;
    point.x = current_x + x_vals[i] * dir_x;
    point.y = current_y + x_vals[i] * dir_y;
    point.z = current_z + y_vals[i];
    trajectory_points_.push_back(point);
  }

  return best_angle_;
}
io::Command HangingShooter::aim(Eigen::Quaterniond q, io::LocationInfo info,
                                const double bullet_speed) {
  double yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  double pitch;

  if (info.value) {
    if (bullet_speed < 15.0)
      pitch = HangingShooter::calculate_pitch(info.x, info.y, info.z, 15.6);
    else
      pitch =
          HangingShooter::calculate_pitch(info.x, info.y, info.z, bullet_speed);
  } else
    pitch = std::asin(2.0f * (q.w() * q.y() - q.x() * q.z()));
  return {false, false, yaw, pitch, info.x, info.y, info.z, target_x_, target_y_, target_z_};
}
} // namespace auto_aim