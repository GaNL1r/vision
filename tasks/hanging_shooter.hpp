#ifndef HANGING_SHOOTER_HPP
#define HANGING_SHOOTER_HPP
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "io/ros2/ros2.hpp"
#include "io/command.hpp"

namespace auto_aim
{
struct TrajectoryPoint
{
  double x;
  double y;
  double z;
};

struct TrajectoryConfig
{
  bool enable_visualization;
  int trajectory_color_b;
  int trajectory_color_g;
  int trajectory_color_r;
  int circle_thickness;
  int sample_interval;
};

class HangingShooter  //英雄吊射基地处理函数类
{
public:
  HangingShooter(const std::string & config_path);
  io::Command aim(Eigen::Quaterniond q, io::LocationInfo info, const double bullet_speed);
  std::vector<TrajectoryPoint> get_trajectory_points() const { return trajectory_points_; }
  TrajectoryConfig get_trajectory_config() const { return trajectory_config_; }
  double get_best_angle() const { return best_angle_; }

private:
  double g_;
  double m_;
  double BigBulletRadius_;  // 大弹丸半径
  double C_;                // 空气阻力系数
  double rho_;              // 空气密度
  double PI_;
  // 目标在地图坐标系下位置
  double target_x_;
  double target_y_;
  double target_z_;
  io::LocationInfo location_;
  TrajectoryConfig trajectory_config_;
  std::vector<TrajectoryPoint> trajectory_points_;
  double best_angle_;

  double calculate_pitch(double x, double y, double z, double bullet_speed);
  void RungeKutta_4(
    std::vector<double> & x_vals, std::vector<double> & y_vals, double angle, double bullet_speed,
    double height, double dt = 0.01, double max_time = 10);
};

}  // namespace auto_aim
#endif
