#include <fmt/core.h>

#include <Eigen/Core>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/hanging_shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

class TrajectoryVisualizer { // 轨迹可视化类
public:
  TrajectoryVisualizer(const std::string &config_path) {
    auto yaml = YAML::LoadFile(config_path);

    auto R_gimbal2imubody_data =
        yaml["R_gimbal2imubody"].as<std::vector<double>>();
    auto R_camera2gimbal_data =
        yaml["R_camera2gimbal"].as<std::vector<double>>();
    auto t_camera2gimbal_data =
        yaml["t_camera2gimbal"].as<std::vector<double>>();
    R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(
        R_gimbal2imubody_data.data());
    R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(
        R_camera2gimbal_data.data());
    t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(
        camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
    cv::eigen2cv(camera_matrix, camera_matrix_);
    cv::eigen2cv(distort_coeffs, distort_coeffs_);

    BigBulletRadius_ = yaml["BigBulletRadius"].as<double>();
    image_width_ = yaml["image_width"].as<int>();
    image_height_ = yaml["image_height"].as<int>();
  }

  void set_R_gimbal2world(const Eigen::Quaterniond &q) {
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ =
        R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
  }

  std::vector<cv::Point2f>
  project_trajectory( // 轨迹点投影，返回图像坐标系下的点
      const std::vector<auto_aim::TrajectoryPoint> &trajectory_points) {
    std::vector<cv::Point2f> image_points;

    for (const auto &point : trajectory_points) {
      Eigen::Vector3d xyz_in_world(point.x, point.y, point.z);
      Eigen::Vector3d xyz_in_gimbal =
          R_gimbal2world_.transpose() * xyz_in_world;
      Eigen::Vector3d xyz_in_camera =
          R_camera2gimbal_.transpose() * (xyz_in_gimbal - t_camera2gimbal_);

      if (xyz_in_camera[2] <= 0.1)
        continue;

      std::vector<cv::Point3f> object_points = {cv::Point3f(0, 0, 0)};
      std::vector<cv::Point2f> projected_points;

      cv::Vec3d rvec(0, 0, 0);
      cv::Vec3d tvec(xyz_in_camera[0], xyz_in_camera[1], xyz_in_camera[2]);
      cv::projectPoints(object_points, rvec, tvec, camera_matrix_,
                        distort_coeffs_, projected_points);

      if (!projected_points.empty()) {
        image_points.push_back(projected_points[0]);
      }
    }

    return image_points;
  }

  int calculate_circle_radius(
      double z_in_camera) { // 计算可视化绘制的圆圈的半径
    if (z_in_camera <= 0.1)
      return 2;
    double focal_length = camera_matrix_.at<double>(0, 0);
    int radius =
        static_cast<int>(BigBulletRadius_ * focal_length / z_in_camera);
    return std::max(2, std::min(radius, 20));
  }

  void draw_trajectory(cv::Mat &img,
                       const std::vector<cv::Point2f> &image_points,
                       const auto_aim::TrajectoryConfig
                           &config) { // 遍历投影后的轨迹点，绘制吊射弹道
    if (!config.enable_visualization || image_points.empty())
      return;

    cv::Scalar color(config.trajectory_color_b, config.trajectory_color_g,
                     config.trajectory_color_r);

    for (size_t i = 0; i < image_points.size(); ++i) {
      int radius = 3 + static_cast<int>(i * 0.1);
      radius = std::min(radius, 10);
      cv::circle(img, image_points[i], radius, color, config.circle_thickness);
    }
  }

private:
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  double BigBulletRadius_;
  int image_width_;
  int image_height_;
};

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{@config-path   | configs/hero.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char *argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::ROS2 ros2;

  auto_aim::HangingShooter hangingshooter(config_path);
  TrajectoryVisualizer visualizer(config_path);

  cv::Mat img;
  cv::Mat img_with_trajectory;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  io::LocationInfo current_location_info; // 英雄当前位置信息

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto yaml = YAML::LoadFile(config_path);

  bool hanging_simulate = yaml["hanging_simulate"].as<bool>();
  bool show_visualization =
      yaml["trajectory_enable_visualization"].as<bool>(false);

  if (hanging_simulate) {
    current_location_info.x = yaml["current_x"].as<double>();
    current_location_info.y = yaml["current_y"].as<double>();
    current_location_info.z = yaml["current_z"].as<double>();
    current_location_info.value = true;
  } else {
    current_location_info.x = 0.0;
    current_location_info.y = 0.0;
    current_location_info.z = 0.0;
    current_location_info.value = false;
  }

  if (show_visualization) {
    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::namedWindow("With Trajectory", cv::WINDOW_NORMAL);
  }

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    io::Command command =
        hangingshooter.aim(q, ros2.subscribe(), cboard.bullet_speed);

    cboard.send(command);

    if (show_visualization && !img.empty()) {
      img.copyTo(img_with_trajectory);

      visualizer.set_R_gimbal2world(q);

      auto trajectory_points = hangingshooter.get_trajectory_points();
      auto image_points = visualizer.project_trajectory(trajectory_points);

      auto config = hangingshooter.get_trajectory_config();
      visualizer.draw_trajectory(img_with_trajectory, image_points, config);

      cv::imshow("Original", img);
      cv::imshow("With Trajectory", img_with_trajectory);
      cv::waitKey(1);
    }
  }

  return 0;
}