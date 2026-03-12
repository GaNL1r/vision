#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "io/ros2/ros2.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/hanging_shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
using namespace std::chrono;
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/hero.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])  // 测试已对准目标情况下吊射效果，example.yaml文件中修改目标位置
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  io::ROS2 ros2;
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto_aim::HangingShooter hangingshooter(config_path);
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    q = cboard.imu_at(t - 1ms);
    io::Command command = hangingshooter.aim(q,ros2.subscribe(),cboard.bullet_speed);
    cboard.send(command);
  }
  return 0;
}