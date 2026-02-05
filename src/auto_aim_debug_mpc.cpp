#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/duo.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto yaml = tools::load(config_path);
  int camera_delay = tools::read<int>(yaml, "camera_delay");

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw*57.3;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch*57.3;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw*57.3;
      data["target_pitch"] = plan.target_pitch*57.3;

      data["plan_yaw"] = plan.yaw*57.3;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch*57.3;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        Eigen::VectorXd x = target->ekf_x();
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
        data["last_id"] = target->last_id;
        data["av"] = std::sqrt(tools::square(x[1])+tools::square(x[3])+tools::square(x[5]));

        // 卡方检验数据
        data["residual_yaw"] = target->ekf().data.at("residual_yaw");
        data["residual_pitch"] = target->ekf().data.at("residual_pitch");
        data["residual_distance"] = target->ekf().data.at("residual_distance");
        data["residual_angle"] = target->ekf().data.at("residual_angle");
        data["nis"] = target->ekf().data.at("nis");
        data["nees"] = target->ekf().data.at("nees");
        data["nis_fail"] = target->ekf().data.at("nis_fail");
        data["nees_fail"] = target->ekf().data.at("nees_fail");
        data["recent_nis_failures"] = target->ekf().data.at("recent_nis_failures");
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      data["bs"] = gs.bullet_speed;

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t - std::chrono::nanoseconds(camera_delay));

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    if (!armors.empty()) {
      for (const auto & armor : armors) {
        auto x = armor.xyz_in_world[0];
        auto y = armor.xyz_in_world[1];
        auto z = armor.xyz_in_world[2];
        if (x!=0.0) tools::logger()->debug("x:{:.4f} y:{:.4f} z:{:.4f}", x, y, z);
      }
    }
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
        float armor_x = 0;
        float armor_y = 0;
        for (auto point : image_points) {
          armor_x+= point.x;
          armor_y+= point.y;
        }
        armor_x /= 4;
        armor_y /= 4;
        cv::Point armor_center(armor_x, armor_y);
        tools::draw_point(img, armor_center, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }
    cv::Point2f center (img.cols/2,img.rows/2);
    tools::draw_point(img, center);
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}