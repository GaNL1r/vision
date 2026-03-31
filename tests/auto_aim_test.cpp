#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();
  auto_aim::Plan last_plan;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto planner_start = std::chrono::steady_clock::now();
    auto target = targets.empty() ? std::optional<auto_aim::Target>() : targets.front();
    auto plan = planner.plan(target, 22);

    if (plan.control && std::abs(plan.yaw - last_plan.yaw) * 57.3 < 2) {
      plan.fire = true;
    }

    if (plan.control) last_plan = plan;

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, planner: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(planner_start, tracker_start) * 1e3,
      tools::delta_time(finish, planner_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "plan: control={}, yaw={:.2f}, pitch={:.2f}, fire={}", plan.control, plan.yaw * 57.3,
        plan.pitch * 57.3, plan.fire),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw={:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    if (auto corrector = planner.get_corrector()) {
      auto correction = corrector->get_correction();
      tools::draw_text(
        img,
        fmt::format("correction: yaw={:.2f}, pitch={:.2f}", correction(0) * 57.3, correction(1) * 57.3),
        {10, 120}, {100, 200, 100});
    }

    nlohmann::json data;

    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["plan_yaw"] = plan.yaw * 57.3;
    data["plan_pitch"] = plan.pitch * 57.3;
    data["fire"] = plan.fire;

    if (target.has_value()) {
      std::vector<Eigen::Vector4d> armor_xyza_list = target->armor_xyza_list();

      for (const auto & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head<3>(), xyza[3], target->armor_type, target->name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head<3>(), aim_xyza[3], target->armor_type, target->name);
      if (plan.control) tools::draw_points(img, image_points, {0, 0, 255});

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

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;
  }

  return 0;
}
