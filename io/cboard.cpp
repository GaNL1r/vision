#include "cboard.hpp"

#include "message/info.h"
#include "message/packet.h"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
  can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))
// 注意: callback的运行会早于Cboard构造函数的完成
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "cboard_port");
  imu_packet_id_ = tools::read<short>(yaml, "receive.gimbal"); // 对应 GimbalReceive
  status_packet_id_ = tools::read<short>(yaml, "receive.shoot"); // 对应 ShootReceive
  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[CBoard] Serial open failed: {}", e.what());
    exit(1);
  }
  thread_ = std::thread(&CBoard::read_thread, this);
  tools::logger()->info("[Cboard] Waiting for q...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[Cboard] Opened.");
}
void CBoard::read_thread() {
  while (!quit_) {
    try {
      short frame_len = 0;
      if (serial_.read(reinterpret_cast<uint8_t*>(&frame_len), 2) != 2) continue;

      if (frame_len <= 0 || frame_len > 512) continue;

      std::vector<char> buffer(frame_len);
      if (serial_.read(reinterpret_cast<uint8_t*>(buffer.data()), frame_len) != (size_t)frame_len) continue;

      auto t = std::chrono::steady_clock::now();
      srm::message::Packet rx_packet;
      rx_packet.assign(buffer.begin(), buffer.end());

      short id;
      while (rx_packet.Read(id)) {
        if (id == imu_packet_id_) {
          srm::message::GimbalReceive data;
          if (rx_packet.Read(data)) {
            // 解算四元数 (ZYX 顺序: Yaw -> Pitch -> Roll)
            Eigen::Quaterniond q =
                Eigen::AngleAxisd(data.yaw*M_PI/180,   Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(data.pitch*M_PI/180, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(data.roll*M_PI/180,  Eigen::Vector3d::UnitX());

            queue_.push({q.normalized(), t});
          }
        } else if (id == status_packet_id_) {
          srm::message::ShootReceive data;
          if (rx_packet.Read(data)) {
            std::lock_guard<std::mutex> lock(mutex_);
            bullet_speed = data.bullet_speed;
            // mode = Mode(data.mode); // 根据 info.h 实际字段映射
          }
        }
      }
    } catch (...) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}
Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  double ratio = std::chrono::duration<double>(timestamp - t_a).count() /
                 std::chrono::duration<double>(t_b - t_a).count();

  return data_ahead_.q.slerp(std::clamp(ratio, 0.0, 1.0), data_behind_.q).normalized();
}

void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

  try {
    can_.write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

}  // namespace io