#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  send_gimbal_id_ = tools::read<short>(yaml, "send.gimbal");
  send_shoot_id_ = tools::read<short>(yaml, "send.shoot");
  recv_gimbal_id_ = tools::read<short>(yaml, "receive.gimbal");
  recv_shoot_id_ = tools::read<short>(yaml, "receive.shoot");
  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  try {
    srm::message::Packet packet;
    srm::message::GimbalSend g_data;
    g_data.yaw = yaw*180.0/M_PI;
    g_data.pitch = pitch*180.0/M_PI;

    srm::message::ShootSend s_data;
    s_data.fire_flag = fire ? 1 : 0;

    packet.Write(send_gimbal_id_);
    packet.Write(g_data);
    packet.Write(send_shoot_id_);

    short total_size = static_cast<short>(packet.size());
    std::vector<uint8_t> frame;
    frame.resize(sizeof(short) + total_size);
    std::memcpy(frame.data(), &total_size, sizeof(short));
    std::memcpy(frame.data() + sizeof(short), packet.data(), total_size);

    if (control) serial_.write(frame.data(), frame.size());
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 1. 读取 SRM 协议的 2 字节物理帧长度 (short)
    short frame_len = 0;
    // 注意：这里使用底层 serial_.read 而不是原本封装的 read(rx_data_)
    if (serial_.read(reinterpret_cast<uint8_t *>(&frame_len), sizeof(short)) != sizeof(short)) {
      error_count++;
      continue;
    }

    // 长度合法性简单校验（例如 SRM 数据包通常不会超过 256 字节）
    if (frame_len <= 0 || frame_len > 1024) {
      serial_.flushInput();
      continue;
    }

    auto t = std::chrono::steady_clock::now();

    // 2. 读取后续 Payload 数据
    std::vector<char> buffer(frame_len);
    if (serial_.read(reinterpret_cast<uint8_t *>(buffer.data()), frame_len) != (size_t)frame_len) {
      error_count++;
      continue;
    }

    // 3. 将数据装载进 srm::message::Packet 并开始解包
    srm::message::Packet rx_packet;
    // 假设你已按上条建议将 Packet 继承改为 public，或提供了写接口
    rx_packet.assign(buffer.begin(), buffer.end());

    error_count = 0;
    short id = 0;

    // 4. 循环读取 Packet 里的所有 ID 数据包
    while (rx_packet.Read(id)) {
      std::lock_guard<std::mutex> lock(mutex_);

      if (id == recv_gimbal_id_) { // 从 YAML 加载的 receive.gimbal: 1
        srm::message::GimbalReceive g_rx;
        if (rx_packet.Read(g_rx)) {
          // --- 同步到原有的 state_ 成员，保持外部接口不变 ---
          state_.yaw = g_rx.yaw*M_PI/180.0;
          state_.pitch = g_rx.pitch*M_PI/180.0;
          Eigen::Quaterniond q =
            Eigen::AngleAxisd(g_rx.yaw*M_PI/180.0,   Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(g_rx.pitch*M_PI/180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(g_rx.roll*M_PI/180.0,  Eigen::Vector3d::UnitX());
          queue_.push({q, t});
          // state_.yaw_vel = ... (如果 SRM 结构体里有就同步)

          // 映射模式
          switch (g_rx.mode) {
            case 0: mode_ = GimbalMode::IDLE; break;
            case 1: mode_ = GimbalMode::AUTO_AIM; break;
            case 2: mode_ = GimbalMode::SMALL_BUFF; break;
            case 3: mode_ = GimbalMode::BIG_BUFF; break;default: ;
          }
        }
      }
      else if (id == recv_shoot_id_) { // 从 YAML 加载的 receive.shoot: 2
        srm::message::ShootReceive s_rx;
        if (rx_packet.Read(s_rx)) {
          state_.bullet_speed = s_rx.bullet_speed;
        }
      }
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io