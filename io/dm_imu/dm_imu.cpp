#include "dm_imu.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io
{
DM_IMU::DM_IMU() : queue_(5000)
{
  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[DM_IMU] initialized");
}

DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void DM_IMU::init_serial()
{
  try {
    serial_.setPort("/dev/ttyACM0");
    serial_.setBaudrate(921600);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);  //default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000);  //1s

    tools::logger()->info("[DM_IMU] serial port opened");
  }

  catch (serial::IOException & e) {
    tools::logger()->warn("[DM_IMU] failed to open serial port ");
    exit(0);
  }
}

void DM_IMU::get_imu_data_thread()
{
  const size_t target_size = sizeof(srm::message::GimbalReceive);
  std::vector<uint8_t> buffer(target_size);

  while (!stop_thread_) {
    if (serial_.isOpen() && serial_.available() >= target_size) {
      // 1. 读取原始串口字节
      serial_.read(buffer.data(), target_size);

      // 2. 将数据转存到 stm-message 的 Packet 中
      srm::message::Packet packet;
      packet.insert(packet.end(), buffer.begin(), buffer.end());

      // 3. 使用 Packet 的 Read 接口解析到结构体
      // 这会自动处理字节流到 float/int 的转换
      if (packet.Read(imu_raw_data_)) {
        auto timestamp = std::chrono::steady_clock::now();

        // 4. 保持原有接口：将 RPY 转换为四元数
        // 注意：这里假设 imu_raw_data_.yaw/pitch/roll 是欧拉角
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(imu_raw_data_.yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(imu_raw_data_.pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(imu_raw_data_.roll * M_PI / 180.0, Eigen::Vector3d::UnitX());

        q.normalize();
        queue_.push({q, timestamp});
      }
    } else {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

}  // namespace io
