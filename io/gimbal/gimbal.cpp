#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/termios.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include "tools/yaml.hpp"
namespace srm::message {

#if defined(__APPLE__)
const std::string kSystem = "macos";
#elif defined(__linux__)
const std::string kSystem = "linux";
#endif

/// 串口收发
class Serial2 {
 public:
  Serial2() = default;
  virtual ~Serial2() = default;
  virtual bool Initialize() = 0;
  virtual bool Read(Packet REF_OUT data, short size) = 0;
  virtual bool Write(Packet REF_IN data, short size) = 0;
};

/// 物理串口收发
class PhySerial2 final : public Serial2 {
 public:
  PhySerial2() = default;
  ~PhySerial2() override;
  bool Initialize() override;
  bool Read(srm::message::Packet REF_OUT data, short size) override;
  bool Write(Packet REF_IN data, short size) override;

 private:
  std::string serial_port_;
  int serial_fd_{};
};

/// 虚拟串口收发
class SimSerial2 final : public Serial2 {
 public:
  SimSerial2() = default;
  ~SimSerial2() override = default;
  bool Initialize() override;
  bool Read(Packet REF_OUT data, short size) override;
  bool Write(Packet REF_IN data, short size) override;
};

/// 上下位机通信类
class StmMessage2 final : public BaseMessage {
  inline static auto registry = RegistrySub<BaseMessage, StmMessage2>("stm322");

 public:
  StmMessage2() = default;
  ~StmMessage2() override = default;
  bool Initialize(const std::string&config_path) override;
  bool Connect(bool flag) override;
  bool Send() override;
  bool Receive() override;

 private:
  std::unique_ptr<Serial2> serial_;
};

PhySerial2::~PhySerial2() { close(serial_fd_); }

bool PhySerial2::Initialize() {
  FILE *ls = (kSystem == "macos" ? popen("ls --color=never /dev/cu.usb*", "r")
                                 : popen("ls --color=never /dev/ttyACM*", "r"));
  char name[127];
  auto ret = fscanf(ls, "%s", name);
  pclose(ls);
  if (ret == -1) {
    tools::logger()->error("No UART device found.");
    return false;
  } else {
    serial_port_ = name;
  }

  if (kSystem == "linux" && chmod(serial_port_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
    tools::logger()->error("Running in user mode, manually setting permission is required.");
    tools::logger()->error("$ sudo chmod 777 {}", serial_port_);
  }

  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ == -1) {
    tools::logger()->error("Failed to open serial port {}", serial_port_);
    return false;
  }

  termios termios{};
  tcgetattr(serial_fd_, &termios);
  cfmakeraw(&termios);
  cfsetispeed(&termios, B115200);
  tcsetattr(serial_fd_, TCSANOW, &termios);
  tools::logger()->info("Serial port {} is open." , serial_port_);
  return true;
}

bool PhySerial2::Read(Packet REF_OUT data, short size) {
  using namespace std::chrono_literals;
  std::vector<char> buffer(sizeof(short) + size);
  auto start_time = std::chrono::high_resolution_clock::now();

  char *ptr = buffer.data();
  int cnt = 0;

  while (cnt < sizeof(short)) {
    if (std::chrono::high_resolution_clock::now() - start_time >= 3s) {
      return false;
    }
    int bytes_read = read(serial_fd_, ptr + cnt, sizeof(short) - cnt);
    if (bytes_read > 0) cnt += bytes_read;
    else if (bytes_read <= 0) std::this_thread::sleep_for(1ms);
  }

  short received_size = *(short *)ptr;
  if (size != received_size) {
    tcflush(serial_fd_, TCIFLUSH);
    return false;
  }

  while (cnt < buffer.size()) {
    if (std::chrono::high_resolution_clock::now() - start_time >= 3s) {
      return false;
    }
    int bytes_read = read(serial_fd_, ptr + cnt, buffer.size() - cnt);
    if (bytes_read > 0) cnt += bytes_read;
    else if (bytes_read <= 0) std::this_thread::sleep_for(1ms);
  }

  data.Resize(size);
  memcpy(data.Ptr(), ptr + sizeof(short), size);
  return true;
}

bool PhySerial2::Write(Packet REF_IN data, short size) {
  std::vector<char> buffer(sizeof(short) + size);
  memcpy(buffer.data(), &size, sizeof(short));
  memcpy(buffer.data() + sizeof(short), data.Ptr(), data.Size());

  int cnt = write(serial_fd_, buffer.data(), buffer.size());
  return cnt == buffer.size();
}

extern "C" {
void InitParam2();
static int8_t CdcReceiveFs(uint8_t *Buf, uint32_t *Len);
char buffer2[256];
short buffer_size2 = 0;

// Vision通信使用的数据结构
typedef struct {
  float yaw;
  float pitch;
  float roll;
  int mode;
  int color;
} VisionGimbalReceive;

typedef struct {
  float bullet_speed;
} VisionShootReceive;

typedef struct {
  float yaw;
  float pitch;
} VisionGimbalSend;

typedef struct {
  int fire_flag;
} VisionShootSend;

typedef struct {
  void *ptr_list[32];
  short size_list[32];
} Message;

VisionGimbalReceive vision_gimbal_recv2;
VisionShootReceive vision_shoot_recv2;
VisionGimbalSend vision_gimbal_send2;
VisionShootSend vision_shoot_send2;

Message receive2;
Message send2;
short receive_size2 = 0;
short send_size2 = 0;
short send_id_list2[32];
short send_id_num2 = 0;

void InitParam2(void) {
#define RIGISTER_ID(data, id, packet) \
    data.ptr_list[id] = &(packet);    \
    data.size_list[id] = sizeof(packet);

    // STM32接收vision发来的数据
    RIGISTER_ID(receive2, 1, vision_gimbal_send2);
    RIGISTER_ID(receive2, 2, vision_shoot_send2);

    // STM32发送给vision的数据
    RIGISTER_ID(send2, 1, vision_gimbal_recv2);
    RIGISTER_ID(send2, 2, vision_shoot_recv2);
}

static int8_t CdcReceiveFs(uint8_t *Buf, uint32_t *Len) {
  if (!Buf) return -1;

  short buf_pos = 0;
  char *ptr;

  if (receive_size2 == 0) {
    if (buf_pos + sizeof(short) > 64) return -1;
    memcpy(&receive_size2, Buf + buf_pos, sizeof(short));
    buf_pos += sizeof(short);
    buffer_size2 = 0;
    if (receive_size2 <= 0 || receive_size2 > 1024) {
      receive_size2 = 0;
      return -1;
    }
  }

  short remain_size = receive_size2 - buffer_size2;
  if (remain_size > 0) {
    short copy_size = (remain_size > 64 - buf_pos) ? (64 - buf_pos) : remain_size;
    if (buffer_size2 + copy_size > sizeof(buffer2)) {
      receive_size2 = 0;
      buffer_size2 = 0;
      return -1;
    }
    memcpy(buffer2 + buffer_size2, Buf + buf_pos, copy_size);
    buffer_size2 += copy_size;
  }

  if (receive_size2 != buffer_size2) return 0;

  ptr = buffer2;
  while (ptr < buffer2 + buffer_size2) {
    if (ptr + sizeof(short) > buffer2 + buffer_size2) break;

    short id;
    memcpy(&id, ptr, sizeof(short));
    ptr += sizeof(short);

    if (id == -1) {
      receive_size2 = 0;
      return 0;
    } else if (id == 0) {
      send_id_num2 = 0;
      send_size2 = 0;
      while (ptr < buffer2 + buffer_size2) {
        if (ptr + sizeof(short) > buffer2 + buffer_size2) break;
        memcpy(&id, ptr, sizeof(short));
        ptr += sizeof(short);
        if (id < 0 || id >= 32) continue;
        send_id_list2[send_id_num2++] = id;
        send_size2 += send2.size_list[id] + sizeof(short);
        if (send_id_num2 >= 32) break;
      }
    } else {
      if (id < 0 || id >= 32) break;
      if (ptr + receive2.size_list[id] > buffer2 + buffer_size2) break;
      memcpy(receive2.ptr_list[id], ptr, receive2.size_list[id]);
      ptr += receive2.size_list[id];
    }
  }

  buffer_size2 = sizeof(short) + send_size2;
  if (buffer_size2 > sizeof(buffer2)) {
    receive_size2 = 0;
    return -1;
  }

  ptr = buffer2;
  memcpy(ptr, &send_size2, sizeof(short));
  ptr += sizeof(short);

  for (int i = 0; i < send_id_num2 && i < 32; i++) {
    short id = send_id_list2[i];
    if (id < 0 || id >= 32 || !send2.ptr_list[id]) continue;
    if (ptr + sizeof(short) + send2.size_list[id] > buffer2 + sizeof(buffer2)) break;
    memcpy(ptr, &id, sizeof(short));
    ptr += sizeof(short);
    memcpy(ptr, send2.ptr_list[id], send2.size_list[id]);
    ptr += send2.size_list[id];
  }

  receive_size2 = 0;
  return 0;
}
}

bool SimSerial2::Initialize() {
  InitParam2();
  return true;
}

bool SimSerial2::Read(Packet REF_OUT data, short size) {
  if (buffer_size2 < sizeof(short)) return false;
  short received_size = *(short *)buffer2;
  if (received_size != size) return false;
  if (buffer_size2 < sizeof(short) + size) return false;

  data.Resize(size);
  memcpy(data.Ptr(), buffer2 + sizeof(short), size);
  return true;
}

bool SimSerial2::Write(Packet REF_IN data, short size) {
  std::vector<char> buffer;
  buffer.insert(buffer.begin(), (char *)&size, (char *)&size + sizeof(short));

  auto data_clone = data;
  while (!data_clone.Empty()) {
    char c;
    if (!data_clone.Read(c)) break;
    buffer.push_back(c);
    if (buffer.size() >= 64) {
      CdcReceiveFs((uint8_t *)buffer.data(), nullptr);
      buffer.clear();
    }
  }
  if (!buffer.empty()) {
    CdcReceiveFs((uint8_t *)buffer.data(), nullptr);
  }
  return true;
}

bool StmMessage2::Initialize(const std::string & config_path) {
  auto yaml = tools::load(config_path);
  auto type = tools::read<std::string>(yaml,"serial_type");
  type == "physical" ? serial_ = std::make_unique<PhySerial2>() : serial_ = std::make_unique<SimSerial2>();
  if (!serial_ || !serial_->Initialize()) {
    tools::logger()->error("Failed to initialize serial.");
    return false;
  }
  return true;
}

bool StmMessage2::Connect(bool flag) {
  if (flag) {
    send_buffer_.Write((short)0);
    for (auto &[_, it] : receive_registry_) {
      send_buffer_.Write(it.first);
    }
  } else {
    send_buffer_.Write((short)-1);
  }
  if (!serial_->Write(send_buffer_, send_buffer_.Size())) {
    send_buffer_.Clear();
    tools::logger()->error("Failed to establish connection.");
    return false;
  }
  send_buffer_.Clear();
  return true;
}

bool StmMessage2::Send() {
  if (!serial_->Write(send_buffer_, send_size_)) {
    tools::logger()->error("Failed to send data to serial.");
    send_buffer_.Clear();
    return false;
  }
  send_buffer_.Clear();
  return true;
}

bool StmMessage2::Receive() {
  if (!serial_->Read(receive_buffer_, receive_size_)) {
    tools::logger()->warn( "Failed to read data from serial.");
    receive_buffer_.Clear();
    return false;
  }

  char *ptr = receive_buffer_.Ptr();
  while (ptr < receive_buffer_.Ptr() + receive_buffer_.Size()) {
    short id = *(short *)ptr;
    ptr += sizeof(short);
    auto &packet = packet_received_[id];
    memcpy(packet.Ptr(), ptr, packet.Size());
    ptr += packet.Size();
  }
  return true;
}

}  // namespace srm::message
namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    message_.reset(srm::message::CreateMessage("stm322"));
    if (!message_) {
      tools::logger()->error("Failed to create message");
      return ;
    }
    if (!message_->Initialize(config_path)) {
      tools::logger()->error("Failed to initialize message");
      return ;
    }
    message_->ReceiveRegister<srm::message::GimbalReceive>(tools::read<short>(yaml, "receive.gimbal"));
    message_->ReceiveRegister<srm::message::ShootReceive>(tools::read<short>(yaml, "receive.shoot"));
    message_->SendRegister<srm::message::GimbalSend>(tools::read<short>(yaml, "send.gimbal"));
    message_->SendRegister<srm::message::ShootSend>(tools::read<short>(yaml, "send.shoot"));
    message_->Connect(true);
    receive_packet = new srm::message::ReiceivePacket();
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
  message_->Connect(false);
  //serial_.close();
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
  const srm::message::GimbalSend gimbal_send{VisionToGimbal.yaw, VisionToGimbal.pitch};
  const srm::message::ShootSend shoot_send{VisionToGimbal.mode};
  try {
    message_->WriteData(gimbal_send);
    message_->WriteData(shoot_send);
    message_->Send();
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  const srm::message::GimbalSend gimbal_send{yaw, pitch};
  tools::logger()->info("send:yaw={:1f},pitch={:1f} Receive:yaw={:1f},ptich={:1f}",yaw, pitch,gimbal_receive.yaw, gimbal_receive.pitch);
  const srm::message::ShootSend shoot_send{control ? (fire ? 2 : 1) : 0};

  try {
    message_->WriteData(gimbal_send);
    message_->WriteData(shoot_send);
    message_->Send();
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  return true;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  mode_ = GimbalMode::AUTO_AIM;

  while (!quit_) {
    auto t = std::chrono::steady_clock::now();
    message_->Receive();
    message_->ReadData(gimbal_receive);
    message_->ReadData(shoot_receive);
    receive_packet->yaw = gimbal_receive.yaw;
    receive_packet->pitch = gimbal_receive.pitch;
    receive_packet->roll = gimbal_receive.roll;
    receive_packet->mode = gimbal_receive.mode;
    receive_packet->color = gimbal_receive.color;
    receive_packet->bullet_speed = shoot_receive.bullet_speed;
    auto yaw_ = receive_packet->yaw * M_PI / 180;
    auto pitch_ = receive_packet->pitch * M_PI / 180;
    auto roll_ = receive_packet->roll * M_PI / 180;
    Eigen::Vector3d ypr(yaw_, pitch_, roll_);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
    //推入数据队列
    q.normalize();
    queue_.push({q, t});
    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = static_cast<float>(yaw_);
    state_.pitch = static_cast<float>(pitch_);
    state_.bullet_speed = shoot_receive.bullet_speed;

  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

}  // namespace io
