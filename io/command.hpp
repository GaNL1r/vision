#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  // --- 新增 ---
  double self_xyz[3];
  double target_xyz[3];
};

}  // namespace io

#endif  // IO__COMMAND_HPP