//
// Created by guo on 26-2-27.
//

#ifndef COORD_CONVERTER_HPP
#define COORD_CONVERTER_HPP

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tools/math.hpp"
#include "tools/filter.hpp"

#include "io/gimbal/gimbal.hpp"
#include "io/camera.hpp"

namespace aimer
{
// 系统信息

// 用于复现的瞄准参数
struct ShootParam {
  double v0 = 0.;
  double aim_angle = 0.;
  Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();
  // Eigen::Vector3d target_pos = Eigen::Vector3d();
};

struct AimInfo {
  aimer::math::YpdCoord ypd;
  aimer::math::YpdCoord ypd_v;
  // 若此命令发射，且 ypd = 0，则落点为
  // 若此命令发射，则发射参数为
  aimer::ShootParam shoot_param;
  int info = 0;

  AimInfo(
      const aimer::math::YpdCoord& ypd,
      const aimer::math::YpdCoord& ypd_v,
      const aimer::ShootParam& shoot_param
  ):
      ypd { ypd },
      ypd_v { ypd_v },
      shoot_param { shoot_param }{}
  enum : int { TOP = 1 << 0 };
  static const AimInfo idle() {
    return AimInfo(
        aimer::math::YpdCoord(),
        aimer::math::YpdCoord(),
        aimer::ShootParam()
    );
  }
  // convention: 0 位 (1 << 0)，1 表示在陀螺
};

template<typename T>
class FrameBinder {
public:
    void update(const T& value, const int& frame) {
        this->value = value;
        this->frame = frame;
    }
    T get(const int& frame) const {
        assert(this->frame == frame);
        return value;
    }

private:
    T value = T();
    int frame = 0;
};

class CoordConverter
{
private:
  std::unique_ptr<io::Gimbal> gimbal_;
  std::unique_ptr<io::Camera> camera_;
  // 相机参数和陀螺仪状态
  Eigen::Matrix3d rot_ic_sup; // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
  Eigen::Matrix3d f_mat; // 相机内参矩阵EIGEN-Matrix
  Eigen::Matrix<double, 1, 5> c_mat; // 相机畸变矩阵EIGEN-Matrix
  cv::Mat rot_ic_sup_cv_mat; // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
  cv::Mat f_cv_mat; // 相机内参矩阵CV-Mat
  cv::Mat c_cv_mat; // 相机畸变矩阵CV-Mat
  Eigen::Quaterniond q;
  Eigen::Matrix3d rot_ic_q; // 相机 - 世界坐标系旋转所需矩阵

  cv::Mat img;
  int frame = 0; // 帧计数
  double img_t = 0.; // 相机图像时间

  std::string file_str;

  // from detection_result 我得到 img_timestamp
  // double predict_timestamp = 0.;  // s
  aimer::FrameBinder<double> predict_timestamp_binder;
  // double send_timestamp = 0.;     // s estimate
  // always make reader understand!
  aimer::SingleFilter<1> predict_to_send_latency_filter {};
public:
  CoordConverter(const std::string & config_path)
  {
    gimbal_ = std::make_unique<io::Gimbal>(config_path);
    camera_ = std::make_unique<io::Camera>(config_path);
  }
  /**
   * @brief 把 坐标系 [原点: 相机, 方向: 陀螺仪] 下的坐标转为
   * 坐标系 [原点: 枪口, 方向：陀螺仪] 下的坐标
   */
  Eigen::Vector3d xyz_i_camera_to_xyz_i_barrel(const Eigen::Vector3d& xyz_i_camera) const;
  Eigen::Vector3d xyz_i_barrel_to_xyz_i_camera(const Eigen::Vector3d& xyz_i_barrel) const;

  // 通过相机的陀螺仪姿态，求出陀螺仪坐标系
  // 前提：传入的 pc 所对应的姿态是真的相机姿态
  // 相机 xyz -> 陀螺仪 xyz
  /** @brief xyz Point in Camera coordinate system to xyz Point in World
 * coordinate system */
  Eigen::Vector3d pc_to_pi(const Eigen::Vector3d& pc) const;
  // 陀螺仪 xyz -> 相机 xyz，z朝前
  Eigen::Vector3d pi_to_pc(const Eigen::Vector3d& pi) const;

  // pu 是 camera_pu, pc 是枪口 pc，pi 也是枪口 pi
  // pu 有问题，pu 直接求 pc 是相机 pc
  // 相机（实际上是枪口） xyz -> 图像 xy（中心点）
  // 不矫正畸变
  /** @brief xyz Point in Camera coordinate_system to Point Undistorted */
  cv::Point2f pc_to_pu(const Eigen::Vector3d& pc) const;
  Eigen::Vector3d pu_to_pc_norm(const cv::Point2f& pu) const;
  aimer::math::YpdCoord pu_to_yp_c(const cv::Point2f& pu) const;

  cv::Point2f pu_to_pd(const cv::Point2f& pu) const;
  cv::Point2f pd_to_pu(const cv::Point2f& pd) const;

  cv::Point2f pi_to_pu(const Eigen::Vector3d& pi) const;
  cv::Point2f pi_to_pd(const Eigen::Vector3d& pi) const;
  /** @brief Point Distorted to Yaw Pitch in Camera coordinate system */
  aimer::math::YpdCoord pd_to_yp_c(const cv::Point2f& pd) const;

  // 此为利用相机的坐标系系统所求出的近似值
  aimer::math::YpdCoord
  get_camera_ypd_v(const Eigen::Vector3d& xyz_i, const Eigen::Vector3d& xyz_v_i) const;
  // pts 转标准化 pis
  std::vector<Eigen::Vector3d> pts_to_pis_norm(const std::vector<cv::Point2f>& pts) const;

  cv::Point2f aim_ypd_to_pu(const aimer::math::YpdCoord& ypd) const;

  const cv::Mat& get_f_cv_mat_ref() const {
    return this->f_cv_mat;
  }
  const cv::Mat& get_c_cv_mat_ref() const {
    return this->c_cv_mat;
  }
  const cv::Mat& get_rot_ic_sup_cv_mat_ref() const {
    return this->rot_ic_sup_cv_mat;
  }
  Eigen::Quaterniond get_q() const {
    return this->q;
  }

  double get_img_to_predict_latency() const;
  double get_predict_to_send_latency() const;
  double get_send_to_control_latency() const;
  double get_control_to_fire_latency() const;
  double get_fire_to_hit_latency(const Eigen::Vector3d& aim_xyz_i_barrel) const;
  // 获取预测时间量，推导见 aimer/docs/latency.md
  double get_img_to_prediction_latency(const Eigen::Vector3d& aim_xyz_i_barrel) const;
  double get_prediction_time(const Eigen::Vector3d& aim_xyz_i_barrel) const;
  double get_img_to_hit_latency(const Eigen::Vector3d& aim_xyz_i_barrel) const;
  double get_hit_time(const Eigen::Vector3d& aim_xyz_i_barrel) const;
  double get_img_to_control_latency() const;
  double get_img_to_fire_latency() const;

  // 获取帧计数
  int get_frame() const;
  // 输出图像对应的相机时间
  double get_img_t() const;
  double get_bullet_speed() const;
};

// 无法更新时，可信任时间的倒数器，实现漏帧保护
// 不可信任也不必立即杀死，因此还有留存之功能
class CreditClock {
public:
  CreditClock(aimer::CoordConverter* const converter, const double& credit_time);
  void update();

  bool credit() const;

  double get_update_t() const;

private:
  aimer::CoordConverter* const converter;
  const double credit_time;
  double update_t = 0.;
};

}

#endif //COORD_CONVERTER_HPP
