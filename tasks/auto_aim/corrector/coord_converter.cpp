//
// Created by guo on 26-2-27.
//

#include "coord_converter.hpp"

#include <ceres/types.h>
#include <fmt/format.h>

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>

namespace aimer {

const double MIN_BULLET_SPEED = 10.; // m/s
const double MIN_IMG_TO_PREDICT_LATENCY = 0.001;
const double MAX_IMG_TO_PREDICT_LATENCY = 0.050;
// 测试中，显示为 13~17 ms
const double DEFAULT_IMG_TO_PREDICT_LATENCY = 0.015;

// 通过相机的陀螺仪姿态，求出陀螺仪坐标系
// 前提：传入的 pc 所对应的姿态是真的相机姿态
// 相机 xyz -> 陀螺仪 xyz
Eigen::Vector3d CoordConverter::pc_to_pi(const Eigen::Vector3d& pc) const {
  auto rot_ci = (this->rot_ic_sup * this->rot_ic_q).transpose();
  // R_IW 来自 q，表示旋转
  // R_CI 是标定得到的 3 * 3 矩阵
  return rot_ci * pc;
}

// 陀螺仪 xyz -> 相机 xyz，z朝前
Eigen::Vector3d CoordConverter::pi_to_pc(const Eigen::Vector3d& pi) const {
  auto rot_ic = this->rot_ic_sup * this->rot_ic_q;
  // R_IW 居然是 WORLD_TO_IMU
  return rot_ic * pi;
}

// pu 是 camera_pu, pc 是枪口 pc，pi 也是枪口 pi
// pu 有问题，pu 直接求 pc 是相机 pc
// 相机（实际上是枪口） xyz -> 图像 xy（中心点）
// 不矫正畸变
cv::Point2f CoordConverter::pc_to_pu(const Eigen::Vector3d& pc) const {
  Eigen::Vector3d pu_eigen = this->f_mat * pc / pc(2, 0);
  return cv::Point2f { (float)pu_eigen(0, 0), (float)pu_eigen(1, 0) };
}

Eigen::Vector3d CoordConverter::pu_to_pc_norm(const cv::Point2f& pu) const {
  Eigen::Vector3d pu_vec { pu.x, pu.y, 1. };
  Eigen::Vector3d pc_norm = this->f_mat.inverse() * pu_vec;
  return pc_norm;
}

aimer::math::YpdCoord CoordConverter::pu_to_yp_c(const cv::Point2f& pu) const {
  return aimer::math::camera_xyz_to_ypd(this->pu_to_pc_norm(pu));
}

cv::Point2f CoordConverter::pu_to_pd(const cv::Point2f& pu) const {
  std::vector<cv::Point2f> pus = { pu };
  std::vector<cv::Point2f> pds = {};
  aimer::math::distort_points(pus, pds, this->f_cv_mat, this->c_cv_mat);
  return pds[0];
}

cv::Point2f CoordConverter::pd_to_pu(const cv::Point2f& pd) const {
  std::vector<cv::Point2f> pds = { pd };
  std::vector<cv::Point2f> pus = {};
  cv::undistortPoints(pds, pus, this->f_cv_mat, this->c_cv_mat, cv::noArray(), this->f_cv_mat);
  return pus[0];
}

cv::Point2f CoordConverter::pi_to_pu(const Eigen::Vector3d& pi) const {
  return this->pc_to_pu(this->pi_to_pc(pi));
}

cv::Point2f CoordConverter::pi_to_pd(const Eigen::Vector3d& pi) const {
  return this->pu_to_pd(this->pc_to_pu(this->pi_to_pc(pi)));
}

aimer::math::YpdCoord CoordConverter::pd_to_yp_c(const cv::Point2f& pd) const {
  return this->pu_to_yp_c(this->pd_to_pu(pd));
}

Eigen::Vector3d CoordConverter::xyz_i_camera_to_xyz_i_barrel(const Eigen::Vector3d& xyz_i_camera
) const {
  Eigen::Vector3d xyz_c_camera = this->pi_to_pc(xyz_i_camera);
  // 注意是目标坐标，平移方向与相机平移到枪口相反
  Eigen::Vector3d xyz_c_barrel = {
    xyz_c_camera(0, 0) - base::get_param<double>("launching-mechanism.camera-to-barrel-x"),
    xyz_c_camera(1, 0) - base::get_param<double>("launching-mechanism.camera-to-barrel-y"),
    xyz_c_camera(2, 0)
};
  // 以枪口为原点的世界坐标系
  Eigen::Vector3d xyz_i_barrel = this->pc_to_pi(xyz_c_barrel);
  return xyz_i_barrel;
}

Eigen::Vector3d CoordConverter::xyz_i_barrel_to_xyz_i_camera(const Eigen::Vector3d& xyz_i_barrel
) const {
  const Eigen::Vector3d xyz_c_barrel = this->pi_to_pc(xyz_i_barrel);
  const Eigen::Vector3d xyz_c_camera = { Eigen::Vector3d(
      xyz_c_barrel(0, 0) + base::get_param<double>("launching-mechanism.camera-to-barrel-x"),
      xyz_c_barrel(1, 0) + base::get_param<double>("launching-mechanism.camera-to-barrel-y"),
      xyz_c_barrel(2, 0)
  ) };
  const Eigen::Vector3d xyz_i_camera = { this->pc_to_pi(xyz_c_camera) };
  return xyz_i_camera;
}

double CoordConverter::get_img_to_predict_latency() const {
    double latency = this->predict_timestamp_binder.get(this->get_frame()) - this->img_t;
    return aimer::math::clamp_default(
        latency,
        aimer::MIN_IMG_TO_PREDICT_LATENCY,
        aimer::MAX_IMG_TO_PREDICT_LATENCY,
        aimer::DEFAULT_IMG_TO_PREDICT_LATENCY
    );
}

double CoordConverter::get_predict_to_send_latency() const { // estimated
    return this->predict_to_send_latency_filter.predict(this->get_img_t())(0, 0);
}

double CoordConverter::get_send_to_control_latency() const {
    return base::get_param<double>("auto-aim.latency.send-to-control");
}

double CoordConverter::get_control_to_fire_latency() const {
    // double latency =
    //     double(int(this->get_robot_status().latency_cmd_to_fire)) / 1e3;
    // return aimer::math::in_range(
    //     latency, aimer::MIN_CONTROL_TO_FIRE_LATENCY,
    //     aimer::MAX_CONTROL_TO_FIRE_LATENCY,
    //     base::get_param<double>("DEFAULT_CONTROL_TO_FIRE_LATENCY"));
    return base::get_param<double>("auto-aim.latency.control-to-fire");
}

double CoordConverter::get_fire_to_hit_latency(const Eigen::Vector3d& aim_xyz_i_barrel) const {
    double bs = this->get_bullet_speed();
    // 不考虑空气阻力
    return aim_xyz_i_barrel.norm() / bs;
}

double CoordConverter::get_img_to_prediction_latency(const Eigen::Vector3d& aim_xyz_i_barrel
) const {
    return this->get_img_to_predict_latency() + this->get_predict_to_send_latency()
        + this->get_send_to_control_latency() + this->get_fire_to_hit_latency(aim_xyz_i_barrel);
}

// yaw_v 和 原始 yaw 均采用该时间点的信息
double CoordConverter::get_prediction_time(const Eigen::Vector3d& aim_xyz_i_barrel) const {
    return this->get_img_t() + this->get_img_to_prediction_latency(aim_xyz_i_barrel);
}

// 此刻给出发弹指令，目标为 aim_pos，则该指令对应子弹的命中延迟
double CoordConverter::get_img_to_hit_latency(const Eigen::Vector3d& aim_xyz_i_barrel) const {
    return this->get_img_to_predict_latency() + this->get_predict_to_send_latency()
        + this->get_send_to_control_latency() + this->get_control_to_fire_latency()
        + this->get_fire_to_hit_latency(aim_xyz_i_barrel);
}

// 此刻给出发弹指令，目标为 aim_pos，则该指令对应子弹的命中时间
double CoordConverter::get_hit_time(const Eigen::Vector3d& aim_xyz_i_barrel) const {
    return this->get_img_t() + this->get_img_to_hit_latency(aim_xyz_i_barrel);
}

double CoordConverter::get_img_to_control_latency() const {
    return this->get_img_to_predict_latency() + this->get_predict_to_send_latency()
        + this->get_send_to_control_latency();
}

double CoordConverter::get_img_to_fire_latency() const {
    return this->get_img_to_predict_latency() + this->get_predict_to_send_latency()
        + this->get_send_to_control_latency() + this->get_control_to_fire_latency();
}

// 获取帧计数
int CoordConverter::get_frame() const {
  return this->frame;
}
// 输出图像对应的相机时间
double CoordConverter::get_img_t() const {
  return this->img_t;
}
double CoordConverter::get_bullet_speed() const {
  return std::max(double(this->get_robot_status_ref().bullet_speed), aimer::MIN_BULLET_SPEED);
}
} // namespace aimer
