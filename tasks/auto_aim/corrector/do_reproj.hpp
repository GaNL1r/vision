//
// Created by guo on 26-2-27.
//

#ifndef DO_REPROJ_HPP
#define DO_REPROJ_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
namespace aimer::aim {
class DoReproj {
  using Quat = Eigen::Quaternion<double>;

  Eigen::Matrix4d cam;
  Eigen::Matrix3d imu;

  // from quaternion to transfrom matrix
  Eigen::Matrix4d from_q_get_trans_mat(const Quat& q);
  // get transform matrix between two frames
  Eigen::Matrix3d get_fr_trans_mat(const Quat& q1, const Quat& q2);

public:
  DoReproj();
  DoReproj(const cv::Mat& cam, const cv::Mat& imu);
  void init(const cv::Mat& cam, const cv::Mat& imu);

  cv::Mat reproj(const cv::Mat& src, const Quat& q1, const Quat& q2);
};
} // namespace auto_aim::corrector

#endif //DO_REPROJ_HPP
