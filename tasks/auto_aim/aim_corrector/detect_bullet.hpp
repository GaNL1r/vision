#ifndef AUTO_AIM__AIM_CORRECTOR__DETECT_BULLET_HPP
#define AUTO_AIM__AIM_CORRECTOR__DETECT_BULLET_HPP

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <vector>
#include "do_reproj.hpp"

namespace auto_aim
{

struct ImageBullet
{
  cv::Point2f center;
  float radius;

  ImageBullet() = default;
  ImageBullet(const cv::Point2f & center, float radius) : center(center), radius(radius) {}
};

class DetectBullet
{
public:
  DetectBullet();
  
  void init(const DoReproj & do_reproj);
  
  std::vector<ImageBullet> process_new_frame(const cv::Mat & frame, const Eigen::Quaterniond & q);

  const std::vector<ImageBullet> & get_bullets() const { return bullets_; }

private:
  void get_possible();
  void sort_points(std::vector<cv::Point> & vec);
  bool test_is_bullet(const std::vector<cv::Point> & contour);
  void get_bullets_from_contours();

  cv::Mat lst_frame_;
  cv::Mat cur_frame_;
  Eigen::Quaterniond lst_fr_q_;
  Eigen::Quaterniond cur_fr_q_;
  cv::Mat cur_hsv_;
  cv::Mat lst_hsv_;
  cv::Mat lst_msk_;

  cv::Mat kernel1_;
  cv::Mat kernel2_;

  DoReproj do_reproj_;

  std::vector<std::vector<cv::Point>> contours_;
  std::vector<std::vector<uint32_t>> sort_pts_;

  std::vector<ImageBullet> bullets_;

  static bool test_is_bullet_color(const cv::Vec3b & hsv_col);
  cv::Mat get_frame_diff(
    const cv::Mat & s1,
    const cv::Mat & s2,
    const cv::Mat & ref,
    const cv::Mat & lst_fr_bullets);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIM_CORRECTOR__DETECT_BULLET_HPP
