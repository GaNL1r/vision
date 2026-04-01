#include "detect_bullet.hpp"
#include <algorithm>

namespace auto_aim
{

constexpr float WEIGHTS[3] = {4.f, 4.f, 2.f};
constexpr uint8_t DIFF_STEP = 5;
constexpr uint8_t DIFF_THRESHOLD = 30;
const cv::Size KERNEL1_SIZE = cv::Size(10, 10);
const cv::Size KERNEL2_SIZE = cv::Size(4, 4);
const cv::Scalar COLOR_LOWB = cv::Scalar(25, 40, 40);
const cv::Scalar COLOR_UPB = cv::Scalar(90, 255, 255);
const cv::Scalar MIN_VUE = cv::Scalar(0, 255 * .1, 255 * .2);

DetectBullet::DetectBullet()
{
  kernel1_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, KERNEL1_SIZE);
  kernel2_ = cv::getStructuringElement(cv::MORPH_CROSS, KERNEL2_SIZE);
}

void DetectBullet::init(const DoReproj & do_reproj) { do_reproj_ = do_reproj; }

bool DetectBullet::test_is_bullet_color(const cv::Vec3b & hsv_col)
{
  return hsv_col[2] > 50 && std::abs(static_cast<int>(hsv_col[0]) - 50) < 10 + 0.5 * std::exp((hsv_col[1] + hsv_col[2]) / 100.0);
}

cv::Mat DetectBullet::get_frame_diff(
  const cv::Mat & s1,
  const cv::Mat & s2,
  const cv::Mat & ref,
  const cv::Mat & lst_fr_bullets)
{
  cv::Mat res = cv::Mat::zeros(s1.rows, s1.cols, CV_8U);

  for (std::size_t y = 0; y < static_cast<std::size_t>(s1.rows); y += DIFF_STEP) {
    for (std::size_t x = 0; x < static_cast<std::size_t>(s1.cols); x += DIFF_STEP) {
      cv::Point p(static_cast<int>(x), static_cast<int>(y));
      if (!ref.at<uint8_t>(p) || (!lst_fr_bullets.empty() && lst_fr_bullets.at<uint8_t>(p))) {
        continue;
      }
      const cv::Vec3b & c1 = s1.at<cv::Vec3b>(p);
      bool flag = true;
      for (int dy = 0; dy < 1 && flag; ++dy) {
        int ty = static_cast<int>(y) + dy;
        if (ty < 0 || ty >= s1.rows) continue;
        for (int dx = 0; dx < 1 && flag; ++dx) {
          int tx = static_cast<int>(x) + dx;
          if (tx < 0 || tx >= s1.cols) continue;
          const cv::Vec3b & c2 = s2.at<cv::Vec3b>(cv::Point(tx, ty));
          uint8_t tmp = static_cast<uint8_t>(
            (WEIGHTS[0] * std::abs(c1[0] - c2[0]) + WEIGHTS[1] * std::abs(c1[1] - c2[1]) + WEIGHTS[2] * std::abs(c1[2] - c2[2])) / (WEIGHTS[0] + WEIGHTS[1] + WEIGHTS[2]));
          if (tmp < DIFF_THRESHOLD) {
            flag = false;
          }
        }
      }
      res.at<uint8_t>(p) = flag ? 255 : 0;
    }
  }

  cv::dilate(res, res, kernel1_);
  if (!lst_fr_bullets.empty()) {
    res |= lst_fr_bullets;
  }
  return res;
}

void DetectBullet::get_possible()
{
  cv::Mat lst_reproj;
  if (!lst_hsv_.empty()) {
    lst_reproj = do_reproj_.reproj(lst_hsv_, lst_fr_q_, cur_fr_q_);
  } else {
    lst_reproj = cv::Mat::zeros(cur_hsv_.size(), cur_hsv_.type());
  }

  cv::Mat res, msk_not_dark;
  cv::inRange(cur_hsv_, COLOR_LOWB, COLOR_UPB, res);
  cv::inRange(cur_hsv_, MIN_VUE, cv::Scalar(255, 255, 255), msk_not_dark);
  res &= msk_not_dark;

  cv::Mat mat_diff = get_frame_diff(cur_hsv_, lst_reproj, res, lst_msk_);
  res &= mat_diff;

  cv::morphologyEx(res, res, cv::MORPH_OPEN, kernel2_);

  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(res, contours_, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
}

void DetectBullet::sort_points(std::vector<cv::Point> & vec)
{
  if (sort_pts_.empty()) {
    sort_pts_ = std::vector<std::vector<uint32_t>>(static_cast<std::size_t>(cur_frame_.cols));
  }

  uint32_t mn_x = static_cast<uint32_t>(cur_frame_.cols), mx_x = 0;
  for (const cv::Point & pt : vec) {
    uint32_t x = static_cast<uint32_t>(pt.x);
    uint32_t y = static_cast<uint32_t>(pt.y);
    sort_pts_[x].push_back(y);
    if (x < mn_x) mn_x = x;
    if (x > mx_x) mx_x = x;
  }

  vec.clear();
  for (uint32_t x = mn_x; x <= mx_x; ++x) {
    std::vector<uint32_t> & vc_x = sort_pts_[x];
    if (vc_x.size() > 10) {
      std::sort(vc_x.begin(), vc_x.end());
    } else {
      for (std::size_t i = 0; i < vc_x.size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
          if (vc_x[j] > vc_x[i]) {
            std::swap(vc_x[i], vc_x[j]);
          }
        }
      }
    }
    for (uint32_t y : vc_x) {
      vec.emplace_back(static_cast<int>(x), static_cast<int>(y));
    }
    vc_x.clear();
  }
}

bool DetectBullet::test_is_bullet(const std::vector<cv::Point> & contour)
{
  std::vector<cv::Point> sorted_contour = contour;
  sort_points(sorted_contour);

  bool flag = false;
  for (std::size_t i = 0, j = 0; i < sorted_contour.size() && !flag; i = j) {
    int x = sorted_contour[i].x;
    while (j < sorted_contour.size() && x == sorted_contour[j].x) {
      ++j;
    }
    for (int y = sorted_contour[i].y; y <= sorted_contour[j - 1].y && !flag; ++y) {
      if (test_is_bullet_color(cur_hsv_.at<cv::Vec3b>(cv::Point(x, y)))) {
        flag = true;
      }
    }
  }
  return flag;
}

void DetectBullet::get_bullets_from_contours()
{
  bullets_.clear();
  lst_msk_ = cv::Mat::zeros(cur_frame_.rows, cur_frame_.cols, CV_8U);

  for (std::size_t i = 0; i < contours_.size(); ++i) {
    const std::vector<cv::Point> & contour = contours_[i];
    cv::RotatedRect rect = cv::minAreaRect(contour);
    cv::Size2f rect_size = rect.size;
    if (rect_size.area() < 30) continue;
    double ratio = cv::contourArea(contour) / rect_size.area();
    if (ratio < 0.5) continue;

    if (test_is_bullet(contour)) {
      bullets_.emplace_back(rect.center, std::min(rect_size.height, rect_size.width) * 0.5f);
      cv::drawContours(lst_msk_, contours_, static_cast<int>(i), 255, cv::FILLED);
    }
  }
}

std::vector<ImageBullet> DetectBullet::process_new_frame(const cv::Mat & new_frame, const Eigen::Quaterniond & q)
{
  lst_hsv_ = cur_hsv_.clone();
  lst_frame_ = cur_frame_.clone();
  cur_frame_ = new_frame.clone();
  lst_fr_q_ = cur_fr_q_;
  cur_fr_q_ = q;

  cv::cvtColor(cur_frame_, cur_hsv_, cv::COLOR_BGR2HSV);

  if (!lst_frame_.empty()) {
    get_possible();
    get_bullets_from_contours();
  }

  return bullets_;
}

}  // namespace auto_aim
