#ifndef AUTO_AIM__AIM_CORRECTOR__SINGLE_FILTER_HPP
#define AUTO_AIM__AIM_CORRECTOR__SINGLE_FILTER_HPP

#include <Eigen/Dense>
#include <vector>

namespace auto_aim
{

template <int ORDER>
class SingleFilter
{
public:
  using State = Eigen::Matrix<double, ORDER, 1>;

  SingleFilter() = default;

  void init_x(const State & x)
  {
    x_ = x;
    P_ = State::Ones().asDiagonal();
    initialized_ = true;
  }

  void set_x(const State & x) { x_ = x; }

  void set_t(double t) { t_ = t; }

  State get_x() const { return x_; }

  State predict(double t) const
  {
    double dt = t - t_;
    State result = x_;
    for (int i = 1; i < ORDER; ++i) {
      double coeff = 1.0;
      for (int j = 1; j <= i; ++j) {
        coeff *= dt / j;
      }
      for (int k = 0; k < ORDER - i; ++k) {
        result(k) += coeff * x_(k + i);
      }
    }
    return result;
  }

  void update(double measurement, double t, const std::vector<double> & q_vec, const std::vector<double> & r_vec)
  {
    double dt = t - t_;
    if (dt < 0) dt = 0.001;

    State x_pred = predict(t);

    Eigen::Matrix<double, ORDER, ORDER> F = State::Ones().asDiagonal();
    for (int i = 1; i < ORDER; ++i) {
      for (int j = 0; j < ORDER - i; ++j) {
        double coeff = 1.0;
        for (int k = 1; k <= i; ++k) {
          coeff *= dt / k;
        }
        F(j, j + i) = coeff;
      }
    }

    Eigen::Matrix<double, ORDER, ORDER> Q = Eigen::Matrix<double, ORDER, ORDER>::Zero();
    for (int i = 0; i < ORDER && i < static_cast<int>(q_vec.size()); ++i) {
      Q(i, i) = q_vec[i];
    }

    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = r_vec.empty() ? 1.0 : r_vec[0];

    Eigen::Matrix<double, 1, ORDER> H = Eigen::Matrix<double, 1, ORDER>::Zero();
    H(0, 0) = 1.0;

    State x_p = F * x_;
    Eigen::Matrix<double, ORDER, ORDER> P_p = F * P_ * F.transpose() + Q;

    Eigen::Matrix<double, ORDER, 1> y;
    y(0) = measurement - H(0, 0) * x_p(0);

    Eigen::Matrix<double, 1, 1> S = H * P_p * H.transpose() + R;
    Eigen::Matrix<double, ORDER, 1> K = P_p * H.transpose() * S.inverse();

    x_ = x_p + K * y;
    P_ = (Eigen::Matrix<double, ORDER, ORDER>::Identity() - K * H) * P_p;
    t_ = t;

    if (!initialized_) {
      initialized_ = true;
    }
  }

  bool is_initialized() const { return initialized_; }

private:
  State x_ = State::Zero();
  Eigen::Matrix<double, ORDER, ORDER> P_ = Eigen::Matrix<double, ORDER, ORDER>::Identity();
  double t_ = 0.0;
  bool initialized_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIM_CORRECTOR__SINGLE_FILTER_HPP