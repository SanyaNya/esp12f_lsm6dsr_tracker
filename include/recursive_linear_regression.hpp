#pragma once

#include <cmath>

namespace zt
{

class RecursiveLinearRegression
{
  // Forgetting factor:
  //   1.0   -> no forgetting
  //   0.999 -> fairly slow forgetting
  //   0.99  -> much faster forgetting
  static constexpr double lambda_ = 0.999;

  // RLS parameter vector:
  // theta_[0] = slope
  // theta_[1] = intercept
  double theta_[2]
  {
    1,
    0
  };

  // Covariance matrix P
  //
  //      [ P00 P01 ]
  // P =  [ P10 P11 ]
  //
  // Since P is symmetric, only four values are stored for simplicity.
  double P00_ = 1e12;
  double P01_ = 0;
  double P10_ = 0;
  double P11_ = 1e12;

public:
  constexpr void reset() noexcept
  {
    theta_[0] = 1;
    theta_[1] = 0;

    P00_ = 1e12;
    P01_ = 0;
    P10_ = 0;
    P11_ = 1e12;
  }

  constexpr void update(double x, double y) noexcept
  {
    // Regression vector:
    //
    // phi = [x, 1]
    //
    const double phi0 = x;
    const double phi1 = 1;

    // ------------------------------------------------------------
    // RLS:
    //
    // gamma = lambda + phi^T * P * phi
    //
    // K = P * phi / gamma
    //
    // theta = theta + K * (y - phi^T * theta)
    //
    // P = (P - K * phi^T * P) / lambda
    // ------------------------------------------------------------

    // P * phi
    const double pphi0 =
      P00_ * phi0 +
      P01_ * phi1;

    const double pphi1 =
      P10_ * phi0 +
      P11_ * phi1;

    const double denominator =
      lambda_ +
      phi0 * pphi0 +
      phi1 * pphi1;

    // Innovation / prediction error
    const double prediction =
      theta_[0] * phi0 +
      theta_[1] * phi1;

    const double error = y - prediction;

    // Kalman/RLS gain
    const double inv_denominator =
      double(1) / denominator;

    const double k0 = pphi0 * inv_denominator;
    const double k1 = pphi1 * inv_denominator;

    // Update parameters
    theta_[0] += k0 * error;
    theta_[1] += k1 * error;

    // phi^T * P
    const double phip0 =
      phi0 * P00_ +
      phi1 * P10_;

    const double phip1 =
      phi0 * P01_ +
      phi1 * P11_;

    // Update covariance
    P00_ = (P00_ - k0 * phip0) / lambda_;
    P01_ = (P01_ - k0 * phip1) / lambda_;
    P10_ = (P10_ - k1 * phip0) / lambda_;
    P11_ = (P11_ - k1 * phip1) / lambda_;
  }

  [[nodiscard]] constexpr double slope() const noexcept
  {
    return theta_[0];
  }

  [[nodiscard]] constexpr double intercept() const noexcept
  {
    return theta_[1];
  }

  [[nodiscard]] double predict(double x) const noexcept
  {
    return slope() * x + intercept();
  }
};

} //namespace zt
