#pragma once

#include "types.hpp"

namespace zt
{

struct CalibratedSample
{
  real_t acc[3];
  real_t gyr[3];

  constexpr bool operator==(const CalibratedSample&) const noexcept = default;
};

class Calibration
{
  real_t dts;

  mat3x3_t acc_mat;
  mat3x3_t gyro_mat;
  mat3x3_t K_ga;

  vector4_t b_a;
  vector4_t b_g;

public:
  constexpr Calibration(
    real_t dts,
    const mat3x3_t& K_a,
    const mat3x3_t& R_a,
    const vector4_t& b_a,
    const mat3x3_t& K_g,
    const mat3x3_t& R_g,
    const mat3x3_t& K_ga,
    const vector4_t& b_g) noexcept
  {
    this->dts = dts;

    acc_mat = R_a.inverse() * K_a.inverse();

    gyro_mat = R_g.inverse() * K_g.inverse();

    this->K_ga = K_ga;
    this->b_a = b_a;
    this->b_g = b_g;
  }

  constexpr real_t get_dts() const noexcept
  {
    return dts;
  }

  CalibratedSample apply(vector4_t raw_gyr, vector4_t raw_acc) const noexcept
  {
    vector4_t acc = acc_mat * (raw_acc - b_a);
    vector4_t gyr = gyro_mat * (raw_gyr - ((K_ga * acc) + b_g));

    return
    {
      .acc = {acc.x, acc.y, acc.z},
      .gyr = {gyr.x, gyr.y, gyr.z}
    };
  }
};

} // namespace zt
