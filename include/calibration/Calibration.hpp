#pragma once

#include "types.hpp"
#include "TemperatureCalibration.hpp"

namespace zt
{

struct VQFParamsOverride
{
  real_t tauAcc;
  real_t biasForgettingTime;
  real_t biasClip;
  real_t biasSigmaMotion;
  real_t biasVerticalForgettingFactor;
  real_t biasSigmaRest;
  real_t restMinT;
  real_t restFilterTau;
  real_t restThGyr;
  real_t restThAcc;
};

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
  GyroTemperatureCalibration temp_calib_g;

public:
  constexpr Calibration(
    real_t dts,
    const mat3x3_t& K_a,
    const mat3x3_t& R_a,
    const vector4_t& b_a,
    const mat3x3_t& K_g,
    const mat3x3_t& R_g,
    const mat3x3_t& K_ga,
    const GyroTemperatureCalibration& temp_calib_g) noexcept
  {
    this->dts = dts;

    acc_mat = R_a.inverse() * K_a.inverse();

    gyro_mat = R_g.inverse() * K_g.inverse();

    this->K_ga = K_ga;
    this->b_a = b_a;
    this->temp_calib_g = temp_calib_g;
  }

  constexpr real_t get_dts() const noexcept
  {
    return dts;
  }

  CalibratedSample apply(vector4_t raw_gyr, vector4_t raw_acc, real_t temp) const noexcept
  {
    vector4_t acc = acc_mat * (raw_acc - b_a);
    vector4_t gyr = gyro_mat * (raw_gyr - ((K_ga * acc) + temp_calib_g.get(temp)));

    return
    {
      .acc = {acc.x, acc.y, acc.z},
      .gyr = {gyr.x, gyr.y, gyr.z}
    };
  }
};

} // namespace zt
