#pragma once

#include <array>
#include "types.hpp"

namespace zt
{

struct GyroTemperatureCalibration
{
  static constexpr real_t k_t_min = 25.0;
  static constexpr real_t k_t_max = 50.0;
  static constexpr real_t k_t_step = 0.5;
  static constexpr std::size_t k_bucket_count = (k_t_max - k_t_min) / k_t_step + 1;

  vector4_t b_g_ref;
  std::array<vector4_t, k_bucket_count> buckets;

  vector4_t get(real_t temp) const noexcept
  {
    return b_g_ref + get_bias_ref_diff(temp);
  }

  vector4_t get_bias_ref_diff(real_t temp) const noexcept
  {
    const std::size_t i = get_bucket_idx(temp);

    std::size_t left, right;
    if(i > 0 && temp < buckets[i].w_)
    {
      left = i-1;
      right = i;
    }
    else if(i < buckets.size()-1 && temp >= buckets[i].w_)
    {
      left = i;
      right = i + 1;
    }
    else return buckets[i];

    const real_t alpha = (temp - buckets[left].w_) / (buckets[right].w_ - buckets[left].w_);
    return buckets[left] + (buckets[right] - buckets[left]) * alpha;
  }

  static constexpr std::size_t get_bucket_idx(real_t temp) noexcept
  {
    real_t ctemp = std::clamp(temp, k_t_min, k_t_max);

    return (ctemp - k_t_min) / k_t_step;
  }

  static constexpr real_t idx_to_temp(std::size_t idx) noexcept
  {
    return k_t_min + k_t_step * idx;
  }
};

} //namespace zt
