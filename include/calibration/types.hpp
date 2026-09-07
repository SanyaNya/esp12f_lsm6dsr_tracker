#pragma once

#include <chrono>
#include "math.hpp"

namespace zt
{

using real_t = double;
using euler_angles_t = euler_angles<real_t>;
using axis_angle_t = axis_angle<real_t>;
using quat_t = quat<real_t>;
using mat3x3_t = mat3x3<real_t>;
using vector4_t = vec<real_t>;
using qv_t = qv<real_t>;
using clck_t = std::chrono::high_resolution_clock;

constexpr real_t operator""_r(long double r) noexcept { return real_t(r); }

} //namespace zt
