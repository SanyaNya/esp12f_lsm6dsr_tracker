#pragma once

// Convention
//
// Coordinate frame
// +X - Right
// +Y - Up
// -Z - Forward
//
// Quaternion multiplication
// q = q_global * q_local
//
// Matrix
// column vector


//#include <cassert>
#include <concepts>
#include <cmath>
#include <numbers>
#include <limits>
#include <algorithm>

namespace zt
{

template<std::floating_point T>
struct vec
{
  T x, y, z, w_;

  constexpr vec() noexcept :
    x(0.0), y(0.0), z(0.0), w_(0.0) {}

  constexpr vec(T x_, T y_, T z_, T w__) :
    x(x_), y(y_), z(z_), w_(w__) {}

  constexpr vec(T x_, T y_, T z_) :
    x(x_), y(y_), z(z_), w_(0.0) {}

  constexpr T norm() const noexcept
  {
    return std::sqrt(x*x + y*y + z*z);
  }

  constexpr vec normalize() const noexcept
  {
    T n = norm();
    //assert(n > 0);
    return {x/n, y/n, z/n};
  }

  constexpr bool is_normalized(T threshold = 0.00001) const noexcept
  {
    return std::fabs(norm() - T(1.0)) <= threshold;
  }

  constexpr vec operator-() const noexcept
  {
    return {-x, -y, -z};
  }

  constexpr vec operator+=(const vec& v) noexcept
  {
    x += v.x;
    y += v.y;
    z += v.z;

    return *this;
  }

  constexpr vec operator-=(const vec& v) noexcept
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;

    return *this;
  }

  constexpr vec operator*=(const T& t) noexcept
  {
    x *= t;
    y *= t;
    z *= t;

    return *this;
  }

  constexpr vec operator/=(const T& t) noexcept
  {
    x /= t;
    y /= t;
    z /= t;

    return *this;
  }

  friend constexpr vec operator*(const vec& v, const T& t) noexcept
  {
    return {v.x*t, v.y*t, v.z*t};
  }
  friend constexpr vec operator*(const T& t, const vec& v) noexcept
  {
    return {v.x*t, v.y*t, v.z*t};
  }

  friend constexpr vec operator/(const vec& v, const T& t) noexcept
  {
    return {v.x/t, v.y/t, v.z/t};
  }
  friend constexpr vec operator/(const T& t, const vec& v) noexcept
  {
    return {t/v.x, t/v.y, t/v.z};
  }

  friend constexpr vec operator+(const vec& v1, const vec& v2) noexcept
  {
    return {v1.x+v2.x, v1.y+v2.y, v1.z+v2.z};
  }
  friend constexpr vec operator-(const vec& v1, const vec& v2) noexcept
  {
    return {v1.x-v2.x, v1.y-v2.y, v1.z-v2.z};
  }
};

template<std::floating_point T>
struct euler_angles{ T yaw, pitch, roll; };

template<std::floating_point T>
struct axis_angle { vec<T> axis; T angle; };

template<std::floating_point T>
struct quat
{
  T x, y, z, w;

  constexpr quat() noexcept :
    x(0.0), y(0.0), z(0.0), w(1.0) {}

  constexpr quat(T x_, T y_, T z_, T w_) :
    x(x_), y(y_), z(z_), w(w_) {}

  constexpr quat(const quat&) noexcept = default;

  constexpr quat(const euler_angles<T>& e) noexcept
  {
    T hy = e.yaw   * T(0.5);
    T hp = e.pitch * T(0.5);
    T hr = e.roll  * T(0.5);

    T cy = std::cos(hy);
    T sy = std::sin(hy);

    T cp = std::cos(hp);
    T sp = std::sin(hp);

    T cr = std::cos(hr);
    T sr = std::sin(hr);

    x = cy * sp * cr + sy * cp * sr;
    y = sy * cp * cr - cy * sp * sr;
    z = cy * cp * sr - sy * sp * cr;
    w = cy * cp * cr + sy * sp * sr;
  }

  constexpr quat(const axis_angle<T>& aa) noexcept
  {
    T half = T(0.5) * aa.angle;
    T s = std::sin(half);

    x = aa.axis.x * s;
    y = aa.axis.y * s;
    z = aa.axis.z * s;
    w = std::cos(half);
  }

  constexpr T norm() const noexcept
  {
    return std::sqrt(x*x + y*y + z*z + w*w);
  }

  constexpr quat normalize() const noexcept
  {
    T n = norm();
    //assert(n > 0);
    return {x/n, y/n, z/n, w/n};
  }

  constexpr bool is_normalized(T threshold = 0.00001) const noexcept
  {
    return std::fabs(norm() - T(1.0)) <= threshold;
  }

  constexpr quat inverse() const noexcept
  {
    //assert(is_normalized());
    return -(*this);
  }

  constexpr euler_angles<T> euler() const noexcept
  {
    euler_angles<T> e;

    // Pitch (X)
    T sinp = T(2.0) * (w * x - y * z);
    if (std::abs(sinp) >= T(1.0))
        e.pitch = std::copysign(std::numbers::pi_v<T> / T(2.0), sinp);
    else
        e.pitch = std::asin(sinp);

    // Yaw (Y)
    T siny = T(2.0) * (w * y + x * z);
    T cosy = T(1.0) - T(2.0) * (x * x + y * y);
    e.yaw = std::atan2(siny, cosy);

    // Roll (Z)
    T sinr = T(2.0) * (w * z + x * y);
    T cosr = T(1.0) - T(2.0) * (x * x + z * z);
    e.roll = std::atan2(sinr, cosr);

    return e;
  }

  constexpr quat remove_yaw() const noexcept
  {
    return (quat(euler_angles<T>{-euler().yaw, 0, 0}) * *this).normalize();
  }
  constexpr quat remove_pitch() const noexcept
  {
    return (quat(euler_angles<T>{0, -euler().pitch, 0}) * *this).normalize();
  }
  constexpr quat remove_roll() const noexcept
  {
    return (quat(euler_angles<T>{0, 0, -euler().roll}) * *this).normalize();
  }

  constexpr axis_angle<T> to_axis_angle() const noexcept
  {
    T angle = T(2.0) * std::acos(std::clamp(w, -T(1.0), T(1.0)));

    T s = std::sqrt(std::max(T(0.0), T(1.0) - w*w));

    if(s < std::numeric_limits<T>::epsilon()) [[unlikely]]
      return {{T(1.0), T(0.0), T(0.0)}, angle};

    return {{x/s, y/s, z/s}, angle};
  }

  constexpr quat operator-() const noexcept
  {
    return {-x, -y, -z, w};
  }

  friend constexpr quat operator*(const quat& q1, const quat& q2) noexcept
  {
    return
    {
      q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
      q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
      q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
      q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    };
  }
};

template<std::floating_point T>
struct mat3x3
{
  T m[3][3];

  constexpr mat3x3 inverse() const noexcept
  {
    const T c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const T c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const T c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];

    const T det =
      m[0][0] * c00 +
      m[0][1] * c01 +
      m[0][2] * c02;

    if(std::abs(det) < std::numeric_limits<T>::epsilon()) [[unlikely]]
      return {};

    const T inv_det = 1.0 / det;

    return
    {{
      {
        c00 * inv_det,
        (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
        (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det
      },
      {
        c01 * inv_det,
        (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
        (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det
      },
      {
        c02 * inv_det,
        (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
        (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det
      }
    }};
  }

  constexpr vec<T> operator*(const vec<T>& v) const noexcept
  {
    return
    {
      m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
      m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
      m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    };
  }

  friend constexpr mat3x3 operator*(const mat3x3& a, const mat3x3& b) noexcept
  {
    return
    {{
      {
        a.m[0][0] * b.m[0][0] + a.m[0][1] * b.m[1][0] + a.m[0][2] * b.m[2][0],
        a.m[0][0] * b.m[0][1] + a.m[0][1] * b.m[1][1] + a.m[0][2] * b.m[2][1],
        a.m[0][0] * b.m[0][2] + a.m[0][1] * b.m[1][2] + a.m[0][2] * b.m[2][2]
      },
      {
        a.m[1][0] * b.m[0][0] + a.m[1][1] * b.m[1][0] + a.m[1][2] * b.m[2][0],
        a.m[1][0] * b.m[0][1] + a.m[1][1] * b.m[1][1] + a.m[1][2] * b.m[2][1],
        a.m[1][0] * b.m[0][2] + a.m[1][1] * b.m[1][2] + a.m[1][2] * b.m[2][2]
      },
      {
        a.m[2][0] * b.m[0][0] + a.m[2][1] * b.m[1][0] + a.m[2][2] * b.m[2][0],
        a.m[2][0] * b.m[0][1] + a.m[2][1] * b.m[1][1] + a.m[2][2] * b.m[2][1],
        a.m[2][0] * b.m[0][2] + a.m[2][1] * b.m[1][2] + a.m[2][2] * b.m[2][2]
      }
    }};
  }

  constexpr mat3x3 transpose() const noexcept
  {
    return
    {{
      {m[0][0], m[1][0], m[2][0]},
      {m[0][1], m[1][1], m[2][1]},
      {m[0][2], m[1][2], m[2][2]}
    }};
  }

  constexpr vec<T> diag() const noexcept
  {
    return {m[0][0], m[1][1], m[2][2]};
  }

  constexpr void setColumn(int c, const vec<T>& v) noexcept
  {
    //assert(c >= 0 && c < 3);
    m[0][c] = v.x;
    m[1][c] = v.y;
    m[2][c] = v.z;
  }

  static constexpr mat3x3 identity() noexcept
  {
    return
    {{
      {T(1.0), T(0.0), T(0.0)},
      {T(0.0), T(1.0), T(0.0)},
      {T(0.0), T(0.0), T(1.0)}
    }};
  }

  static constexpr mat3x3 fromDiag(const vec<T>& d) noexcept
  {
    return
    {{
      {d.x, T(0.0), T(0.0)},
      {T(0.0), d.y, T(0.0)},
      {T(0.0), T(0.0), d.z}
    }};
  }

  static constexpr mat3x3 fromColumns(const vec<T>& c0, const vec<T>& c1, const vec<T>& c2) noexcept
  {
    mat3x3 r{};
    r.setColumn(0, c0);
    r.setColumn(1, c1);
    r.setColumn(2, c2);
    return r;
  }

  friend constexpr mat3x3 operator+(const mat3x3& a, const mat3x3& b) noexcept
  {
    return
    {{
      {a.m[0][0] + b.m[0][0], a.m[0][1] + b.m[0][1], a.m[0][2] + b.m[0][2]},
      {a.m[1][0] + b.m[1][0], a.m[1][1] + b.m[1][1], a.m[1][2] + b.m[1][2]},
      {a.m[2][0] + b.m[2][0], a.m[2][1] + b.m[2][1], a.m[2][2] + b.m[2][2]}
    }};
  }

  friend constexpr mat3x3 operator-(const mat3x3& a, const mat3x3& b) noexcept
  {
    return
    {{
      {a.m[0][0] - b.m[0][0], a.m[0][1] - b.m[0][1], a.m[0][2] - b.m[0][2]},
      {a.m[1][0] - b.m[1][0], a.m[1][1] - b.m[1][1], a.m[1][2] - b.m[1][2]},
      {a.m[2][0] - b.m[2][0], a.m[2][1] - b.m[2][1], a.m[2][2] - b.m[2][2]}
    }};
  }

  friend constexpr mat3x3 operator*(const mat3x3& a, const T& t) noexcept
  {
    return
    {{
      {a.m[0][0] * t, a.m[0][1] * t, a.m[0][2] * t},
      {a.m[1][0] * t, a.m[1][1] * t, a.m[1][2] * t},
      {a.m[2][0] * t, a.m[2][1] * t, a.m[2][2] * t}
    }};
  }

  friend constexpr mat3x3 operator*(const T& t, const mat3x3& a) noexcept
  {
    return a * t;
  }

  friend constexpr mat3x3 operator/(const mat3x3& a, const T& t) noexcept
  {
    return a * (T(1.0) / t);
  }
};

template<std::floating_point T>
struct qv
{
  quat<T> q;
  vec<T> v;
};

template<std::floating_point T>
constexpr T dot(vec<T> v1, vec<T> v2) noexcept
{
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

template<std::floating_point T>
constexpr vec<T> cross(vec<T> v1, vec<T> v2) noexcept
{
  return
  {
    v1.y * v2.z - v1.z * v2.y,
    v1.z * v2.x - v1.x * v2.z,
    v1.x * v2.y - v1.y * v2.x
  };
}

template<std::floating_point T>
constexpr vec<T> sqrt_vec(vec<T> v) noexcept
{
  return {std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z)};
}

template<std::floating_point T>
constexpr quat<T> quat_off(quat<T> from, quat<T> to) noexcept
{
  //assert(from.is_normalized());
  //assert(to.is_normalized());

  // dq * from = to
  // dq * from * from.inverse() = to * from.inverse()
  // dq = to * from.inverse

  return to * from.inverse();
}

template<std::floating_point T>
constexpr vec<T> quat_mul_vec(quat<T> q, vec<T> v) noexcept
{
  //assert(q.is_normalized());

  // t = 2 * (q.xyz × v)
  const T tx = T(2.0) * (q.y * v.z - q.z * v.y);
  const T ty = T(2.0) * (q.z * v.x - q.x * v.z);
  const T tz = T(2.0) * (q.x * v.y - q.y * v.x);

  // v' = v + q.w * t + (q.xyz × t)
  return
  {
    v.x + q.w * tx + (q.y * tz - q.z * ty),
    v.y + q.w * ty + (q.z * tx - q.x * tz),
    v.z + q.w * tz + (q.x * ty - q.y * tx)
  };
}

template<std::floating_point T>
constexpr vec<T> qv_mul_vec(qv<T> qv, vec<T> v) noexcept
{
  //assert(qv.q.is_normalized());

  return qv.v + quat_mul_vec(qv.q, v);
}

template<std::floating_point T>
constexpr quat<T> quat_lerp(quat<T> from, quat<T> to, T t) noexcept
{
  //assert(from.is_normalized());
  //assert(to.is_normalized());

  T dot =
    from.w * to.w +
    from.x * to.x +
    from.y * to.y +
    from.z * to.z;

  T sign = (dot < T(0.0)) ? -T(1.0) : T(1.0);

  quat<T> q
  {
    (T(1.0) - t) * from.x + t * sign * to.x,
    (T(1.0) - t) * from.y + t * sign * to.y,
    (T(1.0) - t) * from.z + t * sign * to.z,
    (T(1.0) - t) * from.w + t * sign * to.w
  };

  return q.normalize();
}

template<std::floating_point T>
constexpr vec<T> angular_velocity(quat<T> from, quat<T> to, T dt) noexcept
{
  //assert(from.is_normalized());
  //assert(to.is_normalized());

  quat<T> dq = quat_off(from, to);
  axis_angle<T> aa = dq.axis_angle();
  aa.axis = aa.axis.normalize();

  return aa.axis * (aa.angle/dt);
}

template<std::floating_point T>
constexpr quat<T> quat_predict(quat<T> q, vec<T> vel, T dt) noexcept
{
  //assert(q.is_normalized());

  vec<T> rot_vec = vel*dt;
  T rot_angle = rot_vec.norm();

  if(rot_angle < std::numeric_limits<T>::epsilon()) [[unlikely]] return q;

  vec<T> rot_axis = rot_vec/rot_angle;
  quat<T> dq = axis_angle<T>{rot_axis, rot_angle};

  return q * dq;
}

template<std::floating_point T>
constexpr vec<T> vec_predict(vec<T> v, vec<T> vel, T dt) noexcept
{
  return v + vel*dt;
}

template<std::floating_point T>
constexpr T deg2rad(T d) noexcept
{
  return d * (std::numbers::pi_v<T> / T(180.0));
}

template<std::floating_point T>
constexpr T rad2deg(T r) noexcept
{
  return r * (T(180.0) / std::numbers::pi_v<T>);
}

} //namespace zt
