#pragma once

#include <cstring>
#include <numbers>
#include "bus.hpp"
#include "vqf.hpp"

namespace zt
{

class LSM6DSR
{
  Bus m_bus;
  VQF m_vqf{1.0/208.0};

  static constexpr vqf_real_t gravity = 9.80665;
  static constexpr vqf_real_t gsens = 1000.0 / 35.0;
  static constexpr vqf_real_t asens = 1000.0 / 0.122;
  static constexpr vqf_real_t gscale = ((32768. / gsens) / 32768.) * (std::numbers::pi / 180.0);
  static constexpr vqf_real_t ascale = gravity / asens;

  enum ODR : std::uint8_t
  {
    HZ_208 = 0b0101
  };

  enum FSXL : std::uint8_t
  {
    FS_4G = 0b10
  };

  enum FSG : std::uint8_t
  {
    FS_1000DPS = 0b10
  };

  struct RegCtrl1XL
  {
    static constexpr std::uint8_t addr = 0x10;

    std::uint8_t zero        : 1;
    std::uint8_t lpf2_xl_en  : 1;
    FSXL         fs_xl       : 2;
    ODR          odr_xl      : 4;
  };
  static_assert(sizeof(RegCtrl1XL) == 1);

  struct RegCtrl2G
  {
    static constexpr std::uint8_t addr = 0x11;

    std::uint8_t fs_4000     : 1;
    std::uint8_t fs_125      : 1;
    FSG          fs_g        : 2;
    ODR          odr_g       : 4;
  };
  static_assert(sizeof(RegCtrl2G) == 1);

  struct RegCtrl3C
  {
    static constexpr std::uint8_t addr = 0x12;

    std::uint8_t sw_reset    : 1;
    std::uint8_t zero        : 1;
    std::uint8_t if_inc      : 1;
    std::uint8_t sim         : 1;
    std::uint8_t pp_od       : 1;
    std::uint8_t h_lactive   : 1;
    std::uint8_t bdu         : 1;
    std::uint8_t boot        : 1;
  };
  static_assert(sizeof(RegCtrl3C) == 1);

  struct RegStatus
  {
    static constexpr std::uint8_t addr = 0x1E;

    std::uint8_t xlda : 1;
    std::uint8_t gda  : 1;
    std::uint8_t tda  : 1;
    std::uint8_t zero : 5;
  };
  static_assert(sizeof(RegStatus) == 1);

  template<typename T>
  T read_reg()
  {
    return m_bus.read<T>(T::addr);
  }

  template<typename T>
  void write_reg(T reg)
  {
    m_bus.write<T>(T::addr, reg);
  }

public:
  struct Sample
  {
    static constexpr uint8_t addr = 0x22;

    std::int16_t gyr[3];
    std::int16_t acc[3];
  };
  static_assert(sizeof(Sample) == 12);

  struct SampleWithTemp
  {
    static constexpr uint8_t addr = 0x22;

    std::int16_t gyr[3];
    std::int16_t acc[3];
    std::int16_t temp;
  };
  static_assert(sizeof(SampleWithTemp) == 14);

  void begin(std::uint8_t addr, Pin sda, Pin scl, std::uint32_t freq)
  {
    m_bus.begin(addr, sda, scl, freq);

    //Reboot IMU
    write_reg<RegCtrl3C>({.sw_reset = 1});
    delay(20);

    //Configure
    write_reg<RegCtrl1XL>({.fs_xl = FS_4G, .odr_xl = HZ_208});
    write_reg<RegCtrl2G>({.fs_g = FS_1000DPS, .odr_g = HZ_208});
    write_reg<RegCtrl3C>({.if_inc = 1});
  }

  bool data_ready()
  {
    RegStatus status = read_reg<RegStatus>();
    return status.xlda == 1 && status.gda == 1;
  }

  Sample read_sample()
  {
    return read_reg<Sample>();
  }

  SampleWithTemp read_sample_with_temp()
  {
    return read_reg<SampleWithTemp>();
  }

  void read_quat(vqf_real_t q[4])
  {
    const auto sample = read_sample();
    const vqf_real_t gyr[3] = { vqf_real_t(sample.gyr[0])*gscale, vqf_real_t(sample.gyr[1])*gscale, vqf_real_t(sample.gyr[2])*gscale };
    const vqf_real_t acc[3] = { vqf_real_t(sample.acc[0])*ascale, vqf_real_t(sample.acc[1])*ascale, vqf_real_t(sample.acc[2])*ascale };
    m_vqf.update(gyr, acc);

    m_vqf.getQuat6D(q);
  }
};

} //namespace zt
