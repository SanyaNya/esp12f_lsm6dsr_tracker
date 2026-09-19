#include <Arduino.h>
#include "user_config.hpp"
#include "logger.hpp"
#include "lsm6dsr.hpp"
#include "wifi.hpp"
#include "protocol.hpp"
#include "server_discovery.hpp"
#include "packet_sender.hpp"
#include "precise_timer.hpp"
#include "recursive_linear_regression.hpp"

zt::Logger g_logger;
zt::LSM6DSR g_imu;
zt::PacketSender<zt::RotationPacket> g_packet_sender;
#if IMUCAL_RECORDING
zt::RotationPacket g_packet{};
#else
zt::PreciseTimer g_timer;
zt::RecursiveLinearRegression g_rls;
std::uint32_t g_packet_number = 0;
#endif

void setup()
{
  g_logger.begin();

  pinMode(LED_BUILTIN, OUTPUT);

  g_imu.begin(IMU_I2C_ADDRESS, PIN_IMU_SDA, PIN_IMU_SCL, I2C_SPEED);
  g_logger.println("IMU initialized!");

  zt::wifi_init();
  g_logger.println("Connected to WiFi!");

  ip_addr_t server_ip = zt::server_discovery();
  g_logger.println("Connected to the server!");

  g_packet_sender.begin(server_ip);
  g_logger.println("Init complete!");
}

void loop()
{
  if(g_imu.data_ready())
  {
  #if !IMUCAL_RECORDING
    const double t = g_timer.now();
    g_rls.update(g_packet_number, t);

    vqf_real_t q[4];
    g_imu.read_quat(q, g_rls.slope());

    g_packet_sender.send(
    {
      .packet_type_padding = 0,
      .packet_type = PACKET_TYPE_ROTATION,
      .packet_number = g_packet_number,
      .x = float(q[1]), .y = float(q[2]), .z = float(q[3]), .w = float(q[0])
    });
    ++g_packet_number;
  #else
    const std::uint32_t cycles = ESP.getCycleCount();
    const auto sample = g_imu.read_sample_with_temp();

    const std::uint32_t subpacket_idx = g_packet.packet_number % 4;

    ++g_packet.packet_number;

    g_packet.timestamp_cycles[subpacket_idx] = cycles;

    g_packet.temp[subpacket_idx] = sample.temp;

    g_packet.gyr[subpacket_idx][0] = sample.gyr[0];
    g_packet.gyr[subpacket_idx][1] = sample.gyr[1];
    g_packet.gyr[subpacket_idx][2] = sample.gyr[2];

    g_packet.acc[subpacket_idx][0] = sample.acc[0];
    g_packet.acc[subpacket_idx][1] = sample.acc[1];
    g_packet.acc[subpacket_idx][2] = sample.acc[2];

    if(subpacket_idx+1 == 4)
      g_packet_sender.send(g_packet);
  #endif
  }
}
