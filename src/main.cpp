#include <Arduino.h>
#include "user_config.hpp"
#include "lsm6dsr.hpp"
#include "wifi.hpp"
#include "protocol.hpp"
#include "server_discovery.hpp"
#include "packet_sender.hpp"

zt::LSM6DSR g_imu(IMU_I2C_ADDRESS, PIN_IMU_SDA, PIN_IMU_SCL, I2C_SPEED);
ip_addr_t g_server_ip;
zt::PacketSender<zt::RotationPacket> g_packet_sender;
std::uint32_t g_packet_number = 0;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);
  Serial.println("\n=== ESP-12F LSM6DSR Tracker ===");

  pinMode(LED_BUILTIN, OUTPUT);

  zt::wifi_init();
  Serial.println("Connected to WiFi!");

  g_server_ip = zt::server_discovery();
  Serial.println("Connected to the server!");

  g_packet_sender = zt::PacketSender<zt::RotationPacket>(g_server_ip);
}

void loop()
{
  if(g_imu.data_ready())
  {
  #if !IMUCAL_RECORDING
    vqf_real_t q[4];
    g_imu.read_quat(q);

    g_packet_sender.send(
    {
      .packet_type_padding = 0,
      .packet_type = PACKET_TYPE_ROTATION,
      .packet_number = ++g_packet_number,
      .x = float(q[1]), .y = float(q[2]), .z = float(q[3]), .w = float(q[0])
    });
  #else
    const std::uint32_t timestamp_us = micros();
    const auto sample = g_imu.read_sample_with_temp();
    g_packet_sender.send(
    {
      .packet_number = ++g_packet_number,
      .timestamp_us = timestamp_us,
      .gyr = { sample.gyr[0], sample.gyr[1], sample.gyr[2] },
      .acc = { sample.acc[0], sample.acc[1], sample.acc[2] },
      .temp = sample.temp
    });
  #endif
  }
  yield();
}
