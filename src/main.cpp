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
  auto start = micros();
  if(g_imu.data_ready())
  {
    vqf_real_t q[4];
    g_imu.read_quat(q);

    g_packet_sender.send(
    {
      .packet_type_padding = 0,
      .packet_type = PACKET_TYPE_ROTATION,
      .packet_number = ++g_packet_number,
      .x = float(q[1]), .y = float(q[2]), .z = float(q[3]), .w = float(q[0])
    });
    auto end = micros();
    Serial.printf("%lu\n", end-start);
  }
  yield();
}
