#include <Arduino.h>
#include "user_config.hpp"
#include "lsm6dsr.hpp"

zt::LSM6DSR g_imu(IMU_I2C_ADDRESS, PIN_IMU_SDA, PIN_IMU_SCL, I2C_SPEED);

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);
  Serial.println("\n=== ESP-12F LSM6DSR Tracker ===");

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  if(g_imu.data_ready())
  {
    const auto sample = g_imu.read_sample();
    Serial.printf(
      "gyr: {%i, %i, %i} acc: {%i, %i, %i}\n",
      sample.gyr[0], sample.gyr[1], sample.gyr[2], sample.acc[0], sample.acc[1], sample.acc[2]);
  }
  yield();
}
