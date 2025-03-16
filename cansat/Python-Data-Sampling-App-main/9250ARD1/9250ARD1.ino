
#include "mpu9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define I2C_SDA 21
#define I2C_SCL 22

TwoWire I2CMPU = TwoWire(0);
/* Mpu9250 object */
bfs::Mpu9250 imu;

void setup()
{
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin())
  {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
}

void loop()
{
  /* Check if data read */
  if (imu.Read())
  {
    Serial.print(imu.gyro_x_radps());
    Serial.print(", ");
    Serial.print(imu.gyro_y_radps());
    Serial.print(", ");
    Serial.print(imu.gyro_z_radps());
    Serial.print(", ");

    Serial.print(imu.accel_x_mps2());
    Serial.print(", ");
    Serial.print(imu.accel_y_mps2());
    Serial.print(", ");
    Serial.print(imu.accel_z_mps2());
    Serial.print(", ");

    Serial.print(imu.mag_x_ut());
    Serial.print(", ");
    Serial.print(imu.mag_y_ut());
    Serial.print(", ");
    Serial.println(imu.mag_z_ut());
    delay(100); 
  }
}