#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define I2C_SDA 21
#define I2C_SCL 22

TwoWire I2CMPU = TwoWire(0);
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  I2CMPU.begin(I2C_SDA, I2C_SCL);

  if (!mpu.begin(104, &I2CMPU, 0x68)) {
    Serial.println("No MPU6050 detected");
    while (1) {
      delay(10);
    }
  }

  delay(100);   

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  Serial.println("READING...");
  delay(100);
}

void loop()
{
  delay(7); // To make sampling rate around 100Hz

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.print(g.gyro.z);
  Serial.print(", ");

  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.println(", ");
  Serial.println(a.acceleration.z);
}
