#include <mpu9250.h>
#include <Wire.h>

TwoWire I2CMPU = TwoWire(0);
bfs::Mpu9250 imu;

// Estructura para almacenar las muestras
struct SampleData {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float mag_x;
    float mag_y;
    float mag_z;
};

SampleData samples[5];  // Array para almacenar hasta 5 muestras
int sampleCount = 0;    // Contador de muestras

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    // Inicializar I2C en pines 21 (SDA) y 22 (SCL) para ESP32
    Wire.begin(21, 22);
    Wire.setClock(400000); // Configurar frecuencia a 400kHz

    // Configurar IMU en dirección primaria
    imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

    // Inicializar y configurar el IMU
    if (!imu.Begin()) {
        Serial.println("Error al inicializar la comunicación con el IMU");
        while (1) {}
    }

    // Configurar Sample Rate Divider (SRD)
    if (!imu.ConfigSrd(19)) {
        Serial.println("Error al configurar SRD");
        while (1) {}
    }
}

void loop() {
    unsigned long startTime = millis();
    unsigned long currentTime = startTime;

    // Tomar muestras durante 10 ms
    while (currentTime - startTime < 10) {
        if (imu.Read()) {
            // Leer datos crudos de giroscopio, acelerómetro y magnetómetro
            samples[sampleCount].gyro_x = imu.gyro_x_radps();
            samples[sampleCount].gyro_y = imu.gyro_y_radps();
            samples[sampleCount].gyro_z = imu.gyro_z_radps();
            samples[sampleCount].accel_x = imu.accel_x_mps2();
            samples[sampleCount].accel_y = imu.accel_y_mps2();
            samples[sampleCount].accel_z = imu.accel_z_mps2();
            samples[sampleCount].mag_x = imu.mag_x_ut();
            samples[sampleCount].mag_y = imu.mag_y_ut();
            samples[sampleCount].mag_z = imu.mag_z_ut();
            sampleCount++;

            // Asegúrate de no exceder el tamaño del array
            if (sampleCount >= 5) {
                sampleCount = 0;  // Reinicia el contador
            }
        }
        currentTime = millis();
    }

    // Esperar 50 ms antes de enviar los promedios
    delay(50);

    // Calcular promedios y enviar
    if (sampleCount > 0) {
        float avg_gyro_x = 0, avg_gyro_y = 0, avg_gyro_z = 0;
        float avg_accel_x = 0, avg_accel_y = 0, avg_accel_z = 0;
        float avg_mag_x = 0, avg_mag_y = 0, avg_mag_z = 0;

        for (int i = 0; i < sampleCount; i++) {
            avg_gyro_x += samples[i].gyro_x;
            avg_gyro_y += samples[i].gyro_y;
            avg_gyro_z += samples[i].gyro_z;
            avg_accel_x += samples[i].accel_x;
            avg_accel_y += samples[i].accel_y;
            avg_accel_z += samples[i].accel_z;
            avg_mag_x += samples[i].mag_x;
            avg_mag_y += samples[i].mag_y;
            avg_mag_z += samples[i].mag_z;
        }

        // Calcular el promedio
        avg_gyro_x /= sampleCount;
        avg_gyro_y /= sampleCount;
        avg_gyro_z /= sampleCount;
        avg_accel_x /= sampleCount;
        avg_accel_y /= sampleCount;
        avg_accel_z /= sampleCount;
        avg_mag_x /= sampleCount;
        avg_mag_y /= sampleCount;
        avg_mag_z /= sampleCount;

        // Enviar promedios por Serial
        Serial.print(avg_gyro_x, 6);
        Serial.print(", ");
        Serial.print(avg_gyro_y, 6);
        Serial.print(", ");
        Serial.print(avg_gyro_z, 6);
        Serial.print(", ");
        Serial.print(avg_accel_x, 6);
        Serial.print(", ");
        Serial.print(avg_accel_y, 6);
        Serial.print(", ");
        Serial.print(avg_accel_z, 6);
        Serial.print(", ");
        Serial.print(avg_mag_x, 6);
        Serial.print(", ");
        Serial.print(avg_mag_y, 6);
        Serial.print(", ");
        Serial.println(avg_mag_z, 6);
    }
}
