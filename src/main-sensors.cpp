/**
 * @file main-sensors.cpp
 * @brief Test file for all sensors on the STM32H723VGT6 custom PCB
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Sensor drivers
#include "LIS3MDL.h"
#include "MS5611.h"
#include "BMI088.h"
#include "LSM6DSV32X.h"

// =============================================================================
// PIN CONFIGURATION - MODIFY THESE TO MATCH YOUR PCB DESIGN
// =============================================================================

// --- I2C Configuration ---
#define I2C_SDA_PIN         PB9     // TODO: Change to your I2C SDA pin
#define I2C_SCL_PIN         PB8     // TODO: Change to your I2C SCL pin

// --- SPI Configuration (if using SPI mode) ---
#define SPI_SCK_PIN         PA5     // TODO: Change to your SPI SCK pin
#define SPI_MISO_PIN        PA6     // TODO: Change to your SPI MISO pin
#define SPI_MOSI_PIN        PA7     // TODO: Change to your SPI MOSI pin

// --- Chip Select Pins (for SPI mode) ---
#define LIS3MDL_CS_PIN      PA4     // TODO: Magnetometer CS pin
#define MS5611_CS_PIN       PA3     // TODO: Barometer CS pin
#define BMI088_ACC_CS_PIN   PA2     // TODO: BMI088 Accelerometer CS pin
#define BMI088_GYRO_CS_PIN  PA1     // TODO: BMI088 Gyroscope CS pin
#define LSM6DSV_CS_PIN      PA0     // TODO: LSM6DSV32X CS pin
#define ADXL375_CS_PIN      PC0     // TODO: ADXL375 CS pin

// --- I2C Addresses (modify if using non-default addresses) ---
#define LIS3MDL_I2C_ADDR    0x1C    // 0x1C (SA1=low) or 0x1E (SA1=high)
#define MS5611_I2C_ADDR     0x76    // 0x76 (CSB=high) or 0x77 (CSB=low)
#define BMI088_ACC_I2C_ADDR 0x18    // 0x18 (SDO1=GND) or 0x19 (SDO1=VCC)
#define BMI088_GYRO_I2C_ADDR 0x68   // 0x68 (SDO2=GND) or 0x69 (SDO2=VCC)
#define LSM6DSV_I2C_ADDR    0x6A    // 0x6A (SA0=low) or 0x6B (SA0=high)
#define ADXL375_I2C_ADDR    0x53    // 0x53 (ALT=low) or 0x1D (ALT=high)

// --- GPS UART Configuration ---
#define GPS_SERIAL          Serial2  // TODO: Change to your GPS UART
#define GPS_TX_PIN          PA2      // TODO: GPS TX pin (to MCU RX)
#define GPS_RX_PIN          PA3      // TODO: GPS RX pin (from MCU TX)
#define GPS_BAUDRATE        9600

// --- Communication Mode Selection ---
#define USE_I2C_MODE        true    // Set to false to use SPI mode

// =============================================================================
// END PIN CONFIGURATION
// =============================================================================

// Create sensor instances
LIS3MDL magnetometer;
MS5611 barometer;
BMI088 bmi088;
LSM6DSV32X lsm6dsv;

// Status flags
bool magOk = false;
bool baroOk = false;
bool bmi088Ok = false;
bool lsmOk = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);
    
    Serial.println("\n=== STM32H723VGT6 Sensor Test ===\n");
    
#if USE_I2C_MODE
    // Initialize I2C with configured pins
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();
    
    Serial.println("Using I2C mode\n");
    
    // Initialize sensors with configured I2C addresses
    Serial.print("LIS3MDL... ");
    magOk = magnetometer.begin_I2C(LIS3MDL_I2C_ADDR, &Wire);
    Serial.println(magOk ? "OK" : "FAIL");
    
    Serial.print("MS5611... ");
    baroOk = barometer.begin_I2C(MS5611_I2C_ADDR, &Wire);
    Serial.println(baroOk ? "OK" : "FAIL");
    
    Serial.print("BMI088... ");
    bmi088Ok = bmi088.begin_I2C(BMI088_ACC_I2C_ADDR, BMI088_GYRO_I2C_ADDR, &Wire);
    Serial.println(bmi088Ok ? "OK" : "FAIL");
    
    Serial.print("LSM6DSV32X... ");
    lsmOk = lsm6dsv.begin_I2C(LSM6DSV_I2C_ADDR, &Wire);
    Serial.println(lsmOk ? "OK" : "FAIL");
    
#else
    // Initialize SPI with configured pins
    SPI.setMISO(SPI_MISO_PIN);
    SPI.setMOSI(SPI_MOSI_PIN);
    SPI.setSCLK(SPI_SCK_PIN);
    SPI.begin();
    
    Serial.println("Using SPI mode\n");
    
    // Initialize sensors with configured CS pins
    Serial.print("LIS3MDL... ");
    magOk = magnetometer.begin_SPI(LIS3MDL_CS_PIN, &SPI);
    Serial.println(magOk ? "OK" : "FAIL");
    
    Serial.print("MS5611... ");
    baroOk = barometer.begin_SPI(MS5611_CS_PIN, &SPI);
    Serial.println(baroOk ? "OK" : "FAIL");
    
    Serial.print("BMI088... ");
    bmi088Ok = bmi088.begin_SPI(BMI088_ACC_CS_PIN, BMI088_GYRO_CS_PIN, &SPI);
    Serial.println(bmi088Ok ? "OK" : "FAIL");
    
    Serial.print("LSM6DSV32X... ");
    lsmOk = lsm6dsv.begin_SPI(LSM6DSV_CS_PIN, &SPI);
    Serial.println(lsmOk ? "OK" : "FAIL");
#endif
    
    Serial.println("\n--- Sensor Loop Starting ---\n");
}

void loop() {
    // LIS3MDL Magnetometer
    if (magOk) {
        magnetometer.read();
        Serial.print("MAG: ");
        Serial.print(magnetometer.x, 3); Serial.print(", ");
        Serial.print(magnetometer.y, 3); Serial.print(", ");
        Serial.print(magnetometer.z, 3); Serial.println(" gauss");
    }
    
    // MS5611 Barometer
    if (baroOk) {
        barometer.read();
        Serial.print("BARO: ");
        Serial.print(barometer.pressure, 2); Serial.print(" hPa, ");
        Serial.print(barometer.temperature, 2); Serial.print(" C, ");
        Serial.print(barometer.altitude, 1); Serial.println(" m");
    }
    
    // BMI088 IMU
    if (bmi088Ok) {
        bmi088.read();
        Serial.print("BMI088 Accel: ");
        Serial.print(bmi088.accelX, 3); Serial.print(", ");
        Serial.print(bmi088.accelY, 3); Serial.print(", ");
        Serial.print(bmi088.accelZ, 3); Serial.println(" m/s^2");
        Serial.print("BMI088 Gyro: ");
        Serial.print(bmi088.gyroX, 3); Serial.print(", ");
        Serial.print(bmi088.gyroY, 3); Serial.print(", ");
        Serial.print(bmi088.gyroZ, 3); Serial.println(" rad/s");
    }
    
    // LSM6DSV32X IMU
    if (lsmOk) {
        lsm6dsv.read();
        Serial.print("LSM6DSV Accel: ");
        Serial.print(lsm6dsv.accelX, 3); Serial.print(", ");
        Serial.print(lsm6dsv.accelY, 3); Serial.print(", ");
        Serial.print(lsm6dsv.accelZ, 3); Serial.println(" m/s^2");
        Serial.print("LSM6DSV Gyro: ");
        Serial.print(lsm6dsv.gyroX, 3); Serial.print(", ");
        Serial.print(lsm6dsv.gyroY, 3); Serial.print(", ");
        Serial.print(lsm6dsv.gyroZ, 3); Serial.println(" rad/s");
    }
    
    Serial.println("---");
    delay(100);
}
