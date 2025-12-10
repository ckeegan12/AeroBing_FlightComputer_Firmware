/**
 * @file BMI088.h
 * @brief Driver for the BMI088 6-axis IMU
 */

#ifndef BMI088_H
#define BMI088_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// I2C Addresses
#define BMI088_ACC_ADDRESS_LOW   0x18
#define BMI088_ACC_ADDRESS_HIGH  0x19
#define BMI088_GYRO_ADDRESS_LOW  0x68
#define BMI088_GYRO_ADDRESS_HIGH 0x69

#define BMI088_ACC_DEFAULT_ADDRESS  BMI088_ACC_ADDRESS_LOW
#define BMI088_GYRO_DEFAULT_ADDRESS BMI088_GYRO_ADDRESS_LOW

// Accelerometer registers
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_X_LSB        0x12
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
#define BMI088_ACC_SOFTRESET    0x7E

// Gyroscope registers
#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_X_LSB       0x02
#define BMI088_GYRO_RANGE       0x0F
#define BMI088_GYRO_BANDWIDTH   0x10
#define BMI088_GYRO_SOFTRESET   0x14

// Chip IDs
#define BMI088_ACC_CHIP_ID_VALUE  0x1E
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

typedef enum {
    BMI088_ACC_RANGE_3G = 0, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_12G, BMI088_ACC_RANGE_24G
} bmi088_acc_range_t;

typedef enum {
    BMI088_GYRO_RANGE_2000DPS = 0, BMI088_GYRO_RANGE_1000DPS, BMI088_GYRO_RANGE_500DPS, 
    BMI088_GYRO_RANGE_250DPS, BMI088_GYRO_RANGE_125DPS
} bmi088_gyro_range_t;

class BMI088 {
public:
    BMI088();
    bool begin_I2C(uint8_t acc_addr = BMI088_ACC_DEFAULT_ADDRESS, 
                   uint8_t gyro_addr = BMI088_GYRO_DEFAULT_ADDRESS, TwoWire *wire = &Wire);
    bool begin_SPI(int8_t acc_cs, int8_t gyro_cs, SPIClass *spi = &SPI, uint32_t freq = 10000000);
    
    void setAccRange(bmi088_acc_range_t range);
    void setGyroRange(bmi088_gyro_range_t range);
    bool read();
    void reset();
    
    int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;
    float accelX, accelY, accelZ;  // m/s²
    float gyroX, gyroY, gyroZ;     // rad/s
    float temperature;

private:
    TwoWire *_wire;
    SPIClass *_spi;
    int8_t _accCs, _gyroCs;
    uint8_t _accAddr, _gyroAddr;
    bool _useSPI;
    uint32_t _spiFreq;
    float _accSens, _gyroSens;
    
    uint8_t readAccReg(uint8_t reg);
    void writeAccReg(uint8_t reg, uint8_t val);
    uint8_t readGyroReg(uint8_t reg);
    void writeGyroReg(uint8_t reg, uint8_t val);
};

#endif
