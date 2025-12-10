/**
 * @file LSM6DSV32X.h
 * @brief Driver for the LSM6DSV32XTR 6-axis IMU
 * I2C Address: 0x6A (SA0=0) or 0x6B (SA0=1)
 */

#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define LSM6DSV32X_ADDRESS_LOW    0x6A
#define LSM6DSV32X_ADDRESS_HIGH   0x6B
#define LSM6DSV32X_DEFAULT_ADDR   LSM6DSV32X_ADDRESS_LOW

// Registers
#define LSM6DSV32X_WHO_AM_I       0x0F
#define LSM6DSV32X_CTRL1_XL       0x10
#define LSM6DSV32X_CTRL2_G        0x11
#define LSM6DSV32X_CTRL3_C        0x12
#define LSM6DSV32X_STATUS_REG     0x1E
#define LSM6DSV32X_OUT_TEMP_L     0x20
#define LSM6DSV32X_OUTX_L_G       0x22
#define LSM6DSV32X_OUTX_L_A       0x28

#define LSM6DSV32X_CHIP_ID        0x70

typedef enum {
    LSM6DSV32X_ACC_4G = 0, LSM6DSV32X_ACC_8G, LSM6DSV32X_ACC_16G, LSM6DSV32X_ACC_32G
} lsm6dsv32x_acc_range_t;

typedef enum {
    LSM6DSV32X_GYRO_125DPS = 0, LSM6DSV32X_GYRO_250DPS, LSM6DSV32X_GYRO_500DPS,
    LSM6DSV32X_GYRO_1000DPS, LSM6DSV32X_GYRO_2000DPS, LSM6DSV32X_GYRO_4000DPS
} lsm6dsv32x_gyro_range_t;

typedef enum {
    LSM6DSV32X_ODR_OFF = 0, LSM6DSV32X_ODR_1_875HZ, LSM6DSV32X_ODR_7_5HZ,
    LSM6DSV32X_ODR_15HZ, LSM6DSV32X_ODR_30HZ, LSM6DSV32X_ODR_60HZ,
    LSM6DSV32X_ODR_120HZ, LSM6DSV32X_ODR_240HZ, LSM6DSV32X_ODR_480HZ,
    LSM6DSV32X_ODR_960HZ, LSM6DSV32X_ODR_1920HZ, LSM6DSV32X_ODR_3840HZ,
    LSM6DSV32X_ODR_7680HZ
} lsm6dsv32x_odr_t;

class LSM6DSV32X {
public:
    LSM6DSV32X();
    bool begin_I2C(uint8_t addr = LSM6DSV32X_DEFAULT_ADDR, TwoWire *wire = &Wire);
    bool begin_SPI(int8_t cs, SPIClass *spi = &SPI, uint32_t freq = 10000000);
    
    void setAccRange(lsm6dsv32x_acc_range_t range);
    void setGyroRange(lsm6dsv32x_gyro_range_t range);
    void setAccODR(lsm6dsv32x_odr_t odr);
    void setGyroODR(lsm6dsv32x_odr_t odr);
    
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
    int8_t _cs;
    uint8_t _addr;
    bool _useSPI;
    uint32_t _spiFreq;
    float _accSens, _gyroSens;
    
    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t val);
    void readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif
