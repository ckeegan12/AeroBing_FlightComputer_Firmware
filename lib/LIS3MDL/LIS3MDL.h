/**
 * @file LIS3MDL.h
 * @brief Driver for the LIS3MDLTR 3-axis magnetometer
 * 
 * Supports both I2C and SPI communication.
 * I2C Address: 0x1C (SA1=0) or 0x1E (SA1=1)
 */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// I2C Addresses
#define LIS3MDL_ADDRESS_LOW     0x1C  // SA1 pin low
#define LIS3MDL_ADDRESS_HIGH    0x1E  // SA1 pin high
#define LIS3MDL_DEFAULT_ADDRESS LIS3MDL_ADDRESS_LOW

// Register addresses
#define LIS3MDL_REG_WHO_AM_I    0x0F
#define LIS3MDL_REG_CTRL_REG1   0x20
#define LIS3MDL_REG_CTRL_REG2   0x21
#define LIS3MDL_REG_CTRL_REG3   0x22
#define LIS3MDL_REG_CTRL_REG4   0x23
#define LIS3MDL_REG_CTRL_REG5   0x24
#define LIS3MDL_REG_STATUS      0x27
#define LIS3MDL_REG_OUT_X_L     0x28
#define LIS3MDL_REG_OUT_X_H     0x29
#define LIS3MDL_REG_OUT_Y_L     0x2A
#define LIS3MDL_REG_OUT_Y_H     0x2B
#define LIS3MDL_REG_OUT_Z_L     0x2C
#define LIS3MDL_REG_OUT_Z_H     0x2D
#define LIS3MDL_REG_TEMP_OUT_L  0x2E
#define LIS3MDL_REG_TEMP_OUT_H  0x2F
#define LIS3MDL_REG_INT_CFG     0x30
#define LIS3MDL_REG_INT_SRC     0x31
#define LIS3MDL_REG_INT_THS_L   0x32
#define LIS3MDL_REG_INT_THS_H   0x33

// Device ID
#define LIS3MDL_DEVICE_ID       0x3D

// Full-scale selection
typedef enum {
    LIS3MDL_RANGE_4_GAUSS  = 0b00,  // ±4 gauss
    LIS3MDL_RANGE_8_GAUSS  = 0b01,  // ±8 gauss
    LIS3MDL_RANGE_12_GAUSS = 0b10,  // ±12 gauss
    LIS3MDL_RANGE_16_GAUSS = 0b11   // ±16 gauss
} lis3mdl_range_t;

// Output data rate
typedef enum {
    LIS3MDL_DATARATE_0_625_HZ = 0b0000,
    LIS3MDL_DATARATE_1_25_HZ  = 0b0010,
    LIS3MDL_DATARATE_2_5_HZ   = 0b0100,
    LIS3MDL_DATARATE_5_HZ     = 0b0110,
    LIS3MDL_DATARATE_10_HZ    = 0b1000,
    LIS3MDL_DATARATE_20_HZ    = 0b1010,
    LIS3MDL_DATARATE_40_HZ    = 0b1100,
    LIS3MDL_DATARATE_80_HZ    = 0b1110,
    LIS3MDL_DATARATE_155_HZ   = 0b0001,  // Ultra-high performance
    LIS3MDL_DATARATE_300_HZ   = 0b0011,
    LIS3MDL_DATARATE_560_HZ   = 0b0101,
    LIS3MDL_DATARATE_1000_HZ  = 0b0111
} lis3mdl_datarate_t;

// Performance mode
typedef enum {
    LIS3MDL_LOWPOWERMODE       = 0b00,
    LIS3MDL_MEDIUMMODE         = 0b01,
    LIS3MDL_HIGHMODE           = 0b10,
    LIS3MDL_ULTRAHIGHMODE      = 0b11
} lis3mdl_performancemode_t;

// Operating mode
typedef enum {
    LIS3MDL_CONTINUOUSMODE     = 0b00,
    LIS3MDL_SINGLEMODE         = 0b01,
    LIS3MDL_POWERDOWNMODE      = 0b10
} lis3mdl_operationmode_t;

class LIS3MDL {
public:
    LIS3MDL();
    
    // Initialization
    bool begin_I2C(uint8_t i2c_address = LIS3MDL_DEFAULT_ADDRESS, TwoWire *wire = &Wire);
    bool begin_SPI(int8_t cs_pin, SPIClass *spi = &SPI, uint32_t frequency = 1000000);
    
    // Configuration
    void setRange(lis3mdl_range_t range);
    lis3mdl_range_t getRange();
    
    void setDataRate(lis3mdl_datarate_t rate);
    lis3mdl_datarate_t getDataRate();
    
    void setPerformanceMode(lis3mdl_performancemode_t mode);
    lis3mdl_performancemode_t getPerformanceMode();
    
    void setOperationMode(lis3mdl_operationmode_t mode);
    lis3mdl_operationmode_t getOperationMode();
    
    // Data reading
    bool read();
    bool dataReady();
    
    // Raw data (in LSB)
    int16_t rawX, rawY, rawZ;
    
    // Scaled data (in gauss)
    float x, y, z;
    
    // Temperature
    int16_t rawTemp;
    float temperature;
    
    // Reset
    void reset();

private:
    TwoWire *_wire;
    SPIClass *_spi;
    int8_t _cs;
    uint8_t _address;
    bool _useSPI;
    uint32_t _spiFreq;
    
    lis3mdl_range_t _range;
    float _sensitivity;
    
    void updateSensitivity();
    
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count);
};

#endif // LIS3MDL_H
