/**
 * @file LSM6DSV32X.cpp
 * @brief LSM6DSV32XTR 6-axis IMU driver implementation
 */

#include "LSM6DSV32X.h"

LSM6DSV32X::LSM6DSV32X() {
    _wire = nullptr; _spi = nullptr; _cs = -1; _useSPI = false;
    _accSens = 0.000122f * 9.80665f;  // 4g default
    _gyroSens = 4.375f * 0.001f * 0.017453f;  // 125dps default
}

bool LSM6DSV32X::begin_I2C(uint8_t addr, TwoWire *wire) {
    _wire = wire; _addr = addr; _useSPI = false;
    _wire->begin();
    
    if (readReg(LSM6DSV32X_WHO_AM_I) != LSM6DSV32X_CHIP_ID) return false;
    
    reset();
    delay(10);
    
    // BDU enable, IF_INC enable
    writeReg(LSM6DSV32X_CTRL3_C, 0x44);
    
    // Set default ODR and ranges
    setAccRange(LSM6DSV32X_ACC_32G);
    setGyroRange(LSM6DSV32X_GYRO_4000DPS);
    setAccODR(LSM6DSV32X_ODR_960HZ);
    setGyroODR(LSM6DSV32X_ODR_960HZ);
    
    return true;
}

bool LSM6DSV32X::begin_SPI(int8_t cs, SPIClass *spi, uint32_t freq) {
    _spi = spi; _cs = cs; _useSPI = true; _spiFreq = freq;
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    _spi->begin();
    
    if (readReg(LSM6DSV32X_WHO_AM_I) != LSM6DSV32X_CHIP_ID) return false;
    
    reset();
    delay(10);
    
    writeReg(LSM6DSV32X_CTRL3_C, 0x44);
    setAccRange(LSM6DSV32X_ACC_32G);
    setGyroRange(LSM6DSV32X_GYRO_4000DPS);
    setAccODR(LSM6DSV32X_ODR_960HZ);
    setGyroODR(LSM6DSV32X_ODR_960HZ);
    
    return true;
}

void LSM6DSV32X::setAccRange(lsm6dsv32x_acc_range_t range) {
    uint8_t reg = readReg(LSM6DSV32X_CTRL1_XL);
    reg = (reg & 0xFC) | ((uint8_t)range);
    writeReg(LSM6DSV32X_CTRL1_XL, reg);
    
    float sens[] = {0.000122f, 0.000244f, 0.000488f, 0.000976f};
    _accSens = sens[range] * 9.80665f;
}

void LSM6DSV32X::setGyroRange(lsm6dsv32x_gyro_range_t range) {
    uint8_t reg = readReg(LSM6DSV32X_CTRL2_G);
    reg = (reg & 0xF0) | ((uint8_t)range);
    writeReg(LSM6DSV32X_CTRL2_G, reg);
    
    float sens[] = {4.375f, 8.75f, 17.5f, 35.0f, 70.0f, 140.0f};
    _gyroSens = sens[range] * 0.001f * 0.017453f;  // mdps to rad/s
}

void LSM6DSV32X::setAccODR(lsm6dsv32x_odr_t odr) {
    uint8_t reg = readReg(LSM6DSV32X_CTRL1_XL);
    reg = (reg & 0x0F) | ((uint8_t)odr << 4);
    writeReg(LSM6DSV32X_CTRL1_XL, reg);
}

void LSM6DSV32X::setGyroODR(lsm6dsv32x_odr_t odr) {
    uint8_t reg = readReg(LSM6DSV32X_CTRL2_G);
    reg = (reg & 0x0F) | ((uint8_t)odr << 4);
    writeReg(LSM6DSV32X_CTRL2_G, reg);
}

bool LSM6DSV32X::read() {
    uint8_t buf[14];
    readRegs(LSM6DSV32X_OUT_TEMP_L, buf, 14);
    
    int16_t rawTemp = (int16_t)(buf[1] << 8 | buf[0]);
    temperature = 25.0f + (float)rawTemp / 256.0f;
    
    rawGyroX = (int16_t)(buf[3] << 8 | buf[2]);
    rawGyroY = (int16_t)(buf[5] << 8 | buf[4]);
    rawGyroZ = (int16_t)(buf[7] << 8 | buf[6]);
    
    rawAccX = (int16_t)(buf[9] << 8 | buf[8]);
    rawAccY = (int16_t)(buf[11] << 8 | buf[10]);
    rawAccZ = (int16_t)(buf[13] << 8 | buf[12]);
    
    gyroX = rawGyroX * _gyroSens;
    gyroY = rawGyroY * _gyroSens;
    gyroZ = rawGyroZ * _gyroSens;
    
    accelX = rawAccX * _accSens;
    accelY = rawAccY * _accSens;
    accelZ = rawAccZ * _accSens;
    
    return true;
}

void LSM6DSV32X::reset() {
    writeReg(LSM6DSV32X_CTRL3_C, 0x01);  // SW_RESET
    delay(10);
}

uint8_t LSM6DSV32X::readReg(uint8_t reg) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(reg | 0x80);
        uint8_t val = _spi->transfer(0x00);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
        return val;
    } else {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_addr, (uint8_t)1);
        return _wire->read();
    }
}

void LSM6DSV32X::writeReg(uint8_t reg, uint8_t val) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(reg & 0x7F);
        _spi->transfer(val);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
}

void LSM6DSV32X::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(reg | 0x80);
        for (uint8_t i = 0; i < len; i++) buf[i] = _spi->transfer(0x00);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_addr, len);
        for (uint8_t i = 0; i < len; i++) buf[i] = _wire->read();
    }
}
