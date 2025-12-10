/**
 * @file BMI088.cpp
 * @brief BMI088 6-axis IMU driver implementation
 */

#include "BMI088.h"

BMI088::BMI088() {
    _wire = nullptr; _spi = nullptr;
    _accCs = _gyroCs = -1;
    _useSPI = false;
    _accSens = 1.0f / 10920.0f * 9.80665f;  // 3g default
    _gyroSens = 1.0f / 16.384f * 0.017453f;  // 2000dps default
}

bool BMI088::begin_I2C(uint8_t acc_addr, uint8_t gyro_addr, TwoWire *wire) {
    _wire = wire; _accAddr = acc_addr; _gyroAddr = gyro_addr; _useSPI = false;
    _wire->begin();
    
    // Reset both
    reset();
    delay(50);
    
    // Check chip IDs
    if (readAccReg(BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) return false;
    if (readGyroReg(BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) return false;
    
    // Enable accelerometer
    writeAccReg(BMI088_ACC_PWR_CONF, 0x00);  // Active mode
    delay(1);
    writeAccReg(BMI088_ACC_PWR_CTRL, 0x04);  // Enable accel
    delay(50);
    
    // Configure
    setAccRange(BMI088_ACC_RANGE_24G);
    setGyroRange(BMI088_GYRO_RANGE_2000DPS);
    
    return true;
}

bool BMI088::begin_SPI(int8_t acc_cs, int8_t gyro_cs, SPIClass *spi, uint32_t freq) {
    _spi = spi; _accCs = acc_cs; _gyroCs = gyro_cs; _useSPI = true; _spiFreq = freq;
    pinMode(_accCs, OUTPUT); pinMode(_gyroCs, OUTPUT);
    digitalWrite(_accCs, HIGH); digitalWrite(_gyroCs, HIGH);
    _spi->begin();
    
    reset();
    delay(50);
    
    if (readAccReg(BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) return false;
    if (readGyroReg(BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) return false;
    
    writeAccReg(BMI088_ACC_PWR_CONF, 0x00);
    delay(1);
    writeAccReg(BMI088_ACC_PWR_CTRL, 0x04);
    delay(50);
    
    setAccRange(BMI088_ACC_RANGE_24G);
    setGyroRange(BMI088_GYRO_RANGE_2000DPS);
    
    return true;
}

void BMI088::setAccRange(bmi088_acc_range_t range) {
    writeAccReg(BMI088_ACC_RANGE, (uint8_t)range);
    switch(range) {
        case BMI088_ACC_RANGE_3G:  _accSens = 9.80665f / 10920.0f; break;
        case BMI088_ACC_RANGE_6G:  _accSens = 9.80665f / 5460.0f; break;
        case BMI088_ACC_RANGE_12G: _accSens = 9.80665f / 2730.0f; break;
        case BMI088_ACC_RANGE_24G: _accSens = 9.80665f / 1365.0f; break;
    }
}

void BMI088::setGyroRange(bmi088_gyro_range_t range) {
    writeGyroReg(BMI088_GYRO_RANGE, (uint8_t)range);
    float dps_sens[] = {16.384f, 32.768f, 65.536f, 131.072f, 262.144f};
    _gyroSens = 0.017453f / dps_sens[range];
}

bool BMI088::read() {
    uint8_t buf[6];
    
    // Read accel (need dummy read for SPI)
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_accCs, LOW);
        _spi->transfer(BMI088_ACC_X_LSB | 0x80);
        _spi->transfer(0x00);  // Dummy
        for (int i = 0; i < 6; i++) buf[i] = _spi->transfer(0x00);
        digitalWrite(_accCs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_accAddr);
        _wire->write(BMI088_ACC_X_LSB);
        _wire->endTransmission(false);
        _wire->requestFrom(_accAddr, (uint8_t)6);
        for (int i = 0; i < 6; i++) buf[i] = _wire->read();
    }
    rawAccX = (int16_t)(buf[1] << 8 | buf[0]);
    rawAccY = (int16_t)(buf[3] << 8 | buf[2]);
    rawAccZ = (int16_t)(buf[5] << 8 | buf[4]);
    accelX = rawAccX * _accSens;
    accelY = rawAccY * _accSens;
    accelZ = rawAccZ * _accSens;
    
    // Read gyro
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_gyroCs, LOW);
        _spi->transfer(BMI088_GYRO_X_LSB | 0x80);
        for (int i = 0; i < 6; i++) buf[i] = _spi->transfer(0x00);
        digitalWrite(_gyroCs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_gyroAddr);
        _wire->write(BMI088_GYRO_X_LSB);
        _wire->endTransmission(false);
        _wire->requestFrom(_gyroAddr, (uint8_t)6);
        for (int i = 0; i < 6; i++) buf[i] = _wire->read();
    }
    rawGyroX = (int16_t)(buf[1] << 8 | buf[0]);
    rawGyroY = (int16_t)(buf[3] << 8 | buf[2]);
    rawGyroZ = (int16_t)(buf[5] << 8 | buf[4]);
    gyroX = rawGyroX * _gyroSens;
    gyroY = rawGyroY * _gyroSens;
    gyroZ = rawGyroZ * _gyroSens;
    
    return true;
}

void BMI088::reset() {
    writeAccReg(BMI088_ACC_SOFTRESET, 0xB6);
    delay(1);
    writeGyroReg(BMI088_GYRO_SOFTRESET, 0xB6);
    delay(30);
}

uint8_t BMI088::readAccReg(uint8_t reg) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_accCs, LOW);
        _spi->transfer(reg | 0x80);
        _spi->transfer(0x00);  // Dummy
        uint8_t val = _spi->transfer(0x00);
        digitalWrite(_accCs, HIGH);
        _spi->endTransaction();
        return val;
    } else {
        _wire->beginTransmission(_accAddr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_accAddr, (uint8_t)1);
        return _wire->read();
    }
}

void BMI088::writeAccReg(uint8_t reg, uint8_t val) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_accCs, LOW);
        _spi->transfer(reg & 0x7F);
        _spi->transfer(val);
        digitalWrite(_accCs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_accAddr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
}

uint8_t BMI088::readGyroReg(uint8_t reg) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_gyroCs, LOW);
        _spi->transfer(reg | 0x80);
        uint8_t val = _spi->transfer(0x00);
        digitalWrite(_gyroCs, HIGH);
        _spi->endTransaction();
        return val;
    } else {
        _wire->beginTransmission(_gyroAddr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_gyroAddr, (uint8_t)1);
        return _wire->read();
    }
}

void BMI088::writeGyroReg(uint8_t reg, uint8_t val) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_gyroCs, LOW);
        _spi->transfer(reg & 0x7F);
        _spi->transfer(val);
        digitalWrite(_gyroCs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_gyroAddr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
}
