/**
 * @file LIS3MDL.cpp
 * @brief Driver implementation for the LIS3MDLTR 3-axis magnetometer
 */

#include "LIS3MDL.h"

LIS3MDL::LIS3MDL() {
    _wire = nullptr;
    _spi = nullptr;
    _cs = -1;
    _useSPI = false;
    _range = LIS3MDL_RANGE_4_GAUSS;
    _sensitivity = 6842.0f; // LSB/gauss for ±4 gauss
    
    rawX = rawY = rawZ = 0;
    x = y = z = 0.0f;
    rawTemp = 0;
    temperature = 0.0f;
}

bool LIS3MDL::begin_I2C(uint8_t i2c_address, TwoWire *wire) {
    _wire = wire;
    _address = i2c_address;
    _useSPI = false;
    
    _wire->begin();
    
    // Check device ID
    uint8_t id = readRegister(LIS3MDL_REG_WHO_AM_I);
    if (id != LIS3MDL_DEVICE_ID) {
        return false;
    }
    
    // Reset and configure
    reset();
    delay(10);
    
    // Enable temperature sensor and set ODR to 80Hz, ultra-high performance XY
    writeRegister(LIS3MDL_REG_CTRL_REG1, 0xFC);  // TEMP_EN=1, OM=11, DO=111, FAST_ODR=0
    
    // Set full scale to ±4 gauss
    setRange(LIS3MDL_RANGE_4_GAUSS);
    
    // Continuous conversion mode
    setOperationMode(LIS3MDL_CONTINUOUSMODE);
    
    // Ultra-high performance mode for Z-axis
    writeRegister(LIS3MDL_REG_CTRL_REG4, 0x0C);  // OMZ=11
    
    // Block data update
    writeRegister(LIS3MDL_REG_CTRL_REG5, 0x40);  // BDU=1
    
    return true;
}

bool LIS3MDL::begin_SPI(int8_t cs_pin, SPIClass *spi, uint32_t frequency) {
    _spi = spi;
    _cs = cs_pin;
    _useSPI = true;
    _spiFreq = frequency;
    
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    
    _spi->begin();
    
    // Check device ID
    uint8_t id = readRegister(LIS3MDL_REG_WHO_AM_I);
    if (id != LIS3MDL_DEVICE_ID) {
        return false;
    }
    
    // Reset and configure
    reset();
    delay(10);
    
    // Enable temperature sensor and set ODR to 80Hz, ultra-high performance XY
    writeRegister(LIS3MDL_REG_CTRL_REG1, 0xFC);
    
    // Set full scale to ±4 gauss
    setRange(LIS3MDL_RANGE_4_GAUSS);
    
    // Continuous conversion mode
    setOperationMode(LIS3MDL_CONTINUOUSMODE);
    
    // Ultra-high performance mode for Z-axis
    writeRegister(LIS3MDL_REG_CTRL_REG4, 0x0C);
    
    // Block data update
    writeRegister(LIS3MDL_REG_CTRL_REG5, 0x40);
    
    return true;
}

void LIS3MDL::setRange(lis3mdl_range_t range) {
    _range = range;
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG2);
    reg &= 0x9F;  // Clear FS bits
    reg |= ((uint8_t)range << 5);
    writeRegister(LIS3MDL_REG_CTRL_REG2, reg);
    updateSensitivity();
}

lis3mdl_range_t LIS3MDL::getRange() {
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG2);
    return (lis3mdl_range_t)((reg >> 5) & 0x03);
}

void LIS3MDL::updateSensitivity() {
    switch (_range) {
        case LIS3MDL_RANGE_4_GAUSS:
            _sensitivity = 6842.0f;
            break;
        case LIS3MDL_RANGE_8_GAUSS:
            _sensitivity = 3421.0f;
            break;
        case LIS3MDL_RANGE_12_GAUSS:
            _sensitivity = 2281.0f;
            break;
        case LIS3MDL_RANGE_16_GAUSS:
            _sensitivity = 1711.0f;
            break;
    }
}

void LIS3MDL::setDataRate(lis3mdl_datarate_t rate) {
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG1);
    reg &= 0x03;  // Clear DO and FAST_ODR bits
    reg |= ((uint8_t)rate << 2);
    writeRegister(LIS3MDL_REG_CTRL_REG1, reg);
}

lis3mdl_datarate_t LIS3MDL::getDataRate() {
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG1);
    return (lis3mdl_datarate_t)((reg >> 2) & 0x0F);
}

void LIS3MDL::setPerformanceMode(lis3mdl_performancemode_t mode) {
    // Set XY performance mode in CTRL_REG1
    uint8_t reg1 = readRegister(LIS3MDL_REG_CTRL_REG1);
    reg1 &= 0x9F;  // Clear OM bits
    reg1 |= ((uint8_t)mode << 5);
    writeRegister(LIS3MDL_REG_CTRL_REG1, reg1);
    
    // Set Z performance mode in CTRL_REG4
    uint8_t reg4 = readRegister(LIS3MDL_REG_CTRL_REG4);
    reg4 &= 0xF3;  // Clear OMZ bits
    reg4 |= ((uint8_t)mode << 2);
    writeRegister(LIS3MDL_REG_CTRL_REG4, reg4);
}

lis3mdl_performancemode_t LIS3MDL::getPerformanceMode() {
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG1);
    return (lis3mdl_performancemode_t)((reg >> 5) & 0x03);
}

void LIS3MDL::setOperationMode(lis3mdl_operationmode_t mode) {
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG3);
    reg &= 0xFC;  // Clear MD bits
    reg |= (uint8_t)mode;
    writeRegister(LIS3MDL_REG_CTRL_REG3, reg);
}

lis3mdl_operationmode_t LIS3MDL::getOperationMode() {
    uint8_t reg = readRegister(LIS3MDL_REG_CTRL_REG3);
    return (lis3mdl_operationmode_t)(reg & 0x03);
}

bool LIS3MDL::dataReady() {
    uint8_t status = readRegister(LIS3MDL_REG_STATUS);
    return (status & 0x08);  // ZYXDA bit
}

bool LIS3MDL::read() {
    uint8_t buffer[6];
    
    // Read all 6 bytes (X, Y, Z - 2 bytes each)
    readRegisters(LIS3MDL_REG_OUT_X_L, buffer, 6);
    
    rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    // Convert to gauss
    x = (float)rawX / _sensitivity;
    y = (float)rawY / _sensitivity;
    z = (float)rawZ / _sensitivity;
    
    // Read temperature
    uint8_t tempBuffer[2];
    readRegisters(LIS3MDL_REG_TEMP_OUT_L, tempBuffer, 2);
    rawTemp = (int16_t)(tempBuffer[1] << 8 | tempBuffer[0]);
    temperature = 25.0f + ((float)rawTemp / 8.0f);  // 8 LSB/°C, offset at 25°C
    
    return true;
}

void LIS3MDL::reset() {
    writeRegister(LIS3MDL_REG_CTRL_REG2, 0x04);  // SOFT_RST bit
    delay(10);
}

uint8_t LIS3MDL::readRegister(uint8_t reg) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(reg | 0x80);  // Read bit
        uint8_t value = _spi->transfer(0x00);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
        return value;
    } else {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_address, (uint8_t)1);
        return _wire->read();
    }
}

void LIS3MDL::writeRegister(uint8_t reg, uint8_t value) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(reg & 0x7F);  // Write bit (clear MSB)
        _spi->transfer(value);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->write(value);
        _wire->endTransmission();
    }
}

void LIS3MDL::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(reg | 0xC0);  // Read bit + auto-increment
        for (uint8_t i = 0; i < count; i++) {
            buffer[i] = _spi->transfer(0x00);
        }
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_address);
        _wire->write(reg | 0x80);  // Auto-increment for I2C
        _wire->endTransmission(false);
        _wire->requestFrom(_address, count);
        for (uint8_t i = 0; i < count; i++) {
            buffer[i] = _wire->read();
        }
    }
}
