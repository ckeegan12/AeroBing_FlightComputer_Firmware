/**
 * @file MS5611.cpp
 * @brief Driver implementation for the MS561101BA03-50 barometric pressure sensor
 */

#include "MS5611.h"

MS5611::MS5611() {
    _wire = nullptr;
    _spi = nullptr;
    _cs = -1;
    _useSPI = false;
    _osr = MS5611_OSR_4096;
    _seaLevelPressure = 1013.25f;  // Standard sea level pressure in mbar
    _conversionDelay = 10000;  // 10ms for OSR_4096
    _conversionStart = 0;
    
    pressure = 0.0f;
    temperature = 0.0f;
    altitude = 0.0f;
    rawPressure = 0;
    rawTemperature = 0;
    
    for (int i = 0; i < 7; i++) {
        C[i] = 0;
    }
}

bool MS5611::begin_I2C(uint8_t i2c_address, TwoWire *wire) {
    _wire = wire;
    _address = i2c_address;
    _useSPI = false;
    
    _wire->begin();
    
    // Reset device
    reset();
    delay(10);
    
    // Read calibration data
    if (!readCalibration()) {
        return false;
    }
    
    updateConversionDelay();
    return true;
}

bool MS5611::begin_SPI(int8_t cs_pin, SPIClass *spi, uint32_t frequency) {
    _spi = spi;
    _cs = cs_pin;
    _useSPI = true;
    _spiFreq = frequency;
    
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    
    _spi->begin();
    
    // Reset device
    reset();
    delay(10);
    
    // Read calibration data
    if (!readCalibration()) {
        return false;
    }
    
    updateConversionDelay();
    return true;
}

void MS5611::reset() {
    sendCommand(MS5611_CMD_RESET);
    delay(3);  // Reset requires 2.8ms
}

bool MS5611::readCalibration() {
    // Read PROM coefficients C1-C6
    for (uint8_t i = 0; i <= 6; i++) {
        C[i] = readPROM(i);
    }
    
    // Basic validation - C1 shouldn't be 0 or 0xFFFF
    if (C[1] == 0 || C[1] == 0xFFFF) {
        return false;
    }
    
    return true;
}

void MS5611::setOversamplingRatio(ms5611_osr_t osr) {
    _osr = osr;
    updateConversionDelay();
}

ms5611_osr_t MS5611::getOversamplingRatio() {
    return _osr;
}

void MS5611::updateConversionDelay() {
    switch (_osr) {
        case MS5611_OSR_256:
            _conversionDelay = 600;
            break;
        case MS5611_OSR_512:
            _conversionDelay = 1200;
            break;
        case MS5611_OSR_1024:
            _conversionDelay = 2300;
            break;
        case MS5611_OSR_2048:
            _conversionDelay = 4600;
            break;
        case MS5611_OSR_4096:
        default:
            _conversionDelay = 9100;
            break;
    }
}

bool MS5611::read() {
    // Start temperature conversion
    startTemperatureConversion();
    delayMicroseconds(_conversionDelay + 100);
    rawTemperature = readADCValue();
    
    // Start pressure conversion
    startPressureConversion();
    delayMicroseconds(_conversionDelay + 100);
    rawPressure = readADCValue();
    
    // Calculate compensated values
    calculate();
    
    return true;
}

void MS5611::startPressureConversion() {
    sendCommand(MS5611_CMD_CONV_D1 | (uint8_t)_osr);
    _conversionStart = micros();
}

void MS5611::startTemperatureConversion() {
    sendCommand(MS5611_CMD_CONV_D2 | (uint8_t)_osr);
    _conversionStart = micros();
}

bool MS5611::conversionComplete() {
    return (micros() - _conversionStart) >= _conversionDelay;
}

uint32_t MS5611::readADC() {
    return readADCValue();
}

void MS5611::calculate() {
    // Temperature calculation according to datasheet
    int64_t dT = (int64_t)rawTemperature - ((int64_t)C[5] << 8);
    int64_t TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);
    
    // Pressure calculation
    int64_t OFF = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);
    
    // Second order temperature compensation
    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    
    if (TEMP < 2000) {
        // Low temperature
        T2 = (dT * dT) >> 31;
        int64_t temp_minus_2000 = TEMP - 2000;
        OFF2 = (5 * temp_minus_2000 * temp_minus_2000) >> 1;
        SENS2 = (5 * temp_minus_2000 * temp_minus_2000) >> 2;
        
        if (TEMP < -1500) {
            // Very low temperature
            int64_t temp_plus_1500 = TEMP + 1500;
            OFF2 += 7 * temp_plus_1500 * temp_plus_1500;
            SENS2 += (11 * temp_plus_1500 * temp_plus_1500) >> 1;
        }
    }
    
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    
    int64_t P = (((int64_t)rawPressure * SENS >> 21) - OFF) >> 15;
    
    temperature = (float)TEMP / 100.0f;
    pressure = (float)P / 100.0f;  // Convert to mbar (hPa)
    
    // Calculate altitude using barometric formula
    // h = 44330 * (1 - (P/P0)^0.1903)
    altitude = 44330.0f * (1.0f - pow(pressure / _seaLevelPressure, 0.1903f));
}

void MS5611::setSeaLevelPressure(float slp) {
    _seaLevelPressure = slp;
}

float MS5611::getSeaLevelPressure() {
    return _seaLevelPressure;
}

void MS5611::sendCommand(uint8_t cmd) {
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        _spi->transfer(cmd);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_address);
        _wire->write(cmd);
        _wire->endTransmission();
    }
}

uint16_t MS5611::readPROM(uint8_t addr) {
    uint16_t value = 0;
    
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        _spi->transfer(MS5611_CMD_PROM_READ + (addr << 1));
        value = (uint16_t)_spi->transfer(0x00) << 8;
        value |= _spi->transfer(0x00);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_address);
        _wire->write(MS5611_CMD_PROM_READ + (addr << 1));
        _wire->endTransmission(false);
        _wire->requestFrom(_address, (uint8_t)2);
        value = (uint16_t)_wire->read() << 8;
        value |= _wire->read();
    }
    
    return value;
}

uint32_t MS5611::readADCValue() {
    uint32_t value = 0;
    
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        _spi->transfer(MS5611_CMD_ADC_READ);
        value = (uint32_t)_spi->transfer(0x00) << 16;
        value |= (uint32_t)_spi->transfer(0x00) << 8;
        value |= _spi->transfer(0x00);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    } else {
        _wire->beginTransmission(_address);
        _wire->write(MS5611_CMD_ADC_READ);
        _wire->endTransmission(false);
        _wire->requestFrom(_address, (uint8_t)3);
        value = (uint32_t)_wire->read() << 16;
        value |= (uint32_t)_wire->read() << 8;
        value |= _wire->read();
    }
    
    return value;
}
