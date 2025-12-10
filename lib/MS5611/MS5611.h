/**
 * @file MS5611.h
 * @brief Driver for the MS561101BA03-50 barometric pressure sensor
 * 
 * Supports both I2C and SPI communication.
 * I2C Address: 0x76 (CSB=high) or 0x77 (CSB=low)
 */

#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// I2C Addresses
#define MS5611_ADDRESS_HIGH     0x76  // CSB to VCC
#define MS5611_ADDRESS_LOW      0x77  // CSB to GND
#define MS5611_DEFAULT_ADDRESS  MS5611_ADDRESS_HIGH

// Commands
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_PROM_READ    0xA0  // Base address for PROM read (add addr * 2)
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_CONV_D1      0x40  // Pressure conversion
#define MS5611_CMD_CONV_D2      0x50  // Temperature conversion

// Oversampling ratio (OSR) settings
typedef enum {
    MS5611_OSR_256  = 0x00,  // 0.6 ms conversion time
    MS5611_OSR_512  = 0x02,  // 1.2 ms
    MS5611_OSR_1024 = 0x04,  // 2.3 ms
    MS5611_OSR_2048 = 0x06,  // 4.6 ms
    MS5611_OSR_4096 = 0x08   // 9.1 ms
} ms5611_osr_t;

class MS5611 {
public:
    MS5611();
    
    // Initialization
    bool begin_I2C(uint8_t i2c_address = MS5611_DEFAULT_ADDRESS, TwoWire *wire = &Wire);
    bool begin_SPI(int8_t cs_pin, SPIClass *spi = &SPI, uint32_t frequency = 1000000);
    
    // Configuration
    void setOversamplingRatio(ms5611_osr_t osr);
    ms5611_osr_t getOversamplingRatio();
    
    // Reset
    void reset();
    
    // Data reading (blocking)
    bool read();
    
    // Non-blocking reading
    void startPressureConversion();
    void startTemperatureConversion();
    bool conversionComplete();
    uint32_t readADC();
    void calculate();
    
    // Calculated values
    float pressure;      // Pressure in mbar (hPa)
    float temperature;   // Temperature in °C
    float altitude;      // Altitude in meters (requires sea level pressure)
    
    // Raw ADC values
    uint32_t rawPressure;
    uint32_t rawTemperature;
    
    // Set sea level pressure for altitude calculation (in mbar)
    void setSeaLevelPressure(float slp);
    float getSeaLevelPressure();

private:
    TwoWire *_wire;
    SPIClass *_spi;
    int8_t _cs;
    uint8_t _address;
    bool _useSPI;
    uint32_t _spiFreq;
    
    ms5611_osr_t _osr;
    float _seaLevelPressure;
    
    // Calibration coefficients from PROM
    uint16_t C[7];  // C[0] not used, C[1]-C[6] are calibration values
    
    // Conversion delay based on OSR (in microseconds)
    uint32_t _conversionDelay;
    uint32_t _conversionStart;
    
    bool readCalibration();
    void updateConversionDelay();
    
    void sendCommand(uint8_t cmd);
    uint16_t readPROM(uint8_t addr);
    uint32_t readADCValue();
};

#endif // MS5611_H
