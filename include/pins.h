/**
 * @file pins.h
 * @brief Centralized pin configuration for STM32H723VGT6 custom PCB
 * 
 * ============================================================================
 * MODIFY THESE DEFINITIONS TO MATCH YOUR PCB DESIGN
 * ============================================================================
 */

#ifndef PINS_H
#define PINS_H

// =============================================================================
// I2C BUS CONFIGURATION
// =============================================================================
#define I2C1_SDA_PIN        PB9     // TODO: Primary I2C SDA
#define I2C1_SCL_PIN        PB8     // TODO: Primary I2C SCL

// Secondary I2C bus (if used)
#define I2C2_SDA_PIN        PB11    // TODO: Secondary I2C SDA  
#define I2C2_SCL_PIN        PB10    // TODO: Secondary I2C SCL

// =============================================================================
// SPI BUS CONFIGURATION
// =============================================================================
#define SPI1_SCK_PIN        PA5     // TODO: SPI1 Clock
#define SPI1_MISO_PIN       PA6     // TODO: SPI1 MISO
#define SPI1_MOSI_PIN       PA7     // TODO: SPI1 MOSI

// Secondary SPI bus (if used)
#define SPI2_SCK_PIN        PB13    // TODO: SPI2 Clock
#define SPI2_MISO_PIN       PB14    // TODO: SPI2 MISO
#define SPI2_MOSI_PIN       PB15    // TODO: SPI2 MOSI

// =============================================================================
// SENSOR CHIP SELECT PINS (for SPI mode)
// =============================================================================
#define LIS3MDL_CS_PIN      PA4     // TODO: LIS3MDLTR Magnetometer CS
#define MS5611_CS_PIN       PA3     // TODO: MS5611 Barometer CS
#define BMI088_ACC_CS_PIN   PA2     // TODO: BMI088 Accelerometer CS
#define BMI088_GYRO_CS_PIN  PA1     // TODO: BMI088 Gyroscope CS
#define LSM6DSV_CS_PIN      PA0     // TODO: LSM6DSV32X IMU CS
#define ADXL375_CS_PIN      PC0     // TODO: ADXL375 Accelerometer CS

// =============================================================================
// I2C ADDRESSES
// =============================================================================
// LIS3MDLTR Magnetometer
// Options: 0x1C (SA1 pin = GND), 0x1E (SA1 pin = VCC)
#define LIS3MDL_I2C_ADDR    0x1C

// MS561101BA03-50 Barometer
// Options: 0x76 (CSB pin = VCC), 0x77 (CSB pin = GND)
#define MS5611_I2C_ADDR     0x76

// BMI088 IMU (has separate accel and gyro addresses)
// Accel: 0x18 (SDO1 = GND), 0x19 (SDO1 = VCC)
// Gyro:  0x68 (SDO2 = GND), 0x69 (SDO2 = VCC)
#define BMI088_ACC_I2C_ADDR     0x18
#define BMI088_GYRO_I2C_ADDR    0x68

// LSM6DSV32XTR IMU
// Options: 0x6A (SA0 = GND), 0x6B (SA0 = VCC)
#define LSM6DSV_I2C_ADDR    0x6A

// ADXL375BCCZ Accelerometer
// Options: 0x53 (ALT ADDRESS = GND), 0x1D (ALT ADDRESS = VCC)
#define ADXL375_I2C_ADDR    0x53

// =============================================================================
// GPS UART CONFIGURATION (MAX-M8Q-0)
// =============================================================================
#define GPS_UART            Serial2  // TODO: UART peripheral for GPS
#define GPS_TX_PIN          PA2      // TODO: MCU TX -> GPS RX
#define GPS_RX_PIN          PA3      // TODO: MCU RX <- GPS TX
#define GPS_BAUDRATE        9600     // Default MAX-M8Q baud rate

// =============================================================================
// LED / DEBUG PINS
// =============================================================================
#define LED_PIN             PC13     // TODO: Status LED
#define DEBUG_UART          Serial1  // TODO: Debug UART

// =============================================================================
// INTERRUPT PINS (optional, for data-ready interrupts)
// =============================================================================
#define LIS3MDL_INT_PIN     PB0      // TODO: Magnetometer interrupt
#define BMI088_INT1_PIN     PB1      // TODO: BMI088 interrupt 1
#define BMI088_INT2_PIN     PB2      // TODO: BMI088 interrupt 2
#define LSM6DSV_INT1_PIN    PB3      // TODO: LSM6DSV interrupt 1
#define LSM6DSV_INT2_PIN    PB4      // TODO: LSM6DSV interrupt 2
#define ADXL375_INT1_PIN    PB5      // TODO: ADXL375 interrupt 1
#define ADXL375_INT2_PIN    PB6      // TODO: ADXL375 interrupt 2

#endif // PINS_H
