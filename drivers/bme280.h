// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2024 Witold Olechowski.
 */

#ifndef __BME280_H
#define __BME280_H

#if !HAL_USE_I2C
#error "BME280 requires HAL_USE_I2C"
#endif

// BME280 I2C adress,
#define BME280_ADDR_G 0x76 // I2C address when SDO connected to GND
#define BME280_ADDR_V 0x77 // I2C address when SDO connected to VDDIO

// BME280 registers
#define BME280_REG_CALIB00 0x88	// Calibration data calib00
#define BME280_REG_CALIB25 0xA1	// Calibration data calib25
#define BME280_REG_ID 0xD0			// Chip ID
#define BME280_REG_RESET 0xE0		// Software reset control register
#define BME280_REG_CALIB26 0xE1	// Calibration data calib26
#define BME280_REG_CTRL_HUM 0xF2	// Humidity measure control register
#define BME280_REG_STATUS 0xF3		// Device status register
#define BME280_REG_CTRL_MEAS 0xF4	// Pressure and temperature measure control register
#define BME280_REG_CONFIG 0xF5		// Configuration register
#define BME280_REG_PRESS_MSB 0xF7	// Pressure readings MSB
#define BME280_REG_PRESS_LSB 0xF8	// Pressure readings LSB
#define BME280_REG_PRESS_XLSB 0xF9 // Pressure readings XLSB
#define BME280_REG_TEMP_MSB 0xFA	// Temperature data MSB
#define BME280_REG_TEMP_LSB 0xFB	// Temperature data LSB
#define BME280_REG_TEMP_XLSB 0xFC	// Temperature data XLSB
#define BME280_REG_HUM_MSB 0xFD	// Humidity data MSB
#define BME280_REG_HUM_LSB 0xFE	// Humidity data LSB

// BME280 register bits

// Software reset
#define BME280_SOFT_RESET_KEY 0xB6

// Humidity oversampling control register (0xF2)
#define BME280_OSRS_H_MSK 0x07	 // 'osrs_h' mask
#define BME280_OSRS_H_SKIP 0x00 // Skipped
#define BME280_OSRS_H_x1 0x01	 // x1
#define BME280_OSRS_H_x2 0x02	 // x2
#define BME280_OSRS_H_x4 0x03	 // x4
#define BME280_OSRS_H_x8 0x04	 // x8
#define BME280_OSRS_H_x16 0x05	 // x16

// Status register (0xF3)
#define BME280_STATUS_MSK 0x09		  // Mask to clear unused bits
#define BME280_STATUS_MEASURING 0x08 // Status register bit 3 (conversion is running)
#define BME280_STATUS_IM_UPDATE 0x01 // Status register bit 0 (NVM data being copied to image registers)

// Pressure and temperature control register (0xF4)
// Temperature oversampling (osrs_t [7:5])
#define BME280_OSRS_T_MSK 0xE0	 // 'osrs_t' mask
#define BME280_OSRS_T_SKIP 0x00 // Skipped
#define BME280_OSRS_T_x1 0x20	 // x1
#define BME280_OSRS_T_x2 0x40	 // x2
#define BME280_OSRS_T_x4 0x60	 // x4
#define BME280_OSRS_T_x8 0x80	 // x8
#define BME280_OSRS_T_x16 0xA0	 // x16
// Pressure oversampling (osrs_p [4:2])
#define BME280_OSRS_P_MSK 0x1C	 // 'osrs_p' mask
#define BME280_OSRS_P_SKIP 0x00 // Skipped
#define BME280_OSRS_P_x1 0x04	 // x1
#define BME280_OSRS_P_x2 0x08	 // x2
#define BME280_OSRS_P_x4 0x0C	 // x4
#define BME280_OSRS_P_x8 0x10	 // x8
#define BME280_OSRS_P_x16 0x14	 // x16
// Sensor mode of the device (mode [1:0])
#define BME280_MODE_MSK 0x03	 // 'mode' mask
#define BME280_MODE_SLEEP 0x00	 // Sleep mode
#define BME280_MODE_FORCED 0x01 // Forced mode
#define BME280_MODE_NORMAL 0x03 // Normal mode

// Configuration register: set rate, filter and interface options (0xF5)
// Inactive duration in normal mode (t_sb [7:5])
#define BME280_STBY_MSK 0xE0	 // 't_sb' mask
#define BME280_STBY_0p5ms 0x00	 // 0.5ms
#define BME280_STBY_62p5ms 0x20 // 62.5ms
#define BME280_STBY_125ms 0x40	 // 125ms
#define BME280_STBY_250ms 0x60	 // 250ms
#define BME280_STBY_500ms 0x80	 // 500ms
#define BME280_STBY_1s 0xA0	 // 1s
#define BME280_STBY_10ms 0xC0	 // 10ms
#define BME280_STBY_20ms 0xE0	 // 20ms
// Time constant of the IIR filter (filter [4:2])
#define BME280_FILTER_MSK 0x1C // 'filter' mask
#define BME280_FILTER_OFF 0x00 // Off
#define BME280_FILTER_2 0x04	// 2
#define BME280_FILTER_4 0x08	// 4
#define BME280_FILTER_8 0x0C	// 8
#define BME280_FILTER_16 0x10	// 16

// BME280 function result
typedef enum
{
	BME280_ERROR = 0,
	BME280_SUCCESS = 1
} BME280_RESULT;

// Compensation parameters structure
typedef struct
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
} BME280_Compensation_TypeDef;

// Initialization structure
typedef struct _bme_t
{
	float temperature;
	float humidity;
	float presure;
} bme_t;

struct BME280VMT
{
	BME280_RESULT (*readUTPH)(void *ip);
	float (*readAltitude)(void *ip, float seaLevelhPa);
};

typedef enum
{
	BME280_UNINIT = 0,
	BME280_STOP = 1,
	BME280_READY = 2,
} bme280_state_t;

typedef struct
{
	I2CDriver *i2cp;
	const I2CConfig *i2ccfg;
	uint8_t addr;
} BME280Config;

typedef struct BME280Driver
{
	const struct BME280VMT *vmt;
	bme280_state_t state;
	const BME280Config *config;
	bme_t data;
} BME280Driver;

void bme280ObjectInit(BME280Driver *devp);
void bme280Start(BME280Driver *devp, const BME280Config *config);
void bme280Stop(BME280Driver *devp);

#endif // __BME280_H