// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2024 Witold Olechowski.
 */

#ifndef DRIVERS_AS5048B_H_
#define DRIVERS_AS5048B_H_

#include "hal.h"

#if !HAL_USE_I2C
#error "AS5048B requires HAL_USE_I2C"
#endif

// Default addresses for AS5048B
#define AS5048_ADDRESS 0x40 // 0b10000 + ( A1 & A2 to GND)
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 // bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 // bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC   // bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD   // bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE   // bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF   // bits 0..5
#define AS5048B_RESOLUTION 16384.0 // 14 bits

// Moving average is calculated on Sine et Cosine values of the angle to provide an extrapolated accurate angle value.
#define EXP_MOVAVG_N 5	  // history length impact on moving average impact - keep in mind the moving average will be impacted by the measurement frequency too
#define EXP_MOVAVG_LOOP 1 // number of measurements before starting mobile Average - starting with a simple average - 1 allows a quick start. Value must be 1 minimum

// unit consts - just to make the units more readable
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

struct AS5048BVMT
{
	void (*setClockWise)(bool cw);					   // set clockwise counting, default is false (native sensor)
	void (*progRegister)(void *ip, uint8_t regVal);	   // nothing so far - manipulate the OTP register
	void (*doProg)(void *ip);						   // progress programming slave address OTP
	void (*doProgZero)(void *ip);					   // progress programming zero position OTP
	void (*addressRegW)(void *ip, uint8_t regVal);	   // change the chip address
	uint8_t (*addressRegR)(void *ip);				   // read chip address
	void (*setZeroReg)(void *ip);					   // set Zero to current angle position
	void (*zeroRegW)(void *ip, uint16_t regVal);	   // write Zero register value
	uint16_t (*zeroRegR)(void *ip);					   // read Zero register value
	uint16_t (*angleRegR)(void *ip);				   // read raw value of the angle register
	uint16_t (*magnitudeR)(void *ip);				   // read current magnitude
	double (*angleR)(void *ip, int unit, bool newVal); // read current angle or get last measure with unit conversion : RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
	uint8_t (*getAutoGain)(void *ip);
	uint8_t (*getDiagReg)(void *ip);
	void (*updateMovingAvgExp)(void *ip); // measure the current angle and feed the Exponential Moving Average calculation
	double (*getMovingAvgExp)(int unit);  // get Exponential Moving Average calculation
	void (*resetMovingAvgExp)(void);	  // reset Exponential Moving Average calculation values
};

typedef enum
{
	AS5048B_UNINIT = 0,
	AS5048B_STOP = 1,
	AS5048B_READY = 2,
} as5048b_state_t;

typedef struct
{
	I2CDriver *i2cp;
	const I2CConfig *i2ccfg;
	uint8_t addr;
} AS5048BConfig;

typedef struct AS5048BDriver
{
	const struct AS5048BVMT *vmt;
	as5048b_state_t state;
	AS5048BConfig *config;
} AS5048BDriver;

void as5048BObjectInit(AS5048BDriver *devp);
void as5048BStart(AS5048BDriver *devp, AS5048BConfig *config);
void as5048BStop(AS5048BDriver *devp);

#endif /* DRIVERS_AS5048B_H_ */
