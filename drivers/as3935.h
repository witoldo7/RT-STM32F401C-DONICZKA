// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2023 Witold Olechowski.
 */

#ifndef DRIVERS_AS3935_H_
#define DRIVERS_AS3935_H_

#include "hal.h"

#if !HAL_USE_I2C
#error "AS3935 requires HAL_USE_I2C"
#endif

#define INDOOR 0x12
#define OUTDOOR 0xE
#define DIRECT_COMMAND 0x96
#define UNKNOWN_ERROR 0xFF

enum AS3935_REGISTER_NAMES
{
	AFE_GAIN = 0x00,
	THRESHOLD,
	LIGHTNING_REG,
	INT_MASK_ANT,
	ENERGY_LIGHT_LSB,
	ENERGY_LIGHT_MSB,
	ENERGY_LIGHT_MMSB,
	DISTANCE,
	FREQ_DISP_IRQ,
	CALIB_TRCO = 0x3A,
	CALIB_SRCO = 0x3B,
	RESET_LIGHT = 0x3C,
	CALIB_RCO = 0x3D
};

enum AS3935_REGSTER_MASKS
{
	WIPE_ALL = 0x0,
	INT_MASK = 0xF,
	ENERGY_MASK = 0x1F,
	SPI_READ_M = 0x40,
	CALIB_MASK = 0x40,
	OSC_MASK = 0x1F,
	DISTANCE_MASK = 0x3F,
	DIV_MASK = 0x3F,
	NOISE_FLOOR_MASK = 0x8F,
	GAIN_MASK = 0xC1,
	STAT_MASK = 0xBF,
	DISTURB_MASK = 0xDF,
	LIGHT_MASK = 0xCF,
	SPIKE_MASK = 0xF0,
	THRESH_MASK = 0xF0,
	CAP_MASK = 0xF0,
	POWER_MASK = 0xFE
};

typedef enum INTERRUPT_STATUS
{
	NOISE_TO_HIGH = 0x01,
	DISTURBER_DETECT = 0x04,
	LIGHTNING = 0x08
} lightningStatus;

typedef enum
{
	AS3935_ADR_1 = 0,
	AS3935_ADR_2 = 1,
	AS3935_ADR_3 = 3
} i2c_as3935_addr_t;

typedef enum
{
	AS3935_UNINIT = 0,
	AS3935_STOP = 1,
	AS3935_READY = 2,
} as3935_state_t;

typedef struct
{
	I2CDriver *i2cp;
	const I2CConfig *i2ccfg;
	i2c_as3935_addr_t addr;
} AS3935Config;

struct AS3935VMT
{
	void (*powerDown)(void *ip);
	bool (*wakeUp)(void *ip);
	void (*setIndoorOutdoor)(void *ip, uint8_t _setting);
	uint8_t (*readIndoorOutdoor)(void *ip);
	void (*watchdogThreshold)(void *ip, uint8_t _sensitivity);
	uint8_t (*readWatchdogThreshold)(void *ip);
	void (*setNoiseLevel)(void *ip, uint8_t _floor);
	uint8_t (*readNoiseLevel)(void *ip);
	void (*spikeRejection)(void *ip, uint8_t _spSensitivity);
	uint8_t (*readSpikeRejection)(void *ip);
	void (*lightningThreshold)(void *ip, uint8_t _strikes);
	uint8_t (*readLightningThreshold)(void *ip);
	void (*clearStatistics)(void *ip, bool _clearStat);
	uint8_t (*readInterruptReg)(void *ip);
	void (*maskDisturber)(void *ip, bool _state);
	uint8_t (*readMaskDisturber)(void *ip);
	void (*changeDivRatio)(void *ip, uint8_t _divisionRatio);
	uint8_t (*readDivRatio)(void *ip);
	int8_t (*distanceToStorm)(void *ip);
	void (*displayOscillator)(void *ip, bool _state, uint8_t _osc);
	void (*tuneCap)(void *ip, uint8_t _farad);
	uint8_t (*readTuneCap)(void *ip);
	uint32_t (*lightningEnergy)(void *ip);
	bool (*calibrateOsc)(void *ip);
	void (*resetSettings)(void *ip);
};

typedef struct AS3935Driver
{
	const struct AS3935VMT *vmt;
	as3935_state_t state;
	const AS3935Config *config;
} AS3935Driver;

void as3935ObjectInit(AS3935Driver *devp);
void as3935Start(AS3935Driver *devp, const AS3935Config *config);
void as3935Stop(AS3935Driver *devp);

#endif /* DRIVERS_AS3935_H_ */
