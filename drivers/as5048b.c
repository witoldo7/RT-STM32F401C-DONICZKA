// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2024 Witold Olechowski.
 */

#include "drivers/as5048b.h"
#include "math.h"

// variables
bool _clockWise;
uint8_t _addressRegVal;
uint16_t _zeroRegVal;
double _lastAngleRaw;
double _movingAvgExpAngle;
double _movingAvgExpSin;
double _movingAvgExpCos;
double _movingAvgExpAlpha;
int _movingAvgCountLoop;

// methods
static uint8_t readReg8(AS5048BDriver *drvp, uint8_t address);
static uint16_t readReg16(AS5048BDriver *drvp, uint8_t address); // 16 bit value got from 2x8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
static void writeReg(AS5048BDriver *drvp, uint8_t address, uint8_t value);
static double convertAngle(int unit, double angle); // RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
static double getExpAvgRawAngle(void);
void resetMovingAvgExp(void);
void zeroRegW(void *ip, uint16_t regVal);

/**************************************************************************/
/*!
	@brief  Set / unset clock wise counting - sensor counts CCW natively

	@params[in]
				bool cw - true: CW, false: CCW
	@returns
				none
*/
/**************************************************************************/
void setClockWise(bool cw)
{
	_clockWise = cw;
	_lastAngleRaw = 0.0;
	resetMovingAvgExp();
	return;
}

/**************************************************************************/
/*!
	@brief  writes OTP control register

	@params[in]
				unit8_t register value
	@returns
				none
*/
/**************************************************************************/
void progRegister(void *ip, uint8_t regVal)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	writeReg(drvp, AS5048B_PROG_REG, regVal);
	return;
}

/**************************************************************************/
/*!
	@brief  Burn values to the slave address OTP register

	@params[in]
				none
	@returns
				none
*/
/**************************************************************************/
void doProg(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	// enable special programming mode
	progRegister(drvp, 0xFD);
	chThdSleepMilliseconds(10);

	// set the burn bit: enables automatic programming procedure
	progRegister(drvp, 0x08);
	chThdSleepMilliseconds(10);

	// disable special programming mode
	progRegister(drvp, 0x00);
	chThdSleepMilliseconds(10);

	return;
}

/**************************************************************************/
/*!
	@brief  Burn values to the zero position OTP register

	@params[in]
				none
	@returns
				none
*/
/**************************************************************************/
void doProgZero(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	// this will burn the zero position OTP register like described in the datasheet
	// enable programming mode
	progRegister(drvp, 0x01);
	chThdSleepMilliseconds(10);

	// set the burn bit: enables automatic programming procedure
	progRegister(drvp, 0x08);
	chThdSleepMilliseconds(10);

	// read angle information (equals to 0)
	readReg16(drvp, AS5048B_ANGLMSB_REG);
	chThdSleepMilliseconds(10);

	// enable verification
	progRegister(drvp, 0x40);
	chThdSleepMilliseconds(10);

	// read angle information (equals to 0)
	readReg16(drvp, AS5048B_ANGLMSB_REG);
	chThdSleepMilliseconds(10);

	return;
}

/**************************************************************************/
/*!
	@brief  write I2C address value (5 bits) into the address register

	@params[in]
				unit8_t register value
	@returns
				none
*/
/**************************************************************************/
void addressRegW(void *ip, uint8_t regVal)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	// write the new chip address to the register
	writeReg(drvp, AS5048B_ADDR_REG, regVal);

	// update our chip address with our 5 programmable bits
	// the MSB is internally inverted, so we flip the leftmost bit
	drvp->config->addr = ((regVal << 2) | (drvp->config->addr & 0b11)) ^ (1 << 6);
	return;
}

/**************************************************************************/
/*!
	@brief  reads I2C address register value

	@params[in]
				none
	@returns
				uint8_t register value
*/
/**************************************************************************/
uint8_t addressRegR(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	return readReg8(drvp, AS5048B_ADDR_REG);
}

/**************************************************************************/
/*!
	@brief  sets current angle as the zero position

	@params[in]
				none
	@returns
				none
*/
/**************************************************************************/
void setZeroReg(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	zeroRegW(ip, (uint16_t)0x00);
	uint16_t newZero = readReg16(drvp, AS5048B_ANGLMSB_REG);
	zeroRegW(ip, newZero);
	return;
}

/**************************************************************************/
/*!
	@brief  writes the 2 bytes Zero position register value

	@params[in]
				unit16_t register value
	@returns
				none
*/
/**************************************************************************/
void zeroRegW(void *ip, uint16_t regVal)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	writeReg(drvp, AS5048B_ZEROMSB_REG, (uint8_t)(regVal >> 6));
	writeReg(drvp, AS5048B_ZEROLSB_REG, (uint8_t)(regVal & 0x3F));
	return;
}

/**************************************************************************/
/*!
	@brief  reads the 2 bytes Zero position register value

	@params[in]
				none
	@returns
				uint16_t register value trimmed on 14 bits
*/
/**************************************************************************/
uint16_t zeroRegR(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	return readReg16(drvp, AS5048B_ZEROMSB_REG);
}

/**************************************************************************/
/*!
	@brief  reads the 2 bytes magnitude register value

	@params[in]
				none
	@returns
				uint16_t register value trimmed on 14 bits
*/
/**************************************************************************/
uint16_t magnitudeR(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	return readReg16(drvp, AS5048B_MAGNMSB_REG);
}

uint16_t angleRegR(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	return readReg16(drvp, AS5048B_ANGLMSB_REG);
}

/**************************************************************************/
/*!
	@brief  reads the 1 bytes auto gain register value

	@params[in]
				none
	@returns
				uint8_t register value
*/
/**************************************************************************/
uint8_t getAutoGain(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	return readReg8(drvp, AS5048B_GAIN_REG);
}

/**************************************************************************/
/*!
	@brief  reads the 1 bytes diagnostic register value

	@params[in]
				none
	@returns
				uint8_t register value
*/
/**************************************************************************/
uint8_t getDiagReg(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	return readReg8(drvp, AS5048B_DIAG_REG);
}

/**************************************************************************/
/*!
	@brief  reads current angle value and converts it into the desired unit

	@params[in]
				String unit : string expressing the unit of the angle. Sensor raw value as default
	@params[in]
				bool newVal : have a new measurement or use the last read one. True as default
	@returns
				Double angle value converted into the desired unit
*/
/**************************************************************************/
double angleR(void *ip, int unit, bool newVal)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	double angleRaw;

	if (newVal)
	{
		if (_clockWise)
		{
			angleRaw = (double)(0b11111111111111 - readReg16(drvp, AS5048B_ANGLMSB_REG));
		}
		else
		{
			angleRaw = (double)readReg16(drvp, AS5048B_ANGLMSB_REG);
		}
		_lastAngleRaw = angleRaw;
	}
	else
	{
		angleRaw = _lastAngleRaw;
	}

	return convertAngle(unit, angleRaw);
}

/**************************************************************************/
/*!
	@brief  Performs an exponential moving average on the angle.
			Works on Sine and Cosine of the angle to avoid issues 0°/360° discontinuity

	@params[in]
				none
	@returns
				none
*/
/**************************************************************************/
void updateMovingAvgExp(void *ip)
{
	AS5048BDriver *drvp = (AS5048BDriver *)ip;
	// sine and cosine calculation on angles in radian
	double angle = angleR(drvp, U_RAD, true);

	if (_movingAvgCountLoop < EXP_MOVAVG_LOOP)
	{
		_movingAvgExpSin += sin(angle);
		_movingAvgExpCos += cos(angle);
		if (_movingAvgCountLoop == (EXP_MOVAVG_LOOP - 1))
		{
			_movingAvgExpSin = _movingAvgExpSin / EXP_MOVAVG_LOOP;
			_movingAvgExpCos = _movingAvgExpCos / EXP_MOVAVG_LOOP;
		}
		_movingAvgCountLoop++;
	}
	else
	{
		double movavgexpsin = _movingAvgExpSin + _movingAvgExpAlpha * (sin(angle) - _movingAvgExpSin);
		double movavgexpcos = _movingAvgExpCos + _movingAvgExpAlpha * (cos(angle) - _movingAvgExpCos);
		_movingAvgExpSin = movavgexpsin;
		_movingAvgExpCos = movavgexpcos;
		_movingAvgExpAngle = getExpAvgRawAngle();
	}

	return;
}

/**************************************************************************/
/*!
	@brief  sent back the exponential moving averaged angle in the desired unit

	@params[in]
				String unit : string expressing the unit of the angle. Sensor raw value as default
	@returns
				Double exponential moving averaged angle value
*/
/**************************************************************************/
double getMovingAvgExp(int unit)
{
	return convertAngle(unit, _movingAvgExpAngle);
}

void resetMovingAvgExp(void)
{
	_movingAvgExpAngle = 0.0;
	_movingAvgCountLoop = 0;
	_movingAvgExpAlpha = 2.0 / (EXP_MOVAVG_N + 1.0);
	return;
}

// Private functions
double convertAngle(int unit, double angle)
{
	// convert raw sensor reading into angle unit
	double angleConv;

	switch (unit)
	{
	case U_RAW:
		// Sensor raw measurement
		angleConv = angle;
		break;
	case U_TRN:
		// full turn ratio
		angleConv = (angle / AS5048B_RESOLUTION);
		break;
	case U_DEG:
		// degree
		angleConv = (angle / AS5048B_RESOLUTION) * 360.0;
		break;
	case U_RAD:
		// Radian
		angleConv = (angle / AS5048B_RESOLUTION) * 2 * M_PI;
		break;
	case U_MOA:
		// minute of arc
		angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 360.0;
		break;
	case U_SOA:
		// second of arc
		angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 60.0 * 360.0;
		break;
	case U_GRAD:
		// grade
		angleConv = (angle / AS5048B_RESOLUTION) * 400.0;
		break;
	case U_MILNATO:
		// NATO MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6400.0;
		break;
	case U_MILSE:
		// Swedish MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6300.0;
		break;
	case U_MILRU:
		// Russian MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6000.0;
		break;
	default:
		// no conversion => raw angle
		angleConv = angle;
		break;
	}
	return angleConv;
}

double getExpAvgRawAngle(void)
{
	double angle;
	double twopi = 2 * M_PI;

	if (_movingAvgExpSin < 0.0)
	{
		angle = twopi - acos(_movingAvgExpCos);
	}
	else
	{
		angle = acos(_movingAvgExpCos);
	}

	angle = (angle / twopi) * AS5048B_RESOLUTION;

	return angle;
}

static uint8_t readReg8(AS5048BDriver *drvp, uint8_t address)
{
	uint8_t data[2];
	i2cAcquireBus(drvp->config->i2cp);
	i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);
	i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr, &address, 1, data, 1, TIME_INFINITE);
	i2cReleaseBus(drvp->config->i2cp);
	return data[0];
}

static uint16_t readReg16(AS5048BDriver *drvp, uint8_t address)
{
	// 16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
	uint8_t data[2];
	uint16_t readValue;
	i2cAcquireBus(drvp->config->i2cp);
	i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);
	i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr, &address, 1, data, 2, TIME_INFINITE);
	i2cReleaseBus(drvp->config->i2cp);

	readValue = (((uint16_t)data[0]) << 6);
	readValue += (data[1] & 0x3F);

	return readValue;
}

static void writeReg(AS5048BDriver *drvp, uint8_t address, uint8_t value)
{
	uint8_t data[2] = {address, value};
	i2cAcquireBus(drvp->config->i2cp);
	i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);
	i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr, data, 2, NULL, 0, TIME_INFINITE);
	i2cReleaseBus(drvp->config->i2cp);
	return;
}

static const struct AS5048BVMT vmt_as5048B = {
	setClockWise, progRegister, doProg, doProgZero, addressRegW, addressRegR,
	setZeroReg, zeroRegW, zeroRegR, angleRegR, magnitudeR, angleR, getAutoGain,
	getDiagReg, updateMovingAvgExp, getMovingAvgExp, resetMovingAvgExp};

void as5048BObjectInit(AS5048BDriver *devp)
{
	devp->vmt = &vmt_as5048B;
	devp->config = NULL;
	devp->state = AS5048B_STOP;
}

void as5048BStart(AS5048BDriver *devp, AS5048BConfig *config)
{
	chDbgCheck((devp != NULL) && (config != NULL));

	chDbgAssert((devp->state == AS5048B_STOP) || (devp->state == AS5048B_READY),
				"as5048BStart(), invalid state");

	devp->config = config;

	_clockWise = false;
	_lastAngleRaw = 0.0;
	_zeroRegVal = zeroRegR(devp);
	_addressRegVal = addressRegR(devp);

	resetMovingAvgExp();

	devp->state = AS5048B_READY;
}

void as5048BStop(AS5048BDriver *devp)
{
	chDbgAssert((devp->state == AS5048B_STOP) || (devp->state == AS5048B_READY),
				"as5048BStop(), invalid state");

	if (devp->state == AS5048B_READY)
	{
		// powerDown(&devp);
	}

	devp->state = AS5048B_STOP;
}
