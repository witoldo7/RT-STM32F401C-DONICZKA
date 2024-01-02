// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2023 Witold Olechowski.
 */

#include "as3935.h"

static uint8_t readReg(AS3935Driver *drvp, uint8_t reg)
{
	uint8_t data[2];
	i2cAcquireBus(drvp->config->i2cp);
	i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);
	i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr, &reg, 1, data, 1, TIME_INFINITE);
	i2cReleaseBus(drvp->config->i2cp);
	return data[0];
}

static void writeReg(AS3935Driver *drvp, uint8_t reg, uint8_t mask, uint8_t bits, uint8_t startPosition)
{
	uint8_t txbuf[2];
	txbuf[0] = reg;
	txbuf[1] = readReg(drvp, reg);		 // Get the current value of the register
	txbuf[1] &= mask;					 // Mask the position we want to write to.
	txbuf[1] |= (bits << startPosition); // Write the given bits to the variable

	i2cAcquireBus(drvp->config->i2cp);
	i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);

	i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr,
							 txbuf, sizeof(txbuf), NULL, 0, TIME_INFINITE);

	i2cReleaseBus(drvp->config->i2cp);
}

// REG0x00, bit[0], manufacturer default: 0.
// The product consumes 1-2uA while powered down. If the board is powered down
// the the TRCO will need to be recalibrated: REG0x08[5] = 1, wait 2 ms, REG0x08[5] = 0.
// SPI and I-squared-C remain active when the chip is powered down.
static void powerDown(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	writeReg(drvp, AFE_GAIN, POWER_MASK, 1, 0);
}

// REG0x3A bit[7].
// This register holds the state of the timer RC oscillator (TRCO),
// after it has been calibrated. The TRCO will need to be recalibrated
// after power down. The following function wakes the IC, sends the "Direct Command" to
// CALIB_RCO register REG0x3D, waits 2ms and then checks that it has been successfully
// calibrated. Note that I-squared-C and SPI are active during power down.
static bool wakeUp(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	writeReg(drvp, AFE_GAIN, POWER_MASK, 0, 0); // Set the power down bit to zero to wake it up

	if (drvp->vmt->calibrateOsc(drvp))
		return true;
	else
		return false;
}

// REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
// This funciton changes toggles the chip's settings for Indoors and Outdoors.
static void setIndoorOutdoor(void *ip, uint8_t _setting)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if (_setting == INDOOR)
		writeReg(drvp, AFE_GAIN, GAIN_MASK, INDOOR, 1);
	if (_setting == OUTDOOR)
		writeReg(drvp, AFE_GAIN, GAIN_MASK, OUTDOOR, 1);
}

// REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
// This function returns the indoor/outdoor settting.
static uint8_t readIndoorOutdoor(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, AFE_GAIN);
	return ((regVal &= ~GAIN_MASK) >> 1);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This setting determines the threshold for events that trigger the
// IRQ Pin.
static void watchdogThreshold(void *ip, uint8_t _sensitivity)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if ((_sensitivity < 1) || (_sensitivity > 10)) // 10 is the max sensitivity setting
		return;
	writeReg(drvp, THRESHOLD, THRESH_MASK, _sensitivity, 0);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This function returns the threshold for events that trigger the
// IRQ Pin.
static uint8_t readWatchdogThreshold(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, THRESHOLD);
	return (regVal &= (~THRESH_MASK));
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// The noise floor level is compared to a known reference voltage. If this
// level is exceeded the chip will issue an interrupt to the IRQ pin,
// broadcasting that it can not operate properly due to noise (INT_NH).
// Check datasheet for specific noise level tolerances when setting this register.
static void setNoiseLevel(void *ip, uint8_t _floor)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if ((_floor < 1) || (_floor > 7))
		return;

	writeReg(drvp, THRESHOLD, NOISE_FLOOR_MASK, _floor, 4);
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// This function will return the set noise level threshold: default is 2.
static uint8_t readNoiseLevel(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	int8_t regVal = readReg(drvp, THRESHOLD);
	return (regVal & ~NOISE_FLOOR_MASK) >> 4;
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This setting, like the watchdog threshold, can help determine between false
// events and actual lightning. The shape of the spike is analyzed during the
// chip's signal validation routine. Increasing this value increases robustness
// at the cost of sensitivity to distant events.
static void spikeRejection(void *ip, uint8_t _spSensitivity)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if ((_spSensitivity < 1) || (_spSensitivity > 11))
		return;
	writeReg(drvp, LIGHTNING_REG, SPIKE_MASK, _spSensitivity, 0);
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This function returns the value of the spike rejection register. This value
// helps to differentiate between events and acutal lightning, by analyzing the
// shape of the spike during  chip's signal validation routine.
// Increasing this value increases robustness at the cost of sensitivity to distant events.
static uint8_t readSpikeRejection(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, LIGHTNING_REG);
	return (regVal &= ~SPIKE_MASK);
}

// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// The number of lightning events before IRQ is set high. 15 minutes is The
// window of time before the number of detected lightning events is reset.
// The number of lightning strikes can be set to 1,5,9, or 16.
static void lightningThreshold(void *ip, uint8_t _strikes)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t bits;

	if (_strikes == 1)
		bits = 0;
	else if (_strikes == 5)
		bits = 1;
	else if (_strikes == 9)
		bits = 2;
	else if (_strikes == 16)
		bits = 3;
	else
		return;

	writeReg(drvp, LIGHTNING_REG, LIGHT_MASK, bits, 4);
}

// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// This function will return the number of lightning strikes must strike within
// a 15 minute window before it triggers an event on the IRQ pin. Default is 1.
static uint8_t readLightningThreshold(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, LIGHTNING_REG);

	regVal &= ~LIGHT_MASK;
	regVal >>= 4; // Front of the line.

	if (regVal == 0)
		return 1;
	else if (regVal == 1)
		return 5;
	else if (regVal == 2)
		return 9;
	else if (regVal == 3)
		return 16;
	else
		return regVal;
}

// REG0x02, bit [6], manufacturer default: 1.
// This register clears the number of lightning strikes that has been read in
// the last 15 minute block.
static void clearStatistics(void *ip, bool _clearStat)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if (_clearStat != true)
		return;

	writeReg(drvp, LIGHTNING_REG, STAT_MASK, 1, 6);
	writeReg(drvp, LIGHTNING_REG, STAT_MASK, 0, 6);
	writeReg(drvp, LIGHTNING_REG, STAT_MASK, 1, 6);
}

// REG0x03, bits [3:0], manufacturer default: 0.
// When there is an event that exceeds the watchdog threshold, the register is written
// with the type of event. This consists of two messages: INT_D (disturber detected) and
// INT_L (Lightning detected). A third interrupt INT_NH (noise level too HIGH)
// indicates that the noise level has been exceeded and will persist until the
// noise has ended. Events are active HIGH. There is a one second window of time to
// read the interrupt register after lightning is detected, and 1.5 after
// disturber.
static uint8_t readInterruptReg(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	chThdSleepMilliseconds(2);

	uint8_t _interValue;
	_interValue = readReg(drvp, INT_MASK_ANT);
	_interValue &= INT_MASK;

	return (_interValue);
}

// REG0x03, bit [5], manufacturere default: 0.
// This setting will change whether or not disturbers trigger the IRQ Pin.
static void maskDisturber(void *ip, bool _state)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	writeReg(drvp, INT_MASK_ANT, DISTURB_MASK, _state, 5);
}

// REG0x03, bit [5], manufacturere default: 0.
// This setting will return whether or not disturbers trigger the IRQ Pin.
static uint8_t readMaskDisturber(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, INT_MASK_ANT);
	return (regVal &= ~DISTURB_MASK) >> 5;
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// The antenna is designed to resonate at 500kHz and so can be tuned with the
// following setting. The accuracy of the antenna must be within 3.5 percent of
// that value for proper signal validation and distance estimation.
static void changeDivRatio(void *ip, uint8_t _divisionRatio)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t bits;

	if (_divisionRatio == 16)
		bits = 0;
	else if (_divisionRatio == 32)
		bits = 1;
	else if (_divisionRatio == 64)
		bits = 2;
	else if (_divisionRatio == 128)
		bits = 3;
	else
		return;

	writeReg(drvp, INT_MASK_ANT, DIV_MASK, bits, 6);
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// This function returns the current division ratio of the resonance frequency.
// The antenna resonance frequency should be within 3.5 percent of 500kHz, and
// so when modifying the resonance frequency with the internal capacitors
// (tuneCap()) it's important to keep in mind that the displayed frequency on
// the IRQ pin is divided by this number.
static uint8_t readDivRatio(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, INT_MASK_ANT);
	regVal &= ~DIV_MASK;
	regVal >>= 6; // Front of the line.

	if (regVal == 0)
		return 16;
	else if (regVal == 1)
		return 32;
	else if (regVal == 2)
		return 64;
	else if (regVal == 3)
		return 128;
	else
		return UNKNOWN_ERROR;
}

// REG0x07, bit [5:0], manufacturer default: 0.
// This register holds the distance to the front of the storm and not the
// distance to a lightning strike.
static int8_t distanceToStorm(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t _dist = readReg(drvp, DISTANCE);
	_dist &= DISTANCE_MASK;
	return (_dist);
}

// REG0x08, bits [5,6,7], manufacturer default: 0.
// This will send the frequency of the oscillators to the IRQ pin.
//  _osc 1, bit[5] = TRCO - Timer RCO Oscillators 1.1MHz
//  _osc 2, bit[6] = SRCO - System RCO at 32.768kHz
//  _osc 3, bit[7] = LCO - Frequency of the Antenna
static void displayOscillator(void *ip, bool _state, uint8_t _osc)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if ((_osc < 1) || (_osc > 3))
		return;

	if (_state == true)
	{
		if (_osc == 1)
			writeReg(drvp, FREQ_DISP_IRQ, OSC_MASK, 1, 5);
		if (_osc == 2)
			writeReg(drvp, FREQ_DISP_IRQ, OSC_MASK, 1, 6);
		if (_osc == 3)
			writeReg(drvp, FREQ_DISP_IRQ, OSC_MASK, 1, 7);
	}

	if (_state == false)
	{
		if (_osc == 1)
			writeReg(drvp, FREQ_DISP_IRQ, OSC_MASK, 0, 5); // Demonstrative
		if (_osc == 2)
			writeReg(drvp, FREQ_DISP_IRQ, OSC_MASK, 0, 6);
		if (_osc == 3)
			writeReg(drvp, FREQ_DISP_IRQ, OSC_MASK, 0, 7);
	}
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will add capacitance to the series RLC antenna on the product.
// It's possible to add 0-120pF in steps of 8pF to the antenna.
static void tuneCap(void *ip, uint8_t _farad)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	if (_farad > 120)
		return;
	else if (_farad % 8 != 0)
		return;
	else
		_farad /= 8;

	writeReg(drvp, FREQ_DISP_IRQ, CAP_MASK, _farad, 0);
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will return the capacitance of the internal capacitors. It will
// return a value from one to 15 multiplied by the 8pF steps of the internal
// capacitance.
static uint8_t readTuneCap(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint8_t regVal = readReg(drvp, FREQ_DISP_IRQ);
	return ((regVal &= ~CAP_MASK) * 8); // Multiplied by 8pF
}

// LSB =  REG0x04, bits[7:0]
// MSB =  REG0x05, bits[7:0]
// MMSB = REG0x06, bits[4:0]
// This returns a 20 bit value that is the 'energy' of the lightning strike.
// According to the datasheet this is only a pure value that doesn't have any
// physical meaning.
static uint32_t lightningEnergy(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	uint32_t _pureLight = readReg(drvp, ENERGY_LIGHT_MMSB);
	_pureLight &= ENERGY_MASK;
	_pureLight <<= 8;
	_pureLight |= readReg(drvp, ENERGY_LIGHT_MSB);
	_pureLight <<= 8;
	_pureLight |= readReg(drvp, ENERGY_LIGHT_LSB);
	return _pureLight;
}

// REG0x3D, bits[7:0]
// This function calibrates both internal oscillators The oscillators are tuned
// based on the resonance frequency of the antenna and so it should be trimmed
// before the calibration is done.
static bool calibrateOsc(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	writeReg(drvp, CALIB_RCO, WIPE_ALL, DIRECT_COMMAND, 0); // Send command to calibrate the oscillators

	drvp->vmt->displayOscillator(drvp, true, 2);
	chThdSleepMilliseconds(2); // Give time for the internal oscillators to start up.
	drvp->vmt->displayOscillator(drvp, false, 2);

	// Check it they were calibrated successfully.
	uint8_t regValSrco = readReg(drvp, CALIB_SRCO);
	uint8_t regValTrco = readReg(drvp, CALIB_TRCO);

	regValSrco &= CALIB_MASK;
	regValSrco >>= 6;
	regValTrco &= CALIB_MASK;
	regValTrco >>= 6;

	if (!regValSrco && !regValTrco) // Zero upon success
		return true;
	else
		return false;
}

// REG0x3C, bits[7:0]
// This function resets all settings to their default values.
static void resetSettings(void *ip)
{
	AS3935Driver *drvp = (AS3935Driver *)ip;
	writeReg(drvp, RESET_LIGHT, WIPE_ALL, DIRECT_COMMAND, 0);
}

static const struct AS3935VMT vmt_as3935 = {
	powerDown, wakeUp, setIndoorOutdoor, readIndoorOutdoor, watchdogThreshold, readWatchdogThreshold,
	setNoiseLevel, readNoiseLevel, spikeRejection, readSpikeRejection, lightningThreshold,
	readLightningThreshold, clearStatistics, readInterruptReg, maskDisturber, readMaskDisturber,
	changeDivRatio, readDivRatio, distanceToStorm, displayOscillator, tuneCap, readTuneCap,
	lightningEnergy, calibrateOsc, resetSettings};

void as3935ObjectInit(AS3935Driver *devp)
{
	devp->vmt = &vmt_as3935;
	devp->config = NULL;

	devp->state = AS3935_STOP;
}

void as3935Start(AS3935Driver *devp, const AS3935Config *config)
{
	chDbgCheck((devp != NULL) && (config != NULL));

	chDbgAssert((devp->state == AS3935_STOP) || (devp->state == AS3935_READY),
				"as3935Start(), invalid state");

	devp->config = config;

	devp->state = AS3935_READY;
}

void as3935Stop(AS3935Driver *devp)
{
	chDbgAssert((devp->state == AS3935_STOP) || (devp->state == AS3935_READY),
				"as3935Stop(), invalid state");

	if (devp->state == AS3935_READY)
	{
		powerDown(&devp);
	}

	devp->state = AS3935_STOP;
}
