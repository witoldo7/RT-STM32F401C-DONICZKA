// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2024 Witold Olechowski.
 */

#include "ch.h"
#include "hal.h"
#include "bme280.h"
#include "math.h"

// local functions
static float calcTf(int32_t UT);
static float calcPf(uint32_t UP);
static float calcHf(uint32_t UH);

static void writeReg8(BME280Driver *drvp, uint8_t reg, uint8_t value);
static uint8_t readReg8(BME280Driver *drvp, uint8_t reg);
static msg_t readReg(BME280Driver *drvp, uint8_t address, uint8_t *data, uint8_t size);
static BME280_RESULT check(BME280Driver *drvp);
static void reset(BME280Driver *drvp);
static void setMode(BME280Driver *drvp, uint8_t mode);
static void setFilter(BME280Driver *drvp, uint8_t filter);
static void setStandby(BME280Driver *drvp, uint8_t tsb);
static void setOSRST(BME280Driver *drvp, uint8_t osrs);
static void setOSRSP(BME280Driver *drvp, uint8_t osrs);
static void setOSRSH(BME280Driver *drvp, uint8_t osrs);
static BME280_RESULT readCalibration(BME280Driver *drvp);

// Compensation parameters storage
BME280_Compensation_TypeDef cal_param;
// Carries fine temperature as global value for pressure and humidity calculation
static int32_t t_fine;

static void writeReg8(BME280Driver *drvp, uint8_t address, uint8_t value)
{
  uint8_t data[2] = {address, value};
  i2cAcquireBus(drvp->config->i2cp);
  i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);
  i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr, data, 2, NULL, 0, TIME_MS2I(200));
  i2cReleaseBus(drvp->config->i2cp);
}

static uint8_t readReg8(BME280Driver *drvp, uint8_t address)
{
  uint8_t data[2];
  readReg(drvp, address, data, 2);
  return data[0];
}

static msg_t readReg(BME280Driver *drvp, uint8_t address, uint8_t *data, uint8_t size)
{
  i2cAcquireBus(drvp->config->i2cp);
  i2cStart(drvp->config->i2cp, drvp->config->i2ccfg);
  msg_t status = i2cMasterTransmitTimeout(drvp->config->i2cp, drvp->config->addr, &address, 1, data, size, TIME_MS2I(200));
  i2cReleaseBus(drvp->config->i2cp);
  return status;
}

// Check if BME280 present on I2C bus
// return:
//   BME280_SUCCESS if BME280 present, BME280_ERROR otherwise (not present or it was an I2C timeout)
static BME280_RESULT check(BME280Driver *drvp)
{
  return (readReg8(drvp, BME280_REG_ID) == 0x60) ? BME280_SUCCESS : BME280_ERROR;
}

// Order BME280 to do a software reset
// note: after reset the chip will be unaccessible during 3ms
static void reset(BME280Driver *drvp)
{
  writeReg8(drvp, BME280_REG_RESET, BME280_SOFT_RESET_KEY);
}

// Get version of the BME280 chip
// return:
//   BME280 chip version or zero if no BME280 present on the I2C bus or it was an I2C timeout
uint8_t getVersion(BME280Driver *drvp)
{
  return readReg8(drvp, BME280_REG_ID);
}

// Get current status of the BME280 chip
// return:
//   Status of the BME280 chip or zero if no BME280 present on the I2C bus or it was an I2C timeout
uint8_t getStatus(BME280Driver *drvp)
{
  return readReg8(drvp, BME280_REG_STATUS) & BME280_STATUS_MSK;
}

// Get current sensor mode of the BME280 chip
// return:
//   Sensor mode of the BME280 chip or zero if no BME280 present on the I2C bus or it was an I2C timeout
uint8_t getMode(BME280Driver *drvp)
{
  return readReg8(drvp, BME280_REG_CTRL_MEAS) & BME280_MODE_MSK;
}

// Set sensor mode of the BME280 chip
// input:
//   mode - new mode (BME280_MODE_SLEEP, BME280_MODE_FORCED or BME280_MODE_NORMAL)
static void setMode(BME280Driver *drvp, uint8_t mode)
{
  uint8_t reg;
  // Read the 'ctrl_meas' (0xF4) register and clear 'mode' bits
  reg = readReg8(drvp, BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
  // Configure new mode
  reg |= mode & BME280_MODE_MSK;
  // Write value back to the register
  writeReg8(drvp, BME280_REG_CTRL_MEAS, reg);
}

// Set coefficient of the IIR filter
// input:
//   filter - new coefficient value (one of BME280_FILTER_x values)
void setFilter(BME280Driver *drvp, uint8_t filter)
{
  uint8_t reg;
  // Read the 'config' (0xF5) register and clear 'filter' bits
  reg = readReg8(drvp, BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
  // Configure new filter value
  reg |= filter & BME280_FILTER_MSK;
  // Write value back to the register
  writeReg8(drvp, BME280_REG_CONFIG, reg);
}

// Set inactive duration in normal mode (Tstandby)
// input:
//   tsb - new inactive duration (one of BME280_STBY_x values)
static void setStandby(BME280Driver *drvp, uint8_t tsb)
{
  uint8_t reg;
  // Read the 'config' (0xF5) register and clear 'filter' bits
  reg = readReg8(drvp, BME280_REG_CONFIG) & ~BME280_STBY_MSK;
  // Configure new standby value
  reg |= tsb & BME280_STBY_MSK;
  // Write value back to the register
  writeReg8(drvp, BME280_REG_CONFIG, reg);
}

// Set oversampling of temperature data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_T_Xx values)
static void setOSRST(BME280Driver *drvp, uint8_t osrs)
{
  uint8_t reg;
  // Read the 'ctrl_meas' (0xF4) register and clear 'osrs_t' bits
  reg = readReg8(drvp, BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
  // Configure new oversampling value
  reg |= osrs & BME280_OSRS_T_MSK;
  // Write value back to the register
  writeReg8(drvp, BME280_REG_CTRL_MEAS, reg);
}

// Set oversampling of pressure data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_P_Xx values)
static void setOSRSP(BME280Driver *drvp, uint8_t osrs)
{
  uint8_t reg;
  // Read the 'ctrl_meas' (0xF4) register and clear 'osrs_p' bits
  reg = readReg8(drvp, BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
  // Configure new oversampling valueBME280Driver *drvp
  reg |= osrs & BME280_OSRS_P_MSK;
  // Write value back to the register
  writeReg8(drvp, BME280_REG_CTRL_MEAS, reg);
}

// Set oversampling of humidity data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_H_Xx values)
static void setOSRSH(BME280Driver *drvp, uint8_t osrs)
{
  uint8_t reg;
  // Read the 'ctrl_hum' (0xF2) register and clear 'osrs_h' bits
  reg = readReg8(drvp, BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
  // Configure new oversampling value
  reg |= osrs & BME280_OSRS_H_MSK;
  // Write value back to the register
  writeReg8(drvp, BME280_REG_CTRL_HUM, reg);
  // Changes to 'ctrl_hum' register only become effective after a write to 'ctrl_meas' register
  // Thus read a value of the 'ctrl_meas' register and wrBME280_WriteRegite it back after write to the 'ctrl_hum'
  // Read the 'ctrl_meas' (0xF4) register
  reg = readReg8(drvp, BME280_REG_CTRL_MEAS);
  // Write back value of 'ctrl_meas' register to activate changes in 'ctrl_hum' register
  writeReg8(drvp, BME280_REG_CTRL_MEAS, reg);
}

// Read calibration data
static BME280_RESULT readCalibration(BME280Driver *drvp)
{
  uint8_t buf[7] = {0};
  // Read pressure and temperature calibration data (calib00..calib23)
  buf[0] = BME280_REG_CALIB00; // calib00 register address
  if (readReg(drvp, buf[0], (uint8_t *)&cal_param, 24) != MSG_OK)
  {
    return BME280_ERROR;
  }
  // Skip one byte (calib24) and read H1 (calib25)
  cal_param.dig_H1 = readReg8(drvp, BME280_REG_CALIB25);

  // Read humidity calibration data (calib26..calib41)
  buf[0] = BME280_REG_CALIB26; // calib26 register address

  if (readReg(drvp, buf[0], buf, 7) != MSG_OK)
    return BME280_ERROR;

  // Unpack data
  cal_param.dig_H2 = (int16_t)((((int8_t)buf[1]) << 8) | buf[0]);
  cal_param.dig_H3 = buf[2];
  cal_param.dig_H4 = (int16_t)((((int8_t)buf[3]) << 4) | (buf[4] & 0x0f));
  cal_param.dig_H5 = (int16_t)((((int8_t)buf[5]) << 4) | (buf[4] >> 4));
  cal_param.dig_H6 = (int8_t)buf[6];

  return BME280_SUCCESS;
}

// Read all raw values
// input:
//   UT = pointer to store temperature value
//   UP = pointer to store pressure value
//   UH = pointer to store humidity value
// return:
//   BME280_ERROR in case of I2C timeout, BME280_SUCCESS otherwise
// note: 0x80000 value for UT and UP and 0x8000 for UH means no data
BME280_RESULT readUTPH(void *ip)
{
  BME280Driver *drvp = (BME280Driver *)ip;
  uint8_t buf[8];
  // Clear result values
  drvp->data.humidity = -999;
  drvp->data.presure = -999;
  drvp->data.temperature = -999;

  // Send 'press_msb' register address
  buf[0] = BME280_REG_PRESS_MSB;
  // Read the 'press', 'temp' and 'hum' registers
  if (readReg(drvp, buf[0], buf, 8) == MSG_OK)
  {
    drvp->data.presure = calcPf((int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4)));
    drvp->data.temperature = calcTf((int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4)));
    drvp->data.humidity = calcHf((int32_t)((buf[6] << 8) | buf[7]));
    return BME280_SUCCESS;
  }
  return BME280_ERROR;
}

// Calculate temperature from raw value using floats, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '51.23' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)
static float calcTf(int32_t UT)
{
  float v_x1, v_x2;

  v_x1 = (((float)UT) / 16384.0 - ((float)cal_param.dig_T1) / 1024.0) * ((float)cal_param.dig_T2);
  v_x2 = ((float)UT) / 131072.0 - ((float)cal_param.dig_T1) / 8192.0;
  v_x2 = (v_x2 * v_x2) * ((float)cal_param.dig_T3);
  t_fine = (uint32_t)(v_x1 + v_x2);

  return ((v_x1 + v_x2) / 5120.0);
}

// Calculate pressure from raw value using floats, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa (value of '99158.968' represents 99158.968Pa)
// note: BME280_CalcT of BME280_CalcTf must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
static float calcPf(uint32_t UP)
{
  float v_x1 = 0, v_x2 = 0, p = 0;

  v_x1 = ((float)t_fine / 2.0) - 64000.0;
  v_x2 = v_x1 * v_x1 * ((float)cal_param.dig_P6) / 32768.0;
  v_x2 = v_x2 + v_x1 * ((float)cal_param.dig_P5) * 2.0;
  v_x2 = (v_x2 / 4.0) + (((float)cal_param.dig_P4) * 65536.0);
  v_x1 = (((float)cal_param.dig_P3) * v_x1 * v_x1 / 524288.0 + ((float)cal_param.dig_P2) * v_x1) / 524288.0;
  v_x1 = (1.0 + v_x1 / 32768.0) * ((float)cal_param.dig_P1);
  p = 1048576.0 - (float)UP;
  if (v_x1 == 0)
    return 0; // Avoid exception caused by division by zero
  p = (p - (v_x2 / 4096.0)) * 6250.0 / v_x1;
  v_x1 = ((float)cal_param.dig_P9) * p * p / 2147483648.0;
  v_x2 = p * ((float)cal_param.dig_P8) / 32768.0;
  p += (v_x1 + v_x2 + ((float)cal_param.dig_P7)) / 16.0;

  return p;
}

// Calculate humidity from raw value using floats, resolution is 0.001 %RH
// input:
//   UH - raw humidity value
// return: humidity in %RH (value of '46.333' represents 46.333%RH)
// note: BME280_CalcT or BME280_CalcTf must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
static float calcHf(uint32_t UH)
{
  float h;

  h = (((float)t_fine) - 76800.0);
  if (h == 0)
    return 0;
  h = (UH - (((float)cal_param.dig_H4) * 64.0 + ((float)cal_param.dig_H5) / 16384.0 * h));
  h = h * (((float)cal_param.dig_H2) / 65536.0 * (1.0 + ((float)cal_param.dig_H6) / 67108864.0 * h * (1.0 + ((float)cal_param.dig_H3) / 67108864.0 * h)));
  h = h * (1.0 - ((float)cal_param.dig_H1) * h / 524288.0);
  if (h > 100.0)
  {
    h = 100.0;
  }
  else if (h < 0.0)
  {
    h = 0.0;
  }

  return h;
}

float readAltitude(void *ip, float seaLevelhPa)
{
  float altitude;
  readUTPH(ip);
  BME280Driver *drvp = (BME280Driver *)ip;

  float pressure = drvp->data.presure;
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}

static const struct BME280VMT vmt_bme280 = {
    readUTPH, readAltitude};

void bme280ObjectInit(BME280Driver *drvp)
{
  drvp->vmt = &vmt_bme280;
  drvp->config = NULL;

  drvp->state = BME280_STOP;
}

void bme280Start(BME280Driver *drvp, const BME280Config *config)
{
  chDbgCheck((drvp != NULL) && (config != NULL));

  chDbgAssert((drvp->state == BME280_STOP) || (drvp->state == BME280_READY),
              "bme280Start(), invalid state");

  drvp->config = config;

  if (check(drvp) == BME280_ERROR)
  {
    drvp->state = BME280_STOP;
    return;
  }

  chThdSleepMilliseconds(50);
  reset(drvp);
  chThdSleepMilliseconds(50);
  // Set normal mode inactive duration (standby time)
  readCalibration(drvp);
  setStandby(drvp, BME280_STBY_1s);
  // Set IIR filter constant
  setFilter(drvp, BME280_FILTER_4);
  // Set oversampling for temperature
  setOSRST(drvp, BME280_OSRS_T_x4);
  // Set oversampling for pressure
  setOSRSP(drvp, BME280_OSRS_P_x2);
  // Set oversampling for humidity
  setOSRSH(drvp, BME280_OSRS_H_x1);
  // Set normal mode (perpetual periodic conversion)
  setMode(drvp, BME280_MODE_NORMAL);

  drvp->state = BME280_READY;
}

void bme280Stop(BME280Driver *drvp)
{
  chDbgAssert((drvp->state == BME280_STOP) || (drvp->state == BME280_READY),
              "bme280Stop(), invalid state");

  if (drvp->state == BME280_READY)
  {
    // TODO
  }

  drvp->state = BME280_STOP;
}
