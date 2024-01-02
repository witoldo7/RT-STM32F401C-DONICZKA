// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2024 Witold Olechowski.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "utils.h"
#include "as5048b.h"
#include "as3935.h"

/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
static const WDGConfig wdgcfg = {
    STM32_IWDG_PR_64,
    STM32_IWDG_RL(1000)};

/*===========================================================================*/
/* I2C interface #1                                                          */
/*===========================================================================*/
/*  */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

/*===========================================================================*/
/* AS5048B Rotary sensor                                                     */
/*===========================================================================*/
static AS5048BConfig as5048cfg = {
    &I2CD1,
    &i2cfg1,
    AS5048_ADDRESS,
};

static AS5048BDriver AS5048B;

/*===========================================================================*/
/* AS3935 Lighting sensor                                                    */
/*===========================================================================*/
static const AS3935Config as3935cfg = {
    &I2CD1,
    &i2cfg1,
    AS3935_ADR_3,
};

static AS3935Driver AS3935;

/*===========================================================================*/
/* PWM                                                                       */
/*===========================================================================*/
static PWMConfig pwmcfg = {
    8000000, /* 10kHz PWM clock frequency.   */
    1000,    /* Initial PWM period 1S.       */
    NULL,
    {{PWM_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_DISABLED, NULL},
     {PWM_OUTPUT_DISABLED, NULL}},
    0,
    0,
    0};

static void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0)
  {
    chprintf(chp, "Usage: pwm 1000\r\n");
    return;
  }
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, atoi(argv[0])));
}

static void cmd_r(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  while (true)
  {
    AS5048B.vmt->updateMovingAvgExp(&AS5048B);
    double angle = AS5048B.vmt->getMovingAvgExp(U_DEG);
    chprintf(chp, "Angle: %3.1f , Wind Direction: %s\r\n", angle, degreeToCompass(angle));
    osalThreadSleepMilliseconds(50);
  }
}

static void cmd_l(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  chprintf(chp, "Distance to storm: %d km \r\n", AS3935.vmt->distanceToStorm(&AS3935));
}

thread_t *shelltp = NULL;
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)
static const ShellCommand commands[] = {
    {"pwm", cmd_pwm},
    {"r", cmd_r},
    {"l", cmd_l},
    {NULL, NULL}};

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&SDU1,
    commands};

/*
 * Shell exit event.
 */
static void ShellHandler(eventid_t id)
{
  (void)id;
  if (chThdTerminatedX(shelltp))
  {
    chThdRelease(shelltp);
    shelltp = NULL;
  }
}

/*
 * Application entry point.
 */
int main(void)
{
  static const evhandler_t evhndl[] = {
      ShellHandler};

  event_listener_t el0;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1500);
  usbStart(&USBD1, &usbcfg);

  usbConnectBus(&USBD1);
  chEvtRegister(&shell_terminated, &el0, 0);

  /*
   * Shell manager initialization.
   * Event zero is shell exit.
   */
  shellInit();

  /*
   * Starts the PWM channel 0 using 75% duty cycle.
   */
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));

  /*
   * Starting the watchdog driver.
   */
  wdgStart(&WDGD1, &wdgcfg);

  /*
   * Starts the Rotary sensor driver.
   */
  as5048BObjectInit(&AS5048B);
  as5048BStart(&AS5048B, &as5048cfg);
  AS5048B.vmt->setClockWise(true);
  AS5048B.vmt->zeroRegW(&AS5048B, 0);

  /*
   * Starts the Lighting sensor driver.
   */
  as3935ObjectInit(&AS3935);
  as3935Start(&AS3935, &as3935cfg);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true)
  {
    if (!shelltp && SDU1.config->usbp->state == USB_ACTIVE)
    {
      shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                    "shell", NORMALPRIO + 1,
                                    shellThread, (void *)&shell_cfg1);
    }
    chEvtDispatch(evhndl, chEvtWaitOneTimeout(EVENT_MASK(0), TIME_MS2I(500)));
    wdgReset(&WDGD1);
  }
}
