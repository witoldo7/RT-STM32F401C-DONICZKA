/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "usbcfg.h"

/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(1000)
};


/*===========================================================================*/
/* PWM                                                             */
/*===========================================================================*/
static PWMConfig pwmcfg = {
    8000000,                                    /* 10kHz PWM clock frequency.   */
    1000,                                    /* Initial PWM period 1S.       */
    NULL,
    {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
	{PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0,
    0
};

static void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Usage: pwm 1000\r\n");
    return;
  }
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, atoi(argv[0])));
}

thread_t *shelltp = NULL;
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
static const ShellCommand commands[] = {
  {"pwm", cmd_pwm},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

/*
 * Shell exit event.
 */
static void ShellHandler(eventid_t id) {
  (void)id;
  if (chThdTerminatedX(shelltp)) {
    chThdRelease(shelltp);
    shelltp = NULL;
  }
}

/*
 * Application entry point.
 */
int main(void) {
  static const evhandler_t evhndl[] = {
    ShellHandler
  };

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
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (!shelltp && SDU1.config->usbp->state == USB_ACTIVE) {
      shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                       "shell1", NORMALPRIO + 1,
                                       shellThread, (void *)&shell_cfg1);
    }
    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
    }
    chEvtDispatch(evhndl, chEvtWaitOneTimeout(EVENT_MASK(0), TIME_MS2I(500)));
    wdgReset(&WDGD1);
  }
}
