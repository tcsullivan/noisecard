/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

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

#include "hal.h"
#include "stm32_gpio.h"

static void stm32_gpio_init(void) {

  /* Enabling GPIO-related clocks, the mask comes from the
     registry header file.*/
  rccResetIOP(STM32_GPIO_EN_MASK);
  rccEnableIOP(STM32_GPIO_EN_MASK, true);
}

/**
 * @brief   Early initialization code.
 * @details GPIO ports and system clocks are initialized before everything
 *          else.
 */
void __early_init(void) {
  stm32_gpio_init();
  stm32_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @note    Add your board-specific code, if any.
 */
void boardInit(void) {
    palSetLineMode(LINE_LED0, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED0);
    palSetLineMode(LINE_LED1, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED1);
    palSetLineMode(LINE_LED2, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED2);
    palSetLineMode(LINE_LED3, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED3);
    palSetLineMode(LINE_LED4, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED4);
    palSetLineMode(LINE_LED5, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED5);
    palSetLineMode(LINE_LED6, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED6);
    palSetLineMode(LINE_LED7, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED7);
    palSetLineMode(LINE_LED8, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED8);
    palSetLineMode(LINE_LED9, PAL_MODE_OUTPUT_PUSHPULL); palSetLine(LINE_LED9);

    palSetLineMode(LINE_TP1, PAL_MODE_OUTPUT_PUSHPULL); palClearLine(LINE_TP1);

    palSetLineMode(LINE_I2S_SD, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_I2S_WS, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_I2S_CK, PAL_MODE_ALTERNATE(0));
}

