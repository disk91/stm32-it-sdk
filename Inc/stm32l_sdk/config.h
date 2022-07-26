/* ==========================================================
 * config.h - STM32L specific configuration file
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 6 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU LESSER General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------
 * 
 *
 * ==========================================================
 */

#ifndef STM32L_SDK_CONFIG_H_
#define STM32L_SDK_CONFIG_H_

#include <it_sdk/config.h>

#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#include "stm32l0xx_hal.h"
#elif ITSDK_PLATFORM == __PLATFORM_STM32WLE
	#include "stm32wlxx_hal.h"
#endif


// Fix the poor quality of generated code...
void SystemClock_Config(void);

#include <gpio.h>

// LPUART1 stuff
#if (ITSDK_WITH_UART & __UART_LPUART1) > 0 || (ITSDK_WITH_UART & __UART_USART2) > 0
	#include <usart.h>
#endif

// RTC Stuff
#if  ( ITSDK_WITH_RTC & __RTC_ENABLED ) > 0
	#include <rtc.h>
#endif

// TIMER Stuff
#if ( ITSDK_WITH_HW_TIMER & __TIMER_ENABLED ) > 0
    #include <tim.h>
#endif

#if ITSDK_PLATFORM == __PLATFORM_STM32WLE
  // Missing defines in the new ST framework
  #if ITSDK_DEVICE	== __DEVICE_STM32WLE5JC
  #define GPIOA_PIN_AVAILABLE	((uint16_t)0xFFFFU)
  #define GPIOB_PIN_AVAILABLE	((uint16_t)0xFFFFU)
  #define GPIOC_PIN_AVAILABLE	((uint16_t)0xE07FU)
  #define GPIOH_PIN_AVAILABLE	((uint16_t)0x0004U)
  #else
   #error the device support needs to be added
  #endif

#endif


#endif /* STM32L_SDK_CONFIG_H_ */
