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
#include "stm32l0xx_hal.h"

// Fix the poor quality of generated code...
void SystemClock_Config(void);

#include <gpio.h>

// LPUART1 stuff
#if (ITSDK_WITH_UART & __UART_LPUART1) > 0
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




#endif /* STM32L_SDK_CONFIG_H_ */
