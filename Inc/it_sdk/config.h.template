/* ==========================================================
 * config.h - SDK Configuration file
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  Disk91
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

#ifndef IT_SDK_CONFIG_H_
#define IT_SDK_CONFIG_H_

#include <it_sdk/config_defines.h>

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// | SDK SETTING                   | USER SELECTED VALUE                  | SETTING DESCRIPTION                   |
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define ITSDK_PLATFORM 				__PLATFORM_STM32L0x3					// Hardware platform selection
#define ITSDK_DEVICE				__DEVICE_STM32L053R8					// Specific device
#define ITSDK_RAM_SIZE				2048									// RAM Memory size
#define ITSDK_WITH_UART				( __UART_USART2 | __UART_LPUART1 )		// Use LPUART1 and USART2 for debug
#define ITSDK_WITH_RTC				__RTC_ENABLED							// The Rtc is usd in the firmware
#define ITSDK_WITH_CLK_ADJUST		1										// The RTC (and wtachdog) is calibrated
#define ITSDK_CLK_CORRECTION		1200									// correct clock with 1200 o/oo (+20%) of the ticks (used when clk_adjust = 0)
#define ITSDK_WITH_ADC				__ADC_ENABLED							// Use of Adc (includes the structures)
#define ITSDK_ADC1_PIN				14										// Map the channel for ADC on PIN 14 (PA0)
#define ITSDK_VDD_MV				3300									// VDD value in cV
#define ITSDK_WITH_SPI				__SPI_ENABLED							// Use SPI (inludes the strutures)
#define ITSDK_WITH_HW_TIMER			__TIMER_ENABLED							// Use Hardware Timer
#define ITSDK_HW_TIMER1_HANDLE		htim21									// Timer handler to be used as primary timer
#define ITSDK_HW_TIMER1_ID			21										// Timer hadware 1 - id/name
#define ITSDK_HW_TIMER1_FREQ		16000000								// Primary timer base frequency
#define ITSDK_HW_TIMER1_MAX			65536									// Timer's counter max value ( 2^size )
#define ITSDK_TIMER_SLOTS			2										// Maximum number of SOFT TIMER available in parallel - 0 disable SOFT TIMER code
#define ITSDK_WDG_MS				16000									// WatchDog time out in ms 1 --> 28000 / 0 to disable
#define ITSDK_WDG_CLKFREQ			37000									//  Watchdog clock source frequency

#define ITSDK_LOGGER_CONF			0x0070									// error->info level on serial1 => USART2 (see logger.c)
#define ITSDK_LOGGER_MODULE			( \
									  __LOG_MOD_STIMER \
									| __LOG_MOD_LOWSIGFOX \
									| __LOG_MOD_SIGFOX \
									)										// list the module to be activated in log see config_defines.h

#define ITSDK_LOWPOWER_MOD			( __LOWPWR_MODE_STOP \
									| __LOWPWR_MODE_WAKE_RTC \
									| __LOWPWR_MODE_WAKE_GPIO \
									)										// Mode Stop + wakeup RTC & LPUART + GPIO
#define ITSDK_LOWPOWER_RTC_MS		500										// RTC wake up after 500ms
#define ITSDK_LOWPOWER_GPIO_A_KEEP	(  __LP_GPIO_5  \
									 | __LP_GPIO_2  \
									 | __LP_GPIO_3  \
									 | __LP_GPIO_1	\
									 | __LP_GPIO_6	\
									 | __LP_GPIO_7	\
									 | __LP_GPIO_8	\
		                            )										// Keep 5 activ (led) and 2/3 (Usart2) 1 (cs S2LP) 6/7 (spi) 8 (sl2p shutdown)
#define ITSDK_LOWPOWER_GPIO_B_KEEP	(  __LP_GPIO_3 \
									 | __LP_GPIO_4 \
									)										// Keep 4 (cs eeprom) / 5 ( spi )
#define ITSDK_LOWPOWER_GPIO_C_KEEP	(   __LP_GPIO_4 \
									  | __LP_GPIO_0 \
									  | __LP_GPIO_5 \
									)										// During Low Power mode, keep LPUART & PC0 = S2LP GPIO3
#define ITSDK_LOWPOWER_GPIO_D_KEEP	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank D are all off (not implemented yet)
#define ITSDK_LOWPOWER_GPIO_E_KEEP	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank E are all off (not implemented yet)
#define ITSDK_LOWPOWER_GPIO_H_KEEP	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank H are all off (not implemented yet)
																			// GPIO Wake-Up => the pin should also be in the _KEEP list
#define ITSDK_LOWPOWER_GPIO_A_WAKE	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank A can be used for wakeup
#define ITSDK_LOWPOWER_GPIO_B_WAKE	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank B can be used for wakeup
#define ITSDK_LOWPOWER_GPIO_C_WAKE	(__LP_GPIO_0)							// During Low Power mode, the GPIO bank C can be used for wakeup
#define ITSDK_LOWPOWER_GPIO_D_WAKE	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank D can be used for wakeup
#define ITSDK_LOWPOWER_GPIO_E_WAKE	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank E can be used for wakeup
#define ITSDK_LOWPOWER_GPIO_H_WAKE	(__LP_GPIO_NONE)						// During Low Power mode, the GPIO bank H can be used for wakeup

#define ITSDK_SHEDULER_TASKS		1										// Maximum number of Task (0 will deactivate scheduler code)
#define ITSDK_STATEMACHINE_TASKS	0										// Maximum number of state machine task (0 will deactivate STM code)
#define ITSDK_STATEMACHINE_NAMESZ	8										// Maximum size for task name (-1)

#define ITSDK_WITH_SIGFOX_LIB		1										// Include the sigfox code when 1 disabled when 0
#define ITSDK_SIGFOX_LIB			__SIGFOX_S2LP							// Type of Sigfox module


#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1  || ITSDK_PLATFORM == __PLATFORM_STM32L0x3
	#include <stm32l_sdk/config.h>
	#include "stm32l0xx_hal.h"
#endif

#if ITSDK_WITH_SIGFOX_LIB == 1
	#include <it_sdk/configSigfox.h>
#endif

#include <it_sdk/configDrivers.h>

#endif /* IT_SDK_CONFIG_H_ */
