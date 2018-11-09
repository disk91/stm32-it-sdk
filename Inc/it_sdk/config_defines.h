/* ==========================================================
 * config_defines.h - All the possible configuration settings
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
 * @TODO - Actually only STM32L0x1 is implemented
 *
 * ==========================================================
 */

#ifndef IT_SDK_CONFIG_DEFINES_H_
#define IT_SDK_CONFIG_DEFINES_H_

/**
 * Supported Hardware Platform
 */
#define __PLATFORM_EFM32_TD			0
#define __PLATFORM_STM32L0x1		1
#define __PLATFORM_STM32L0x3		1
#define __PLAFTORM_ESP8266			2

/**
 * Devices
 */
#define __DEVICE_STM32L011D4		1
#define __DEVICE_STM32L053R8		3

/**
 * Supported Low Power Mode
 */
#define __LOWPWR_MODE_RUN			0x0000			// No power mode, always on
#define __LOWPWR_MORE_STANDBY		0x0400			// STANDBY MODE - MCU halt, no memory retained (not supported)
#define __LOWPWR_MODE_STOP			0x0100			// STOP MODE - MCU and most peripherals are stopped (Memory retained)
#define __LOWPWR_MODE_SLEEP			0x0200			// SLEEP MODE - MCU stoped, most peripheral running slow ( Memory retained)
#define __LOWPWR_MODE_WAKE_LPUART	0x0001
#define __LOWPWR_MODE_WAKE_GPIO		0x0002
#define __LOWPWR_MODE_WAKE_RTC		0x0004


/**
 * UART configuration
 */
#define __UART_NONE					0x0000			// No UART used
#define __UART_LPUART1				0x0001			// Use of LPUART1 peripheral
#define __UART_LPUART2				0x0002			// Use of LPUART2 peripheral
#define __UART_USART1				0x0004			// Use of UART1 peripheral
#define __UART_USART2				0x0008			// Use of UART2 peripheral

/**
 * RTC configuration
 */
#define __RTC_NONE					0x0000			// No RTC
#define __RTC_ENABLED				0x0001			// With RTC

/**
 * ADC configuration
 */
#define __ADC_NONE					0x00			// No ADC
#define __ADC_ENABLED				0x01			// with ADC

/**
 * SPI configuration
 */
#define __SPI_NONE					0x00			// No SPI
#define __SPI_ENABLED				0x01			// with SPI

/**
 * GPIO to keep activated on low power mode
 */
#define __LP_GPIO_NONE			0x0000
#define __LP_GPIO_0				GPIO_PIN_0
#define __LP_GPIO_1				GPIO_PIN_1
#define __LP_GPIO_2				GPIO_PIN_2
#define __LP_GPIO_3				GPIO_PIN_3
#define __LP_GPIO_4				GPIO_PIN_4
#define __LP_GPIO_5				GPIO_PIN_5
#define __LP_GPIO_6				GPIO_PIN_6
#define __LP_GPIO_7				GPIO_PIN_7
#define __LP_GPIO_8				GPIO_PIN_8
#define __LP_GPIO_9				GPIO_PIN_9
#define __LP_GPIO_10			GPIO_PIN_10
#define __LP_GPIO_11			GPIO_PIN_11
#define __LP_GPIO_12			GPIO_PIN_12
#define __LP_GPIO_13			GPIO_PIN_13
#define __LP_GPIO_14			GPIO_PIN_14
#define __LP_GPIO_15			GPIO_PIN_15


/**
 * GPIO BANKS
 */
#define __BANK_A				0
#define __BANK_B				1
#define __BANK_C				2
#define __BANK_D				3
#define __BANK_E				4
#define __BANK_F				5
#define __BANK_H				6


/**
 * Supported SIGFOX Interface
 */
#define	__SIGFOX_S2LP			0
#define __SIGFOX_WISOL10		1

/**
 * NVM source for Sigfox lib
 */
#define __SFX_NVM_LOCALEPROM	0					// MCU internal EEPROM
#define __SFX_NVM_M95640		1					// External EEPROM type M95640
#define __SFX_NVM_HEADERS		2					// Configuration stored in the #define

/**
 * Drivers S2LP Config
 */

#define __S2LP_WITH_TCXO		1
#define __S2LP_W_O_TCXO			0

#endif /* IT_SDK_CONFIG_DEFINES_H_ */
