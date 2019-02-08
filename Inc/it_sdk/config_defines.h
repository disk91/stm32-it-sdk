/* ==========================================================
 * config_defines.h - All the possible configuration settings
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
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
 * @TODO - Actually only STM32L0x1 is implemented
 *
 * ==========================================================
 */

#ifndef IT_SDK_CONFIG_DEFINES_H_
#define IT_SDK_CONFIG_DEFINES_H_

/**
 * Cube MX code generator
 */
#define __CUBEMX_VERSIONXXX			499
#define __CUBEMX_VERSION500			500

/**
 * Supported Hardware Platform
 */
#define __PLATFORM_EFM32_TD			0
#define __PLATFORM_STM32L0			1
#define __PLAFTORM_ESP8266			2

/**
 * Devices
 */
#define __DEVICE_STM32L011D4		1
#define __DEVICE_STM32L072XX		2
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
 * CLK CORRECTION SOURCE
 */
#define __CLK_BEST_SRC_UNDEFINED	0x00			// The most accurate source is unknonw
#define __CLK_BEST_SRC_RTC			0x01			// The RTC clock is the most accurate source
#define __CLK_BEST_SRC_CLK			0x02			// The main CLK is the most accurated source

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
 * TIMER configuration
 */
#define __TIMER_NONE				0x00			// No hw timer code
#define __TIMER_ENABLED				0x01			// with hw timer code

/**
 * Basic enable / disable
 */
#define __DISABLE					0x00
#define __ENABLE					0x01

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
#define __BANK_H				7


/**
 * LOG MODULE DEFINES
 */
#define __LOG_MOD_NONE			0x00000000			// No Log
#define __LOG_MOD_STIMER		0x00000001			// software timer module
#define __LOG_MOD_LOWSIGFOX		0x00000002			// sigfox low level driver whatever it is
#define __LOG_MOD_SIGFOX		0x00000004			// sigfox abstraction layer
#define __LOG_MOD_LOWLORADBG	0x00000008			// Lora low level - hardware level
#define __LOG_MOD_LOWLORAINF	0x00000010			// Lora low level - mac level
#define __LOG_MOD_STKLORA		0x00000020			// Lora low level - itsdk level

#define __LOG_MOD_CUSTOM1		0x00010000			// User level logging
#define __LOG_MOD_CUSTOM2		0x00020000			// User level logging
#define __LOG_MOD_CUSTOM3		0x00040000			// User level logging
#define __LOG_MOD_CUSTOM4		0x00080000			// User level logging
#define __LOG_MOD_CUSTOM5		0x00100000			// User level logging
#define __LOG_MOD_CUSTOM6		0x00200000			// User level logging
#define __LOG_MOD_CUSTOM7		0x00400000			// User level logging
#define __LOG_MOD_CUSTOM8		0x00800000			// User level logging
#define __LOG_MOD_CUSTOM9		0x01000000			// User level logging
#define __LOG_MOD_CUSTOMA		0x02000000			// User level logging
#define __LOG_MOD_CUSTOMB		0x04000000			// User level logging
#define __LOG_MOD_CUSTOMC		0x08000000			// User level logging
#define __LOG_MOD_CUSTOMD		0x10000000			// User level logging
#define __LOG_MOD_CUSTOME		0x20000000			// User level logging
#define __LOG_MOD_CUSTOMF		0x40000000			// User level logging
#define __LOG_MOD_RESERVED		0x80000000			// Reserved Level

/**
 * Supported SIGFOX Interface
 */
#define __SIGFOX_NONE			0
#define	__SIGFOX_S2LP			1
#define __SIGFOX_WISOL10		2

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

/**
 * Encryption type - Encryption are cumulative.
 */
#define __PAYLOAD_ENCRYPT_NONE	 0					// No Encryption
#define	__PAYLOAD_ENCRYPT_SIGFOX 1					// Sigfox payload encryption
#define	__PAYLOAD_ENCRYPT_AESCTR 2					// Custom AES-CTR encryption
#define	__PAYLOAD_ENCRYPT_SPECK  4					// Speck encryption


/**
 * Supported LoRaWAN Interface
 */
#define __LORAWAN_NONE			0					// LoRaWan disabled
#define __LORAWAN_SX1276		1					// SX1276 like for Murata or RFM95w

/**
 * Supported LoRaWAN Implementation
 */
#define __LORAWAN_NONE			0					// No selection, LORAWAN should be disabled
#define __LORAWAN_SEMTECH		1					// Activate the LoRaWan stack


/**
 * LoRaWAN Type of activation
 */
#define __LORAWAN_OTAA			0x01				// Over The Air Activation
#define __LORAWAN_ABP			0x02				// Activation By Personalization

/**
 * LoRaWAN ADR (Adaptative Data Rate)
 */
#define __LORAWAN_ADR_UNDEFINED	0x00
#define __LORAWAN_ADR_OFF		0x01
#define __LORAWAN_ADR_ON		0x02

/**
 * LoRaWAN default Data Rate
 */
#define __LORAWAN_DR_UNDEFINED	0x0
#define __LORAWAN_DR_0			0x1
#define __LORAWAN_DR_1			0x2
#define __LORAWAN_DR_2			0x3
#define __LORAWAN_DR_3			0x4
#define __LORAWAN_DR_4			0x5
#define __LORAWAN_DR_5			0x6
#define __LORAWAN_DR_6			0x7
#define __LORAWAN_DR_7			0x8
#define __LORAWAN_DR_8			0x9
#define __LORAWAN_DR_9			0x10
#define __LORAWAN_DR_10			0x11
#define __LORAWAN_DR_11			0x12
#define __LORAWAN_DR_12			0x13
#define __LORAWAN_DR_13			0x14
#define __LORAWAN_DR_14			0x15
#define __LORAWAN_DR_15			0x16

/**
 * LoRaWAN Misc defines
 */
#define __LORAWAN_NWK_PUBLIC		0x01			// Public network
#define __LORAWAN_NWK_PRIVATE		0x02			// Private network

#define __LORAWAN_DEVEUI_STATIC		0x01			// Device EUI is static
#define __LORAWAN_DEVEUI_GENERATED	0x02			// Device EUI stored is generated from the boardId

/**
 * LoRaWAN Region to be compiled
 */
#define __LORAWAN_REGION_NONE		0x0000
#define __LORAWAN_REGION_AS923		0x0001
#define __LORAWAN_REGION_AU915		0x0002
#define __LORAWAN_REGION_CN470		0x0004
#define __LORAWAN_REGION_CN779		0x0008
#define __LORAWAN_REGION_EU433		0x0010
#define __LORAWAN_REGION_EU868		0x0020
#define __LORAWAN_REGION_KR920		0x0040
#define __LORAWAN_REGION_IN865		0x0080
#define __LORAWAN_REGION_US915		0x0100
#define __LORAWAN_REGION_RU864		0x0200

#endif /* IT_SDK_CONFIG_DEFINES_H_ */
