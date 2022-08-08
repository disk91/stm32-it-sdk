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
#define __DEVICE_STM32L082XX    3
#define __DEVICE_STM32L053R8		4
#define __DEVICE_STM32L031K6		5
#define __DEVICE_STM32L052T8    6

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
#define __LOWPWR_MODE_WAKE_UART2	0x0008
#define __LOWPWR_MODE_WAKE_UART1	0x0010
#define __LOWPWR_MODE_WAKE_ALLUART  0x0019

/**
 * Module to keep activated during sleep
 */
#define __LP_HALT_NONE				0x0000
#define __LP_HALT_I2C1				0x0001
#define __LP_HALT_I2C2				0x0002
#define __LP_HALT_SPI1				0x0010
#define __LP_HALT_SPI2				0x0020
#define __LP_HALT_TIM21				0x0080
#define __LP_HALT_ADC1				0x0100


/**
 * UART configuration
 */
#define __UART_NONE					0x0000			// No UART used
#define __UART_LPUART1				0x0001			// Use of LPUART1 peripheral
#define __UART_LPUART2				0x0002			// Use of LPUART2 peripheral
#define __UART_USART1				0x0004			// Use of UART1 peripheral
#define __UART_USART2				0x0008			// Use of UART2 peripheral
#define __UART_CUSTOM				0x0080			// Use of custom defined UART (the print & read function will be overide in the user pgm)

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
 * WDG configuration
 */
#define __WDG_NONE					0x00			// No WDG
#define __WDG_IWDG					0x01			// with iWDG


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
 * I2C configuration
 */
#define __I2C_NONE					0x00			// No I2C
#define __I2C_ENABLED				0x01			// with I2C


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
 * Polarity
 */
#define __LOW						0x00
#define __HIGH						0x01

/**
 * Config mode
 */
#define __CONFIG_STATIC				0x00			// Non config in memory, all is static
#define __CONFIG_MEMORY				0x01			// Config init at boot and store in memory - reset on reboot
#define __CONFIG_EEPROM				0x02			// Config loaded from EEPROM

/**
 * Some MaxValues
 */

#define __INFINITE_32B				0xFFFFFFFF
#define __INFINITE_64B				0xFFFFFFFFFFFFFFFFLu

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
 * GPIO State
 */
#define __GPIO_VAL_SET			1
#define __GPIO_VAL_RESET		0

/**
 * LOG MODULE DEFINES
 */
#define __LOG_MOD_NONE			0x00000000			// No Log
#define __LOG_MOD_STIMER		0x00000001			// software timer module
#define __LOG_MOD_LOWSIGFOX		0x00000002			// sigfox low level driver whatever it is
#define __LOG_MOD_STKSIGFOX		0x00000004			// sigfox abstraction layer
#define __LOG_MOD_LOWLORADBG	0x00000008			// Lora low level - hardware level
#define __LOG_MOD_LOWLORAINF	0x00000010			// Lora low level - mac level
#define __LOG_MOD_STKLORA		0x00000020			// Lora low level - itsdk level
#define __LOG_MOD_STATEMINF		0x00000040			// State Machine info
#define __LOG_MOD_STATEMDBG		0x00000080			// State Machine debug
#define __LOG_MOD_GNSS			0x00000100			// Gnss & underlaying drivers
#define __LOG_MOD_ACCEL			0x00000200			// Accelerometer & underlaying drivers
#define __LOG_MOD_LOWPOWER		0x00000400			// Print the wakeup reason - see lowpower.c

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

#define __LOG_LEVEL_VERBOSE_DEBUG	5
#define __LOG_LEVEL_VERBOSE_STD 	4
#define __LOG_LEVEL_STANDARD    	3
#define __LOG_LEVEL_QUIET			2
#define __LOG_LEVEL_CRITIAL_ONLY 	1

/**
 * Network to activate
 */
#define __ACTIV_NETWORK_NONE	0x00
#define __ACTIV_NETWORK_SIGFOX	0x01
#define __ACTIV_NETWORK_LORAWAN	0x02

/**
 * Supported SIGFOX Interface
 */
#define __SIGFOX_NONE			0
#define	__SIGFOX_S2LP			1
#define __SIGFOX_WISOL10		2
#define __SIGFOX_SX1276		   10

/**
 * Sigfox Extension
 */
#define __SIGFOX_NONE			0
#define __SIGFOX_MONARCH		1

/**
 * NVM source for Sigfox lib
 */
#define __SFX_NVM_NONE			0					// No NVM use
#define __SFX_NVM_LOCALEPROM	1					// MCU internal EEPROM
#define __SFX_NVM_M95640		2					// External EEPROM type M95640
#define __SFX_NVM_CONFIG_STATIC	3					// Configuration stored in the #define

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
#define __LORAWAN_DR_UNDEFINED	0x0			//   EU868   	  US915
#define __LORAWAN_DR_0			0x1			//     250 - SF12	 980 - SF10
#define __LORAWAN_DR_1			0x2			//     440 - SF11	1760 - SF9
#define __LORAWAN_DR_2			0x3         //     980 - SF10	3125 - SF8
#define __LORAWAN_DR_3			0x4         //    1760 - SF9    5470 - SF7
#define __LORAWAN_DR_4			0x5         //    3125 - SF8
#define __LORAWAN_DR_5			0x6         //    5470 - SF7
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

#define __LORAWAN_ACTIVATION_STATIC	 0x01			// Activation type is decided at compilation time
#define __LORAWAN_ACTIVATION_DYNAMIC 0x02			// Activation type can be changed dynamically (not yet supported)

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

/**
 * More generic Region mapping -> must be equivalent to __LORAWAN_REGION...
 */
#define __PLWAN_REGION_NONE			0x00000000
#define __LPWAN_REGION_AS923		0x00000001
#define __LPWAN_REGION_AU915		0x00000002	 	// Autralia (RCZ4)
#define __LPWAN_REGION_CN470		0x00000004
#define __LPWAN_REGION_CN779		0x00000008
#define __LPWAN_REGION_EU433		0x00000010
#define __LPWAN_REGION_EU868		0x00000020		// Europe (RCZ1)
#define __LPWAN_REGION_KR920		0x00000040		// South Korea (RCZ5)
#define __LPWAN_REGION_IN865		0x00000080		// India (RCZ6)
#define __LPWAN_REGION_US915		0x00000100		// USA, Canada (RCZ2)
#define __LPWAN_REGION_RU864		0x00000200
#define __LPWAN_REGION_MEA868		0x00000400		// Iran, Keyan, Oman, Tunisia, UAE (RCZ1)
#define __LPWAN_REGION_SA915		0x00000800		// South Amnerica : Brazil, Mexico, Puerto Rico (RCZ2)
#define __LPWAN_REGION_SA920		0x00001000		// South America :Argentina, Chile, Colombia, Costa Rica, Ecuador, El Salvador, Guatemala, Honduras, Panama, Peru, Uruguay (RCZ4)
#define __LPWAN_REGION_AP920		0x00002000		// Asia Pacific : Australia, Hong Kong, Malaysia, New Zealand, Singapore, Taiwan, Thailand (RCZ4)
#define __LPWAN_REGION_JP923		0x00004000		// Japan (RCZ3a)



/**
 * List of user expected option to filter the unneeded NMEA messages
 */
#define __GNSS_WITH_2DPOS 		0x0001
#define __GNSS_WITH_3DPOS		0x0002
#define __GNSS_WITH_TIME		0x0004
#define __GNSS_WITH_DATE		0x0008
#define __GNSS_WITH_HDOP		0x0010
#define __GNSS_WITH_PDOP_VDOP	0x0020
#define __GNSS_WITH_SAT_DETAILS	0x0040
#define __GNSS_WITH_SPEED		0x0080
#define __GNSS_WITH_COG			0x0100		// Direction


#endif /* IT_SDK_CONFIG_DEFINES_H_ */
