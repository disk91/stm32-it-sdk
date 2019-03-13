/* ==========================================================
 * configDrivers.h - 
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 7 nov. 2018
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

#ifndef INC_IT_SDK_CONFIGDRIVERS_H_
#define INC_IT_SDK_CONFIGDRIVERS_H_

#include <it_sdk/config_defines.h>


// *************************************** EEPROM *****************************************************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// | SDK SETTING                   | USER SELECTED VALUE                  | SETTING DESCRIPTION                   |
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ------------------------------------------------------------------------
// EEPROM : M95640

#define ITSDK_DRIVERS_M95640					__DISABLE					// this driver is activated in build
#if ITSDK_DRIVERS_M95640 == __ENABLE
	#define __SPI_INCLUDED
	#include "spi.h"
#endif
#define ITSDK_DRIVERS_M95640_SPI				hspi1						// Spi port to be used for communications
#define ITSDK_DRIVERS_M95640_SPI_CS_BANK		__BANK_B					// EEPROM Chip Select PIN LAYOUT (Activ Low)
#define ITSDK_DRIVERS_M95640_SPI_CS_PIN			__LP_GPIO_4					//   Config as output, pull-up init High


// ------------------------------------------------------------------------
// Temp / Humidity / Pressure : M95640

#define ITSDK_DRIVERS_BME280					__DISABLE					// BOSH BME 280 Temp / Hygro / Pressure
#if ITSDK_DRIVERS_BME280 == __ENABLE
	#ifndef __I2C_INCLUDED
	  #define __I2C_INCLUDED
	  #include "i2c.h"
	#endif
	#include <drivers/temphygropressure/bosh_bme280/bme280.h>
#endif
#define ITSDK_DRIVERS_BME280_I2C				hi2c1						// I2C port to be used for communications
#define __BME280_SDO_HIGH						0x1
#define __BME280_SDO_LOW						0x0
#define ITSDK_DRIVERS_BME280_ADDRESS			(DRIVER_BME280_DEVICE_ADR | __BME280_SDO_LOW)	//   Device address base on electrical conf

// ------------------------------------------------------------------------
// Light sensor : MAX44009

#define ITSDK_DRIVERS_MAX44009					__DISABLE
#if ITSDK_DRIVERS_MAX44009 == __ENABLE
	#ifndef __I2C_INCLUDED
	  #define __I2C_INCLUDED
	  #include "i2c.h"
	#endif
	#include <drivers/light/max44009/max44009.h>
#endif
#define ITSDK_DRIVERS_MAX44009_I2C				hi2c1						// I2C port to be used for communications
#define __MAX44009_A0_HIGH						0x1
#define __MAX44009_A0_LOW						0x0
#define ITSDK_DRIVERS_MAX44009_ADDRESS			(DRIVER_MAX44009_DEVICE_ADR | __MAX44009_A0_LOW)	// Device address, based on the schematics

// ------------------------------------------------------------------------
// Gauge : MAX17205

#define ITSDK_DRIVERS_MAX17205					__DISABLE
#if ITSDK_DRIVERS_MAX17205 == __ENABLE
	#ifndef __I2C_INCLUDED
	  #define __I2C_INCLUDED
	  #include "i2c.h"
	#endif
	#include <drivers/gauge/max17205/max17205.h>
#endif
#define ITSDK_DRIVERS_MAX17205_I2C				hi2c1						// I2C port to be used for communications
#define ITSDK_DRIVERS_MAX17205_ALRT1_BANK		__BANK_B					// ALERT PIN Configuration
#define ITSDK_DRIVERS_MAX17205_ALRT1_PIN		__LP_GPIO_7					//   __LP_GPIO_NONE if not used
#define ITSDK_DRIVERS_MAX17205_RSENSE_MOHM		1000						// RSense in mOhm

// ------------------------------------------------------------------------
// NFC Tag : ST25DV

#define ITSDK_DRIVERS_ST25DV					__DISABLE
#define ITSDK_DRIVERS_ST25DV_WITH_SERIALUZ		__DISABLE					// Activate the serial User communication code
#define ITSDK_DRIVERS_ST25DV_WITH_SERIALFTM		__DISABLE					// Activate the serial FTM communication code

#if ITSDK_DRIVERS_ST25DV == __ENABLE
	#ifndef __I2C_INCLUDED
	  #define __I2C_INCLUDED
	  #include "i2c.h"
	#endif
	#include <drivers/nfc/st25dv/st25dv.h>
#endif
#define ITSDK_DRIVERS_ST25DV_I2C				hi2c1						// I2C port to be used for communications
#define ITSDK_DRIVERS_ST25DV_GPO_BANK			__BANK_B					// GPO Pin
#define ITSDK_DRIVERS_ST25DV_GPO_PIN			__LP_GPIO_5					//   __LP_GPIO_NONE if not used
#define ITSDK_DRIVERS_ST25DV_LPD_BANK			__BANK_B					// LPD Pin
#define ITSDK_DRIVERS_ST25DV_LPD_PIN			__LP_GPIO_6					//   __LP_GPIO_NONE if not used

#define ITSDK_DRIVERS_ST25DV_I2C_PASSWORD		0x0000000000000000			// changeme => I2C password will be setup on device when != 0
																			// RF Password are 0000000000000000 by default. Only RF can set these password

																			// Only the first 1KB is accessible with a standard NFC read
																			// for the rest of the memory area an extended read is needed.
																			// For a larger reader compatibility, I assume it is better to have
																			//  the Zone 1 & Zone 2 under this 1KB limit.
#define ITSDK_DRIVERS_ST25DV_USER_Z1_SIZE		1024						// Size in byte for User Zone 1 - This zone have no security option
#define ITSDK_DRIVERS_ST25DV_USER_Z1_ACCESS		_ST25DV_ACCESS_RW_OPEN		// Zone 1 is read only

#define ITSDK_DRIVERS_ST25DV_USER_Z2_SIZE		1024						// Size in byte for User Zone 2
#define ITSDK_DRIVERS_ST25DV_USER_Z2_PASS		__ENABLE
#define ITSDK_DRIVERS_ST25DV_USER_Z2_ACCESS		_ST25DV_ACCESS_RO_OPEN		// Zone 2 is read only

#define ITSDK_DRIVERS_ST25DV_USER_Z3_SIZE		2048						// Size in byte for User Zone 2
#define ITSDK_DRIVERS_ST25DV_USER_Z3_PASS		__ENABLE
#define ITSDK_DRIVERS_ST25DV_USER_Z3_ACCESS		_ST25DV_ACCESS_RW_RWSECURED	// Zone 3 is RW both secured by a password

#define ITSDK_DRIVERS_ST25DV_USER_Z4_SIZE		4096						// The reality is Zone 4 is up the memory size.
#define ITSDK_DRIVERS_ST25DV_USER_Z4_PASS		__ENABLE
#define ITSDK_DRIVERS_ST25DV_USER_Z4_ACCESS		_ST25DV_ACCESS_RW_RWSECURED	// Zone 4 is RW both secured by a password

#define ITSDK_DRIVERS_ST25DV_SERIALUZ_ZONE		ST25DV_USERZONE_1			// The serial communication module on User Zone is using Zone 2
#define ITSDK_DRIVERS_ST25DV_SERIALUZ_OFFSET	42							// offset in block (x32b) in the USER Zone 42 = 168

#endif /* INC_IT_SDK_CONFIGDRIVERS_H_ */
