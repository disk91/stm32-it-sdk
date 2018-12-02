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


// *************************************** EEPROM *****************************************************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// | SDK SETTING                   | USER SELECTED VALUE                  | SETTING DESCRIPTION                   |
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define ITSDK_DRIVERS_M95640					1							// this driver is activated in build
#if ITSDK_DRIVERS_M95640 == 1
	#include "spi.h"
#endif
#define ITSDK_DRIVERS_M95640_SPI				hspi1						// Spi port to be used for communications
#define ITSDK_DRIVERS_M95640_SPI_CS_BANK		__BANK_B					// EEPROM Chip Select PIN LAYOUT (Activ Low)
#define ITSDK_DRIVERS_M95640_SPI_CS_PIN			__LP_GPIO_4					//   Config as output, pull-up init High



#endif /* INC_IT_SDK_CONFIGDRIVERS_H_ */
