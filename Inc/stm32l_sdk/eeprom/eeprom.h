/* ==========================================================
 * eeprom.h - Manage EEPROM (NVM)
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 16 sept. 2018
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

#ifndef STM32L_SDK_EEPROM_EEPROM_H_
#define STM32L_SDK_EEPROM_EEPROM_H_
#include <stdbool.h>

// Data EEPROM start and end address
#if    ITSDK_DEVICE == __DEVICE_STM32L011D4 \
	|| ITSDK_DEVICE == __DEVICE_STM32L031K6 \
	|| ITSDK_DEVICE == __DEVICE_STM32L053R8 \
	|| ITSDK_DEVICE == __DEVICE_STM32L072XX \
	|| ITSDK_DEVICE == __DEVICE_STM32L082XX \
	|| ITSDK_DEVICE == __DEVICE_STM32L052T8
   #define EEPROM_START_ADDR     	0x08080000
#endif
#if ITSDK_DEVICE == __DEVICE_STM32L011D4
   #define EEPROM_SIZE				512
#elif ITSDK_DEVICE == __DEVICE_STM32L031K6
	#define EEPROM_SIZE			   1024
#elif ITSDK_DEVICE == __DEVICE_STM32L053R8
   #define EEPROM_SIZE				2048
#elif ITSDK_DEVICE == __DEVICE_STM32L072XX \
  || ITSDK_DEVICE == __DEVICE_STM32L082XX
	#define EEPROM_SIZE				6144
#elif ITSDK_DEVICE == __DEVICE_STM32L052T8
	#define EEPROM_SIZE				2048
#endif
#define EEPROM_END_ADDR 	   (DATA_EEPROM_START_ADDR + EEPROM_SIZE)


uint32_t __eepromRead(uint32_t addr);
bool __eepromWrite(uint32_t addr, uint32_t v);

#endif /* STM32L_SDK_EEPROM_EEPROM_H_ */
