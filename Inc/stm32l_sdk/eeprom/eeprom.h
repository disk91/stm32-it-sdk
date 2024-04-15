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
	|| ITSDK_DEVICE == __DEVICE_STM32L052T8
   #define EEPROM_START_ADDR     	0x08080000
#endif
#if ITSDK_DEVICE == __DEVICE_STM32L011D4
   #define EEPROM_SIZE				512
#elif ITSDK_DEVICE == __DEVICE_STM32L031K6
	#define EEPROM_SIZE			   1024
#elif ITSDK_DEVICE == __DEVICE_STM32L053R8
   #define EEPROM_SIZE				2048
#elif ITSDK_DEVICE == __DEVICE_STM32L072XX
	#define EEPROM_SIZE				6144
#elif ITSDK_DEVICE == __DEVICE_STM32L052T8
	#define EEPROM_SIZE				2048
#elif ITSDK_DEVICE == __DEVICE_STM32WLE5JC
	#define EEPROM_END_ADDR 		0x0803F800						// Max memory to store eeprom at the end of the memory zone (0x803F8 to 0x803E0 seems potentially used)
  	#define EEPROM_PAGE_SIZE 		2048							// 2K pages - hardware related, not a parameter
	#define EEPROM_LINE_PER_PAGE	127								// Number of Lines (memory block per page)
  	#define EEPROM_SIZE_WITH_OVERHEAD (ITSDK_EPROM_SIZE + 4096)		// Add 4K (2 pages, for page switch and structure overhead, works with 2K-8K eeprom)
																	// @TODO - improve this with not fixed value but parameter including the aging
																	// currently, best is to commission more memory than required.
	#define EEPROM_AGING_REPORT		8000							// when a page has been reset this amount of time, report it to user
#endif

#ifndef EEPROM_END_ADDR
   #define EEPROM_END_ADDR 	   (DATA_EEPROM_START_ADDR + EEPROM_SIZE)
#endif


uint32_t __eepromRead(uint32_t addr);
bool __eepromWrite(uint32_t addr, uint32_t v);

#endif /* STM32L_SDK_EEPROM_EEPROM_H_ */
