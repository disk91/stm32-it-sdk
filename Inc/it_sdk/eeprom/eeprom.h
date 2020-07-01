/* ==========================================================
 * eeprom.h - 
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 16 sept. 2018
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

#ifndef IT_SDK_EEPROM_EEPROM_H_
#define IT_SDK_EEPROM_EEPROM_H_

#include <it_sdk/itsdk.h>
#include <it_sdk/config.h>

#define ITDT_EEPROM_MAGIC 0xA5FC
#define ITDT_EEPROM_BANK0 0
#define ITDT_EEPROM_BANK1 1
#define ITDT_EEPROM_BANK2 2
#define ITDT_EEPROM_BANK3 3

typedef struct s_eeprom_entry {
	uint16_t  magic;				// Magic to see of data have been written here
	uint8_t	  version;				// Version for managing update on flashing
	uint16_t  size;					// data size
	uint32_t  crc32;				// Crc32 for verifying data content value
	uint8_t	  align[3];				// align on 32b structure
} t_eeprom_entry;

bool eeprom_write(void * data, uint16_t len, uint8_t version);
bool eeprom_read(void * data, uint16_t len, uint8_t version, uint8_t * versionR);
bool eeprom_getPostConfigOffset(uint32_t * _offset);
bool eeprom_getConfigOffset(uint32_t * _offset);
bool eeprom_getConfigSize(uint32_t * _size);
void eeprom_clearAllEprom();

#endif /* IT_SDK_EEPROM_EEPROM_H_ */
