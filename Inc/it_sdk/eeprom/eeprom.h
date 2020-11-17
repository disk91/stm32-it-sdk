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

#define ITDT_EEPROM_MAGIC_USERLAND 0xCCA3
#define ITDT_EEPROM_MAGIC_CONFIG   0xA5FC
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

itsdk_bool_e eeprom_write_config(void * data, uint16_t len, uint8_t version);
itsdk_bool_e eeprom_read_config(void * data, uint16_t len, uint8_t version, uint8_t * versionR, itsdk_bool_e bypassTest);
itsdk_bool_e eeprom_write_userland(uint32_t offset, void * data, uint16_t len, itsdk_bool_e initialize);
itsdk_bool_e eeprom_read_userland(uint32_t offset, void * data, uint16_t len);
itsdk_bool_e eeprom_getPostConfigOffset(uint32_t * _offset);
itsdk_bool_e eeprom_getConfigOffset(uint32_t * _offset);
itsdk_bool_e eeprom_getConfigSize(uint32_t * _size);
itsdk_bool_e eeprom_getUserLandOffset(uint32_t * _offset);
void eeprom_clearAllEprom();

#endif /* IT_SDK_EEPROM_EEPROM_H_ */
