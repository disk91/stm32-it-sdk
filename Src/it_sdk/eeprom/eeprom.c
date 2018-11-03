/* ==========================================================
 * eeprom.c - Save and Restore data in EEPROM memory
 * 			  Abstraction for underlaying MCU implementation
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 16 sept. 2018
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
 * 
 *
 * ==========================================================
 */
#include <it_sdk/itsdk.h>
#include <it_sdk/debug.h>
#include <it_sdk/eeprom/eeprom.h>
#include <it_sdk/wrappers.h>
#include <stdbool.h>

/**
 * Store a data block into eeprom with the given len in byte
 * Specify a version of the data to be stored. This will be used
 * as a verification at read.
 */
bool eeprom_write(void * data, uint16_t len, uint8_t version) {
	t_eeprom_entry t;
	t.magic = ITDT_EEPROM_MAGIC;
	t.size = len;
	t.version = version;
	t.crc32 = calculateCRC32((uint8_t*)data, len);

	// Write the data header
	_eeprom_write(ITDT_EEPROM_BANK0, 0, (void *) &t, sizeof(t));
	// Write data
	_eeprom_write(ITDT_EEPROM_BANK0, sizeof(t), (void *) data, len);

	_LOG_EEPROM(("[NVM][I] Write %d bytes crc %0X\r\n",len,t.crc32));

	return true;
}

/**
 * Read the EEPROM area to access the data according to the given parameters
 * Verification of magic, size, version and crc to ensure the read data are valid.
 */
bool eeprom_read(void * data, uint16_t len, uint8_t version, uint8_t * versionR) {
	t_eeprom_entry t;

	// Read the data header
	_eeprom_read(ITDT_EEPROM_BANK0, 0, (void *) &t, sizeof(t));

	// Verify different element
	if ( t.magic != ITDT_EEPROM_MAGIC ) {
		_LOG_EEPROM(("[NVM][I] Read invalid magic\r\n"));
		*versionR=0;
		return false;
	}
	*versionR=t.version;
	if ( t.size != len ) {
		_LOG_EEPROM(("[NVM][I] Read invalid size (%d vs %d)\r\n",len,t.size));
		return false;
	}
	if ( t.version != version ) {
		_LOG_EEPROM(("[NVM][I] Read invalid version (%d vs %d)\r\n",version,t.version));
		return false;
	}

	// Read the data
	_eeprom_read(ITDT_EEPROM_BANK0, sizeof(t), (void *) data, len);
	uint32_t _crc = calculateCRC32((uint8_t*)data, len);

	if ( t.crc32 != _crc ) {
		_LOG_EEPROM(("[NVM][I] Read invalid crc\r\n"));
		return false;
	}

	return true;
}
