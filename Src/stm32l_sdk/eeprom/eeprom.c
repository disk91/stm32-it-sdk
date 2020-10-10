/* ==========================================================
 * eeprom.c - Manage NVM (EEPROM)
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
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
#include <it_sdk/wrappers.h>
#include <stm32l_sdk/eeprom/eeprom.h>
#include <it_sdk/debug.h>
#include <stdbool.h>
#include <it_sdk/logger/error.h>


/**
 * Write in the eeprom the given data.
 * A bank is used as a entry index to manage multiple content.
 * Offset allows to write 1 bank in multiple operation where offset is the pointer for bank start
 *  offset is aligned on 32b words
 * Actually bank is not supported, assuming is 0
 * @TODO : manage bank
 */
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len) {
	uint8_t *  _data = (uint8_t *)data;
	uint32_t   _eepromAddr;
	uint32_t   v;

	if ( bank != 0 || (offset + len) > EEPROM_SIZE) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_OUTOFBOUNDS,len);
	}
	_eepromAddr = (uint32_t)(EEPROM_START_ADDR+offset);
	if ( (_eepromAddr & 0x3) != 0 ) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_NOTALIGNED,1);
	    return false;
	}

	// Unlock EEPROM
	if (FLASH->PECR & FLASH_PECR_PELOCK) {
			FLASH->PEKEYR = FLASH_PEKEY1;
			FLASH->PEKEYR = FLASH_PEKEY2;
	}
	// Copy data
	for (int i = 0; i < len; i += 4) {
		v = _data[i] << 24;
		v+= (i+1 < len)?_data[i+1]<<16:0;
		v+= (i+2 < len)?_data[i+2]<<8:0;
		v+= (i+3 < len)?_data[i+3]:0;
		if (v != __eepromRead(_eepromAddr)) __eepromWrite(_eepromAddr,v);
		_eepromAddr+=4;
	}
	// Lock EEPROM
	FLASH->PECR |= FLASH_PECR_PELOCK;
	return true;
}

/**
 * Read a block of data from the EEPROM
 * Offset is to add an offset to bank start - Offset is aligned don 32b word
 */
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len) {
	uint8_t  * _data = (uint8_t *)data;
	uint32_t   _eepromAddr;
	uint32_t   v;

	if ( bank != 0 || (offset + len) > EEPROM_SIZE) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_OUTOFBOUNDS,len);
	}
	_eepromAddr = (uint32_t)(EEPROM_START_ADDR+offset);
	if ( (_eepromAddr & 0x3) != 0 ) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_NOTALIGNED,0);
	    return false;
	}

	// Read data
	for (int i = 0; i < len; i += 4) {
		v = __eepromRead(_eepromAddr);
		_data[i]=(v & 0xFF000000) >> 24;
		if ( i+1 < len) _data[i+1]=(v & 0x00FF0000) >> 16;
		if ( i+2 < len) _data[i+2]=(v & 0x0000FF00) >> 8;
		if ( i+3 < len) _data[i+3]=(v & 0x000000FF);
		_eepromAddr+=4;
	}
	return true;
}


/**
 * Read a word in EEPROM area
 */
uint32_t __eepromRead(uint32_t addr) {
	return (*(volatile uint32_t*)addr);
}

/**
 * Write a word in EEPROM area
 * Return false when an error occured
 */
bool __eepromWrite(uint32_t addr, uint32_t v) {

	uint16_t tmout = 10000;
	while ( (FLASH->SR & FLASH_SR_BSY) && tmout) tmout--;
	if ( tmout == 0 ) return false;

	// Clear the FTDW bit (data will be erased before write if it non zero)
	FLASH->PECR &= (uint32_t)(~(uint32_t)FLASH_PECR_FIX);

	*(volatile uint32_t *)addr = v;

	while ( (FLASH->SR & FLASH_SR_BSY) && tmout) tmout--;
	if ( tmout == 0 ) return false;

	return true;
}



#endif
