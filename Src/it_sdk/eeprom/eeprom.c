/* ==========================================================
 * eeprom.c - Save and Restore data in EEPROM memory
 * 			  Abstraction for underlaying MCU implementation
 * 			  Store configuration structure
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
 * The data are stored after the SecureStore when activated.
 *
 * ==========================================================
 */
#include <it_sdk/itsdk.h>
#include <it_sdk/debug.h>
#include <it_sdk/eeprom/eeprom.h>
#include <it_sdk/wrappers.h>
#include <stdbool.h>
#if ITSDK_WITH_SECURESTORE == __ENABLE
  #include <it_sdk/eeprom/securestore.h>
#endif
#if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
  #include <it_sdk/logger/error.h>
#endif
#if (ITSDK_WITH_SIGFOX_LIB == __ENABLE)
  #include <it_sdk/sigfox/sigfox.h>
#endif

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
	uint32_t offset = 0, sstore=0, ssError=0, sSigfox=0;
  #if ITSDK_WITH_SECURESTORE == __ENABLE
	itsdk_secstore_getStoreSize(&sstore);
  #endif
  #if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
	itsdk_error_getSize(&ssError);
  #endif
  #if (ITSDK_WITH_SIGFOX_LIB == __ENABLE)
	itsdk_sigfox_getNvmSize(&sSigfox);
  #endif
	offset += sstore + ssError + sSigfox;

	// Write the data header
	_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) &t, sizeof(t));
	// Write data
	_eeprom_write(ITDT_EEPROM_BANK0, offset+sizeof(t), (void *) data, len);

	_LOG_EEPROM(("[NVM][I] Write %d bytes crc %0X\r\n",len,t.crc32));

	return true;
}

/**
 * Read the EEPROM area to access the data according to the given parameters
 * Verification of magic, size, version and crc to ensure the read data are valid.
 * In the EEPROM we have
 * ---> SecureStore
 * ---> ErrorReport
 * ---> Configuration
 */
bool eeprom_read(void * data, uint16_t len, uint8_t version, uint8_t * versionR) {
	t_eeprom_entry t;
	uint32_t offset = 0, sstore=0, ssError=0, sSigfox=0;
  #if ITSDK_WITH_SECURESTORE == __ENABLE
	itsdk_secstore_getStoreSize(&sstore);
  #endif
  #if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
	itsdk_error_getSize(&ssError);
  #endif
  #if (ITSDK_WITH_SIGFOX_LIB == __ENABLE)
	itsdk_sigfox_getNvmSize(&sSigfox);
  #endif
	offset += sstore + ssError + sSigfox;

	// Read the data header
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) &t, sizeof(t));

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
	_eeprom_read(ITDT_EEPROM_BANK0, offset+sizeof(t), (void *) data, len);
	uint32_t _crc = calculateCRC32((uint8_t*)data, len);

	if ( t.crc32 != _crc ) {
		_LOG_EEPROM(("[NVM][I] Read invalid crc\r\n"));
		return false;
	}

	return true;
}

/**
 * Returns the offset of the first byte following the configuration & other SDK
 * EEPROM reserved zone. Make sure your data post this zone will be preserved if you
 * change the config area size.
 */
bool eeprom_getPostConfigOffset(uint32_t * _offset) {
	t_eeprom_entry t;
	uint32_t offset = 0, sstore=0, ssError=0, sSigfox=0;
  #if ITSDK_WITH_SECURESTORE == __ENABLE
	itsdk_secstore_getStoreSize(&sstore);
  #endif
  #if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
	itsdk_error_getSize(&ssError);
  #endif
  #if (ITSDK_WITH_SIGFOX_LIB == __ENABLE)
	itsdk_sigfox_getNvmSize(&sSigfox);
  #endif
	offset += sstore + ssError + sSigfox;

	// Read the data header
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) &t, sizeof(t));

	*_offset = offset + sizeof(t) + t.size;
	return true;

}
