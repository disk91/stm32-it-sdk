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
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/wrappers.h>
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
 * Store a data block into eeprom in the zone available for the user
 * offset is inside the userland (starting at 0)
 * when initialize = BOOL_TRUE if the magic is not set, do it. otherwise retrun an error
 */
itsdk_bool_e eeprom_write_userland(uint32_t offset, void * data, uint16_t len, itsdk_bool_e initialize ) {

	t_eeprom_entry t;
	uint32_t _offset = 0;
	eeprom_getUserLandOffset(&_offset);
	_eeprom_read(ITDT_EEPROM_BANK0, _offset, (void *) &t, sizeof(t));

	if ( t.magic != ITDT_EEPROM_MAGIC_USERLAND ) {
		if ( initialize ) {
			t.magic = ITDT_EEPROM_MAGIC_USERLAND;
			t.size = ITSDK_EPROM_SIZE - _offset;
			t.version = 0;
			t.crc32 = 0;
			// write header
			_eeprom_write(ITDT_EEPROM_BANK0, _offset, (void *) &t, sizeof(t));
			_LOG_EEPROM(("[NVM][I] UL Hedaer created\r\n",len));
		} else {
			_LOG_EEPROM(("[NVM][E] UL Bad Header\r\n",len));
			return BOOL_FALSE;
		}
	}

	// verify the location
	_offset = _offset + sizeof(t) + offset;
	if ( (_offset + len) > ITSDK_EPROM_SIZE ) {
		_LOG_EEPROM(("[NVM][E] UL Write out of eeprom area\r\n",len));
		return BOOL_FALSE;
	}

	// Write data
	_eeprom_write(ITDT_EEPROM_BANK0, _offset, (void *) data, len);
	_LOG_EEPROM(("[NVM][I] UL Write %d bytes\r\n",len));

	return BOOL_TRUE;
}

/**
 * Read the EEPROM configuration zone area to access the data according to the given parameters
 * Verification of magic, size, version and crc to ensure the read data are valid.
 * In the EEPROM we have
 * ---> SecureStore
 * ---> ErrorReport
 * ---> Sigfox Nvm
 * ---> Configuration
 * ---> UserLand (*) here
 */
itsdk_bool_e eeprom_read_userland(uint32_t offset, void * data, uint16_t len) {
	t_eeprom_entry t;
	uint32_t _offset = 0;
	eeprom_getUserLandOffset(&_offset);

	// Read the data header
	_eeprom_read(ITDT_EEPROM_BANK0, _offset, (void *) &t, sizeof(t));

	// Verify different element
	if ( t.magic != ITDT_EEPROM_MAGIC_USERLAND ) {
		_LOG_EEPROM(("[NVM][I] Read UL invalid magic\r\n"));
		return BOOL_FALSE;
	}

	// verify the location
	_offset = _offset + sizeof(t) + offset;
	if ( (_offset + len) > ITSDK_EPROM_SIZE ) {
		_LOG_EEPROM(("[NVM][E] UL Read out of eeprom area\r\n",len));
		return BOOL_FALSE;
	}

	// Read the data
	_eeprom_read(ITDT_EEPROM_BANK0, _offset, (void *) data, len);

	return BOOL_TRUE;
}



/**
 * Store a data block into eeprom config zone with the given len in byte
 * Specify a version of the data to be stored. This will be used
 * as a verification at read.
 * ---> SecureStore
 * ---> ErrorReport
 * ---> Sigfox Nvm
 * ---> Configuration (*) here
 * ---> UserLand
 */
itsdk_bool_e eeprom_write_config(void * data, uint16_t len, uint8_t version) {
	t_eeprom_entry t;
	t.magic = ITDT_EEPROM_MAGIC_CONFIG;
	t.size = len;
	t.version = version;
	t.crc32 = itsdk_computeCRC32((uint8_t*)data, len);

	uint32_t offset = 0;
	eeprom_getConfigOffset(&offset);

	// Write the data header
	_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) &t, sizeof(t));
	// Write data
	_eeprom_write(ITDT_EEPROM_BANK0, offset+sizeof(t), (void *) data, len);

	_LOG_EEPROM(("[NVM][I] Write %d bytes crc %0X\r\n",len,t.crc32));

	return BOOL_TRUE;
}


/**
 * Read the EEPROM configuration zone area to access the data according to the given parameters
 * Verification of magic, size, version and crc to ensure the read data are valid.
 * In the EEPROM we have
 * ---> SecureStore
 * ---> ErrorReport
 * ---> Sigfox Nvm
 * ---> Configuration (*) here
 * ---> UserLand
 * BypassTest option allows to read the configuration even if the size has changed an dcrc is invalid
 * Version change is detected with a normal read (bypassTest=BOOL_TRUE), get a False with versiuonR != 0
 */
itsdk_bool_e eeprom_read_config(void * data, uint16_t len, uint8_t version, uint8_t * versionR, itsdk_bool_e bypassTest) {
	t_eeprom_entry t;
	uint32_t offset = 0;
	eeprom_getConfigOffset(&offset);

	// Read the data header
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) &t, sizeof(t));

	// Verify different element
	if ( t.magic != ITDT_EEPROM_MAGIC_CONFIG ) {
		_LOG_EEPROM(("[NVM][I] Read invalid magic\r\n"));
		*versionR=0;
		return BOOL_FALSE;
	}
	*versionR=t.version;

	if ( t.version != version ) {
		_LOG_EEPROM(("[NVM][I] Read invalid version (%d vs %d)\r\n",version,t.version));
		return BOOL_FALSE;
	}

	if ( t.size != len && ! bypassTest ) {
		_LOG_EEPROM(("[NVM][I] Read invalid size (%d vs %d)\r\n",len,t.size));
		return BOOL_FALSE;
	} else {
		len = t.size;
	}

	// Read the data
	_eeprom_read(ITDT_EEPROM_BANK0, offset+sizeof(t), (void *) data, len);
	uint32_t _crc = itsdk_computeCRC32((uint8_t*)data, len);

	if ( t.crc32 != _crc && ! bypassTest ) {
		_LOG_EEPROM(("[NVM][I] Read invalid crc\r\n"));
		return BOOL_FALSE;
	}

	return BOOL_TRUE;
}

/**
 * Compute the EEPROM Userland offset
 * Memory have SecureStore then Log then Sigfox config, then Device config, Then User land
 */
itsdk_bool_e eeprom_getUserLandOffset(uint32_t * _offset) {
  eeprom_getConfigOffset(_offset);
  *_offset += sizeof(t_eeprom_entry)+sizeof(itsdk_configuration_nvm_t);
  return BOOL_TRUE;
}


/**
 * Compute the EEPROM Config offset
 * Memory have SecureStore then Log then Sigfox config, then Device config
 */
itsdk_bool_e eeprom_getConfigOffset(uint32_t * _offset) {
  uint32_t sstore=0, ssError=0, sSigfox=0;
  #if ITSDK_WITH_SECURESTORE == __ENABLE
	itsdk_secstore_getStoreSize(&sstore);
  #endif
  #if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
	itsdk_error_getSize(&ssError);
  #endif
  #if (ITSDK_WITH_SIGFOX_LIB == __ENABLE)
	itsdk_sigfox_getNvmSize(&sSigfox);
  #endif
  *_offset += sstore + ssError + sSigfox;
  return BOOL_TRUE;
}

/**
 * Get the EEprom config size from the config header
 * it includes the header size
 */
itsdk_bool_e eeprom_getConfigSize(uint32_t * _size) {
	t_eeprom_entry t;

	uint32_t offset = 0;
	eeprom_getConfigOffset(&offset);

	// Read the data header
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) &t, sizeof(t));

	*_size = sizeof(t) + t.size;
	return BOOL_TRUE;
}


/**
 * Returns the offset of the first byte following the configuration & other SDK
 * EEPROM reserved zone. Make sure your data post this zone will be preserved if you
 * change the config area size.
 */
itsdk_bool_e eeprom_getPostConfigOffset(uint32_t * _offset) {

	uint32_t size = 0;
	uint32_t offset = 0;

	eeprom_getConfigOffset(&offset);
	eeprom_getConfigSize(&size);

	*_offset = offset +size;
	return BOOL_TRUE;
}

/**
 * Clear (write 0) into the whole BANK0
 */
void eeprom_clearAllEprom() {
	uint32_t v  = 0;
	for ( int i = 0 ; i < ITSDK_EPROM_SIZE ; i+=sizeof(v) ) {
		_eeprom_write(ITDT_EEPROM_BANK0, i, (void *) &v, sizeof(v));
	}
}

