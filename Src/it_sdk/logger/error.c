/* ==========================================================
 * error.c - Store error code in a persistant storage for later
 *           analysis
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 17 feb. 2019
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
 * ==========================================================
 */
#include <it_sdk/config.h>
#include <it_sdk/logger/error.h>
#if ITSDK_WITH_ERROR_RPT == __ENABLE

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/eeprom/eeprom.h>

#if ITSDK_WITH_CONSOLE == __ENABLE
#include <it_sdk/console/console.h>
static itsdk_console_chain_t __console_errorMng;
static itsdk_console_return_e _itsdk_error_consolePriv(char * buffer, uint8_t sz);
#endif

#if ITSDK_WITH_SECURESTORE == __ENABLE
#include <it_sdk/eeprom/securestore.h>
#endif


// =================================================================================
// Technical API with NVM storage
// =================================================================================


/**
 * Get the header of the error blocks
 */
__weak itsdk_error_ret_e _itsdk_error_readHeader(itsdk_error_head_t * header) {
	uint32_t offset = 0;
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	  itsdk_secstore_getStoreSize(&offset);
	#endif
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) header, sizeof(itsdk_error_head_t));
	if ( header->magic != ITSDK_ERROR_STRUCT_MAGIC) return ITSDK_ERROR_FAILED;
	return ITSDK_ERROR_SUCCESS;
}

/**
 * Update the header of the error blocks
 */
__weak itsdk_error_ret_e _itsdk_error_writeHeader(itsdk_error_head_t * header) {
	uint32_t offset = 0;
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	  itsdk_secstore_getStoreSize(&offset);
	#endif
	_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) header, sizeof(itsdk_error_head_t));
	return ITSDK_ERROR_SUCCESS;
}


/**
 * Write the error in the NVM.
 * This function can be override if the MCU EEPROM is not used for this purpose.
 *
 */
__weak itsdk_error_ret_e _itsdk_error_write(uint16_t blockId, itsdk_error_entry_t * entry) {
	uint32_t offset = 0;
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	  itsdk_secstore_getStoreSize(&offset);
	#endif
	_eeprom_write(
		ITDT_EEPROM_BANK0,
		offset+sizeof(itsdk_error_head_t)+(sizeof(itsdk_error_entry_t)*blockId),
		(void *)entry, sizeof(itsdk_error_entry_t)
	);
	return ITSDK_ERROR_SUCCESS;
}

/**
 * Read a given error Id from the NVM
 */
__weak itsdk_error_ret_e _itsdk_error_read(uint16_t blockId,itsdk_error_entry_t * e) {
	uint32_t offset = 0;
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	  itsdk_secstore_getStoreSize(&offset);
	#endif
	_eeprom_read(
		ITDT_EEPROM_BANK0,
		offset+sizeof(itsdk_error_head_t)+(sizeof(itsdk_error_entry_t)*blockId),
		(void *)e, sizeof(itsdk_error_entry_t)
	);
	return ITSDK_ERROR_SUCCESS;
}

// =================================================================================
// Public API
// =================================================================================


/**
 * Init the error block structure if needed
 * This function is called on every device restart
 */
itsdk_error_ret_e itsdk_error_setup() {
	itsdk_error_head_t h;
	if ( _itsdk_error_readHeader(&h) == ITSDK_ERROR_FAILED ) {
		// init the structure
		h.magic = ITSDK_ERROR_STRUCT_MAGIC;
		h.readPt = 0;
		h.writePt = 0;
		_itsdk_error_writeHeader(&h);
	}
#if ITSDK_WITH_CONSOLE == __ENABLE
	__console_errorMng.console_private = _itsdk_error_consolePriv;
	__console_errorMng.console_public = NULL;
	__console_errorMng.next = NULL;
	itsdk_console_registerCommand(&__console_errorMng);
#endif

	return ITSDK_ERROR_SUCCESS;
}


/**
 * Register an error into the NVM
 * The error is enriched with the value when needed and written in the NVM
 * The NVM write can be override.
 * When the level is critical it loop forever after printing an error message
 */
itsdk_error_ret_e itsdk_error_report(uint32_t error,uint16_t value) {
	// Add value into error when needed
	if ( ( error & ITSDK_ERROR_WITH_VALUE ) > 0 ) {
		error |= (value << ITSDK_ERROR_VALUE_SHIFT) & ITSDK_ERROR_VALUE_MASK;
	}
	uint64_t time = itsdk_time_get_ms() / 1000;
	itsdk_error_entry_t e;
	e.error = error;
	e.timeS = (uint32_t)time;

	itsdk_error_head_t h;
	if ( _itsdk_error_readHeader(&h) == ITSDK_ERROR_FAILED ) return ITSDK_ERROR_FAILED;

	// Register error
	_itsdk_error_write(h.writePt, &e);

	// Update the pointers
	h.writePt = (h.writePt + 1) % ITSDK_ERROR_BLOCKS;
	if ( h.writePt == h.readPt ) {
		h.readPt = (h.readPt + 1) % ITSDK_ERROR_BLOCKS;
	}
	_itsdk_error_writeHeader(&h);

	char t = 'S';
	if ((error & ITSDK_ERROR_TYPE_MASK) == ITSDK_ERROR_TYPE_APP) {
		t = 'A';
	}

	// Manage critical level
	if ( (error & ITSDK_ERROR_LEVEL_FATAL ) == ITSDK_ERROR_LEVEL_FATAL ){
		log_error("[CRITICAL ERROR] %c 0x%08X\r\n",t,error);
		while(1);
	} else if ( (error & ITSDK_ERROR_LEVEL_ERROR ) == ITSDK_ERROR_LEVEL_ERROR ){
		log_error("[ERROR] %c 0x%08X\r\n",t,error);
	} else if ( (error & ITSDK_ERROR_LEVEL_WARN ) == ITSDK_ERROR_LEVEL_WARN ){
		log_warn("[WARN] %c 0x%08X\r\n",t,error);
	} else if ( (error & ITSDK_ERROR_LEVEL_INFO ) == ITSDK_ERROR_LEVEL_INFO ){
		log_info("[INFO] %c 0x%08X\r\n",t,error);
	}
	return ITSDK_ERROR_SUCCESS;
}

/**
 * Read a given error Id.
 * When blockId is ITSDK_ERROR_FIRSTBLOCK the first available block is returned
 * Returns the next blockId to be read. ITSDK_ERROR_LASTBLOCK when no more to read
 * The blockId is updated with next block Id value.
 *
 */
itsdk_error_ret_e itsdk_error_get(uint16_t * blockId,itsdk_error_entry_t * e) {

	itsdk_error_head_t h;
	if ( _itsdk_error_readHeader(&h) == ITSDK_ERROR_SUCCESS ) {
		// Manage blockId request
		if ( *blockId == ITSDK_ERROR_FIRSTBLOCK ) {
			*blockId = h.readPt;
		}
		if ( *blockId != h.writePt ) {
			if ( *blockId < ITSDK_ERROR_BLOCKS ) {
				// Read the block
				if ( _itsdk_error_read(*blockId,e) == ITSDK_ERROR_SUCCESS ) {
					*blockId = (*blockId + 1) % ITSDK_ERROR_BLOCKS;
					if ( *blockId == h.writePt ) *blockId = ITSDK_ERROR_LASTBLOCK;
					return ITSDK_ERROR_SUCCESS;
				}
			}
		}
	}
	e->error = 0;
	e->timeS = 0;
	*blockId = ITSDK_ERROR_LASTBLOCK;
	return ITSDK_ERROR_FAILED;
}


/**
 * clear the error log history
 */
itsdk_error_ret_e itsdk_error_clear() {
	itsdk_error_head_t h;
	if ( _itsdk_error_readHeader(&h) == ITSDK_ERROR_FAILED ) return ITSDK_ERROR_FAILED;
	h.readPt = h.writePt;
	_itsdk_error_writeHeader(&h);
	return ITSDK_ERROR_SUCCESS;
}

/**
 * Get the size of the error blocks
 */
itsdk_error_ret_e itsdk_error_getSize(uint32_t * size) {
	*size=sizeof(itsdk_error_t);
	return ITSDK_ERROR_SUCCESS;
}

// =================================================================================
// Console options
// =================================================================================

#if ITSDK_WITH_CONSOLE == __ENABLE
static itsdk_console_return_e _itsdk_error_consolePriv(char * buffer, uint8_t sz) {
	if ( sz == 1 ) {
	  switch(buffer[0]){
		case '?':
			// help
			_itsdk_console_printf("--- ErrorMng\r\n");
			_itsdk_console_printf("e          : print errors log\r\n");
			_itsdk_console_printf("E          : Clear the error logs\r\n");
		  return ITSDK_CONSOLE_SUCCES;
		  break;
		case 'e':
			{
				uint16_t blockId = ITSDK_ERROR_FIRSTBLOCK;
				itsdk_error_entry_t e;
				while ( itsdk_error_get(&blockId,&e) == ITSDK_ERROR_SUCCESS ) {
					char l;
					switch (e.error & ITSDK_ERROR_LEVEL_MASK) {
					default:
					case ITSDK_ERROR_LEVEL_INFO:  l = 'I'; break;
					case ITSDK_ERROR_LEVEL_WARN:  l = 'W'; break;
					case ITSDK_ERROR_LEVEL_ERROR: l = 'E'; break;
					case ITSDK_ERROR_LEVEL_FATAL: l = 'F'; break;
					}
					_itsdk_console_printf("%c %015d : 0x%08X ( %c 0x%03X / 0x%04X )\r\n",
						l,
						e.timeS,
						e.error,
						(((e.error & ITSDK_ERROR_TYPE_APP) > 0) ? 'A' : 'S'),
						(e.error & ITSDK_ERROR_ERROR_MASK) >> ITSDK_ERROR_ERROR_SHIFT,
						( ((e.error & ITSDK_ERROR_WITH_VALUE) > 0)? (e.error & ITSDK_ERROR_VALUE_MASK) >> ITSDK_ERROR_VALUE_SHIFT:0)
					);
				}
				_itsdk_console_printf("OK\r\n");
			}
  		    return ITSDK_CONSOLE_SUCCES;
			break;
		case 'E':
			itsdk_error_clear();
			_itsdk_console_printf("OK\r\n");
  		    return ITSDK_CONSOLE_SUCCES;
			break;
		default:
			break;
	  }
	} //Sz == 1
  return ITSDK_CONSOLE_NOTFOUND;
}
#endif


#else //ITSDK_WITH_ERROR_RPT
itsdk_error_ret_e itsdk_error_noreport(uint32_t error) {
	if ( (error & ITSDK_ERROR_LEVEL_FATAL ) == ITSDK_ERROR_LEVEL_FATAL ) while(1);
	return ITSDK_ERROR_SUCCESS;
}
#endif
