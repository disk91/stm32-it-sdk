/* ==========================================================
 * sigfox_helper.c - Sigfox helper
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 09 nov. 2018
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
#if ITSDK_WITH_SIGFOX_LIB > 0 && ITSDK_SIGFOX_LIB == __SIGFOX_S2LP

#include <it_sdk/itsdk.h>
#include <drivers/s2lp/s2lp.h>
#include <drivers/s2lp/s2lp_spi.h>
#include <drivers/s2lp/st_lib_api.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/s2lp/sigfox_helper.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/encrypt/encrypt.h>
#include <string.h>



/**
 * Protect the private key in memory
 *  - basic Xor ... better than nothing
 */
#if ITSDK_SIGFOX_NVM_SOURCE	== __SFX_NVM_M95640 || ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_HEADERS
void s2lp_sigfox_cifferKey() {
	itsdk_encrypt_cifferKey(s2lp_driver_config.key,16);
}
void s2lp_sigfox_unCifferKey() {
	itsdk_encrypt_unCifferKey(s2lp_driver_config.key,16);
}




/**
 * Some hack of the ST IDRetriever library to extract and set a better protection to
 * the Sigfox secret key stored in clear in the RAM after
 * Basically the strcuture created have the following format
 *
 * struct s_internal {
 *	uint8_t 	unk0[4];			// 0		- 00 00 00 07
 *	uint8_t 	unk1;				// 4		- 0x07
 *	uint8_t		unk2[3];			// 5		- 00 00 00
 *	uint8_t		pac[8];				// 8 		- 36 61 3F 89 44 0B D6 47
 *	uint32_t	deviceID;			// 16		-
 *	uint8_t		unk3[16];			// 20		- 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *	uint8_t		private_key[16];	// 36       - XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX
 *  ...
 * }
 */

uint8_t * __keyPtr = 0;
uint32_t s2lp_sigfox_search4key(int32_t deviceId, uint8_t * pac) {
	LOG_DEBUG_S2LP((">> enc_search4key \r\n"));

	uint8_t * ramPtr = (uint8_t *)0x20000010;	// start at 16 as we search for deviceID

	 int i;
	 for( i = 16 ; i < ITSDK_RAM_SIZE ; i++ ) {
		 // search for deviceId
		 if ( *((uint32_t *)ramPtr) == deviceId ) {
			 // search for pac at -8 bytes
			 int j = 0;
			 while ( j < 8 ) {
				 if ( *(ramPtr-8+j) != pac[j] ) break;
				 j++;
			 }
			 // found !
			 if ( j == 8 ) break;
		 }
		 ramPtr+=4;	// stm32 only support 32b aligned memory access apparently
	 }
	 __keyPtr = ( i < ITSDK_RAM_SIZE )?ramPtr+20:(uint8_t)0;

	 return (uint32_t)__keyPtr;
}

/**
 * Get the sigfox private key from the driver
 */
bool s2lp_sigfox_retreive_key(int32_t deviceId, uint8_t * pac, uint8_t * key) {
	 LOG_DEBUG_S2LP((">> enc_retreive_key \r\n"));
	 if ( __keyPtr == 0 ) s2lp_sigfox_search4key(deviceId, pac);
	 if ( __keyPtr != 0 ) {
		 memcpy(key,__keyPtr,16);
		 return true;
	 }
	 return false;
 }
#endif

/**
 * A valid frame has been received, the last Rssi is a valid Rssi
 */
void s2lp_sigfox_retreive_rssi() {
	s2lp_driver_config.lastReceptionRssi = s2lp_driver_config.lastReadRssi;
}

/**
 * Get from S2LP the Last RSSI level after frame sync
 */
int16_t s2lp_sigfox_getLastRssiLevel() {
	return (int16_t)(s2lp_driver_config.lastReceptionRssi) - 146;
}


uint8_t SE_NVM_set(uint8_t *data_to_write) {
	LOG_DEBUG_S2LP((">> SE_NVM_set\r\n"));
    return MCU_API_set_nv_mem(data_to_write);
}

/**
 * Return the sequence Id
 */
s2lp_sigfox_ret_t s2lp_sigfox_getSeqId( uint16_t * seqId ) {
	LOG_DEBUG_S2LP((">> s2lp_sigfox_getSeqId\r\n"));
	#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	  *seqId = s2lp_driver_config.seqId;
	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM || ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_HEADERS
	  sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE];
	  MCU_API_get_nv_mem(read_data);
      *seqId = (read_data[SFX_NVMEM_SEQ_NUM]+(read_data[SFX_NVMEM_SEQ_NUM+1] << 8)) & 0xFFF;
	#else
	  #error "unsupported configuration"
    #endif
    return S2LP_SIGFOX_ERR_NONE;
}


#endif // ITSDK_WITH_SIGFOX_LIB test

