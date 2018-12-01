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
#if ITSDK_WITH_SIGFOX_LIB > 0

#include <it_sdk/itsdk.h>
#include <drivers/s2lp/s2lp.h>
#include <drivers/s2lp/s2lp_spi.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/s2lp/sigfox_helper.h>
#include <it_sdk/logger/logger.h>
#include <string.h>

s2lp_config_t *	_s2lp_sigfox_config;
void itsdk_sigfox_configInit(s2lp_config_t * cnf) {
	_s2lp_sigfox_config = cnf;
}

/**
 * Init the sigfox library according to the configuration
 * passed as parameter.
 * return true when success.
 */
bool s2lp_sigfox_init(s2lp_config_t * conf) {

	switch ( conf->rcz ) {
	case 1:
		SIGFOX_API_open(&(sfx_rc_t)RC1);
		break;
	case 2:
		SIGFOX_API_open(&(sfx_rc_t)RC2);
		// In FCC we can choose the macro channel to use by a 86 bits bitmask
	    //  In this case we use the first 9 macro channels
		sfx_u32 config_words1[3]={1,0,0};
		SIGFOX_API_set_std_config(config_words1,1);
		log_error("RCZ2 implementation is actually not working");
	    _Error_Handler(__FILE__, __LINE__);
		break;
	case 3:
		SIGFOX_API_open(&(sfx_rc_t)RC3C);
		sfx_u32 config_words2[3]=RC3C_CONFIG;
		SIGFOX_API_set_std_config(config_words2,0);
		break;
	case 4:
		SIGFOX_API_open(&(sfx_rc_t)RC4);
		sfx_u32 config_words3[3]={0,0x40000000,0};
		SIGFOX_API_set_std_config(config_words3,1);
		log_error("RCZ4 implementation is actually not working");
	    _Error_Handler(__FILE__, __LINE__);
		break;
	case 5:
		log_error("RCZ5 implementation is actually supported");
	    _Error_Handler(__FILE__, __LINE__);
		break;

	}

	return true;
}



/**
 * Protect the private key in memory
 *  - basic Xor ... better than nothing
 */
void s2lp_sigfox_cifferKey(s2lp_config_t * conf) {
	 uint32_t key = ITSDK_SIGFOX_PROTECT_KEY;
	 uint32_t * pk = (uint32_t *)(&conf->key[0]);
	 for ( int i = 0  ; i < 4 ; i++,pk++ ) *pk ^= key;
}
void s2lp_sigfox_unCifferKey(s2lp_config_t * conf) {
	s2lp_sigfox_cifferKey(conf);
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

/**
 * Get from S2LP the Last RSSI level after frame sync
 */
int16_t s2lp_sigfox_getLastRssiLevel() {

	uint8_t outBuf[1];
	s2lp_spi_readRegisters(
			&ITSDK_S2LP_SPI,
			S2LP_REG_RSSI_LEVEL,
	        1,
			outBuf
	);
	int16_t rssi = S2LP_UNKNOWN_RSSI;
	if ( outBuf[1] != 0xFF ) {
		// the register contains rssi with 0 = -146
		// -12 is not clear but looks like what we have compared to what is returned...
		rssi = (int16_t)outBuf[1] - 146;
		log_info("Rssi raw %d \r\n",(int)rssi);
		rssi -= 12;
	}
	return rssi;

}

#endif // ITSDK_WITH_SIGFOX_LIB test

