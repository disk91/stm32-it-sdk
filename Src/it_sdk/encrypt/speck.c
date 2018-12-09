/* ==========================================================
 * speck.c - SPECK encrytion library
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 09 dec. 2018
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
 * Library for SPECK encryption
 * - The 64B secret key used for SPECK encryption is hardcoded
 *   In the firmware / eventually the Sigfox key can be reused but
 *   this is reducing the security level. This key must be chosen to
 *   be uniq per device.
 *
 * ==========================================================
 */
#include <string.h>
#include <it_sdk/config.h>

#if ( ITSDK_SIGFOX_ENCRYPTION & __SIGFOX_ENCRYPT_SPECK ) > 0
#include <it_sdk/itsdk.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/encrypt/encrypt.h>
#include <it_sdk/encrypt/speck/speck.h>

#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>

/**
 * Encrypt a block of Data with the given key
 * The key is protected by the ITSDK_PROTECT_KEY
 * Data must be a 4B multiple (you need to add 00 at end of frame is needed)
 */
void itsdk_speck_encrypt(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint64_t  masterKey				// 64B key used for encryption (hidden with ITSDK_PROTECT_KEY)
) {

	if ( (dataLen & 0x3) != 0 ) {
		LOG_ERROR_SIGFOX(("Speck Encryption require a 32B multiple data len\r\n"));
		itsdk_error_handler(__FILE__, __LINE__);
	}

	if ( dataLen > ITSDK_ENCRYPT_MAX_FRAME_SIZE ) {
		LOG_ERROR_SIGFOX(("Speck Encryption only supports frame lower than %dB\r\n",ITSDK_ENCRYPT_MAX_FRAME_SIZE));
		itsdk_error_handler(__FILE__, __LINE__);
	}

	// => ca match pas va faloir revoir le ciffer / unciffer avec des uint64_t
	// => pas pareil que sur des uint8_t
	// @TODO

	uint64_t _masterKey = masterKey;
	itsdk_encrypt_unCifferKey(&_masterKey,8);

	log_info("masterK : [ ");
	for ( int i = 0 ; i < 8 ; i++ ) log_info("%02X ",((uint8_t*)(&_masterKey))[i]);
	log_info("]\r\n");

	// The SPECK library is about 2.4Kb flash & 800B Ram (in stack)
	// Optimization will be possible as the current library supports all SPECK size when we use only one.
	// encryption init time is 3ms / encryption time is < 1ms for 4B

	SimSpk_Cipher speck_cfg;
	if ( Speck_Init(
			&speck_cfg,
			cfg_64_32,				// 32b block / 64b kay
			0,						// mode -  not used in the function
			(void *)&_masterKey,
			NULL,					// Iv - not used in the function
			NULL					// Counter - not used in the funciton
			) ) {
		LOG_ERROR_SIGFOX(("Speck Encryption init failed"));
		itsdk_error_handler(__FILE__, __LINE__);
	}
	uint8_t _encryptedData[ITSDK_ENCRYPT_MAX_FRAME_SIZE];
	for ( int i = 0 ; i < dataLen ; i+= 4) {
		Speck_Encrypt_32(
				speck_cfg.round_limit,
				speck_cfg.key_schedule,
				&clearData[i],
				&_encryptedData[i]
		);
	}
	memcpy(encryptedData,_encryptedData,dataLen);

	log_info("enc-speck : [ ");
	for ( int i = 0 ; i < dataLen ; i++ ) log_info("%02X ",encryptedData[i]);
	log_info("]\r\n");

	_masterKey = 0;

}



#endif // ITSDK_SIGFOX_ENCRYPTION


