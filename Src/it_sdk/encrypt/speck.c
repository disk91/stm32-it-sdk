/* ==========================================================
 * speck.c - SPECK encryption library
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

#if ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_SPECK ) > 0 || ( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_SPECK ) > 0
#include <it_sdk/itsdk.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/encrypt/encrypt.h>
#include <it_sdk/encrypt/speck/speck.h>

#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>


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
		ITSDK_ERROR_REPORT(ITSDK_ERROR_ENCRYP_INVALID_DATALEN,dataLen);
	}

	if ( dataLen > ITSDK_ENCRYPT_MAX_FRAME_SIZE ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_ENCRYP_DATA_TOOLARGE,dataLen);
	}

	uint64_t _masterKey = itsdk_encrypt_unCifferKey64(masterKey);
	uint8_t __masterKey[8];
	__masterKey[0] = ((_masterKey & 0xFF00000000000000 ) >> 56);
	__masterKey[1] = ((_masterKey & 0x00FF000000000000 ) >> 48);
	__masterKey[2] = ((_masterKey & 0x0000FF0000000000 ) >> 40);
	__masterKey[3] = ((_masterKey & 0x000000FF00000000 ) >> 32);
	__masterKey[4] = ((_masterKey & 0x00000000FF000000 ) >> 24);
	__masterKey[5] = ((_masterKey & 0x0000000000FF0000 ) >> 16);
	__masterKey[6] = ((_masterKey & 0x000000000000FF00 ) >>  8);
	__masterKey[7] = ((_masterKey & 0x00000000000000FF )      );
	_masterKey ^= _masterKey;


	// The SPECK library is about 260b flash & 92B Ram (in stack)
	// encryption init time is 3ms / encryption time is < 1ms for 4B
	memcpy(encryptedData,clearData,dataLen);
	speck32_encrypt(__masterKey, encryptedData, dataLen);
	bzero(__masterKey,8);

//	log_info("enc-speck 2: [ ");
//	for ( int i = 0 ; i < dataLen ; i++ ) log_info("%02X ",encryptedData[i]);
//	log_info("]\r\n");


}



#endif // ITSDK_SIGFOX_ENCRYPTION


