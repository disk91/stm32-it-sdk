/* ==========================================================
 * encrypt.h - Encryption headers
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 02 dec. 2018
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

#ifndef IT_SDK_ENCRYPT_H_
#define IT_SDK_ENCRYPT_H_

#include <it_sdk/config.h>

typedef enum {												// Encryption mode are cumulative
	PAYLOAD_ENCRYPT_NONE = __PAYLOAD_ENCRYPT_NONE,			// Clear text payload
	PAYLOAD_ENCRYPT_SIGFOX = __PAYLOAD_ENCRYPT_SIGFOX,		// Sigfox native encryption
	PAYLOAD_ENCRYPT_AESCTR = __PAYLOAD_ENCRYPT_AESCTR,		// Software AES-CTR (like sigfox) encryption
	PAYLOAD_ENCRYPT_SPECK = __PAYLOAD_ENCRYPT_SPECK			// SPECK32 encryption
} itdsk_payload_encrypt_t;

typedef enum {
	ENCRYPT_RETURN_SUCESS = 0,
	ENCRYPT_RETURN_FAILED
}itsdk_encrypt_return_t;

#if defined ITSDK_LORAWAN_ENCRYPTION && ITSDK_LORAWAN_ENCRYPTION > 0
  #define ITSDK_ENCRYPT_MAX_FRAME_SIZE    64				// LoRaWan max frame size (arbitral)
#else
  #define ITSDK_ENCRYPT_MAX_FRAME_SIZE	  12				// Sigfox - max size for a frame to be encrypted (related to buffer size)
#endif

void itsdk_encrypt_cifferKey(uint8_t * key, int len);
void itsdk_encrypt_unCifferKey(uint8_t * key, int len);

itsdk_encrypt_return_t itsdk_encrypt_resetFactoryDefaults(itsdk_bool_e force);
itsdk_encrypt_return_t itsdk_encrypt_aes_getNonce(uint8_t * nonce);
itsdk_encrypt_return_t itsdk_encrypt_aes_getSharedKey(uint32_t * sharedKey);
itsdk_encrypt_return_t itsdk_encrypt_aes_getMasterKey(uint8_t * masterKey);
itsdk_encrypt_return_t itsdk_encrypt_speck_getMasterKey(uint64_t * masterKey);

// uint64_t ciffer/unciffer function
#define itsdk_encrypt_cifferKey64(v) ( \
		(uint64_t)(v) ^ (  ((uint64_t)ITSDK_PROTECT_KEY  | ((uint64_t)ITSDK_PROTECT_KEY << 32) ) ) \
)

#define itsdk_encrypt_unCifferKey64(v) itsdk_encrypt_cifferKey64(v)


void itsdk_aes_ctr_encrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint32_t  deviceId,				// 32b device ID
		uint16_t  seqId,				// 16b sequenceId (incremented for each of the frame)
		uint8_t   nonce,				// 8b  value you can update dynamically from backend
		uint32_t  sharedKey,			// 24b hardcoded value (hidden with ITSDK_PROTECT_KEY)
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
);

void itsdk_speck_encrypt(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint64_t  masterKey				// 64B key used for encryption (hidden with ITSDK_PROTECT_KEY)
);


void itsdk_aes_ecb_encrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
);

void itsdk_aes_ecb_decrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
);


void itsdk_aes_cbc_encrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted can be higher than 16B
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
);

#endif /* IT_SDK_ENCRYPT_H_ */
