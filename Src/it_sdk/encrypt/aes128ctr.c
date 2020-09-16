/* ==========================================================
 * aes128ctr.c - AES CTR encrytion library
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
 * Library for AES CTR encryption
 * - The CTR is created with a 128b vector composed the following way
 *   - 16B sequence id (limited to 12 real bits with Sigfox)
 *   - 32B deviceID
 *   - 16B sequence id (limited to 12 real bits with Sigfox)
 *   - 32B deviceID
 *   -  8B Dynamically changeable value received from network
 *   - 24B Shared key (hardcoded in firmware - shared with all devices)
 * - The 128B secret key used for EAS CTR encryption is hardcoded
 *   In the firmware / eventually the Sigfox key can be reused but
 *   this is reducing the security level. This key must be chosen to
 *   be uniq per device.
 * - The Iv is set to 0
 *
 * Rq : the use of the AES128-CBC encryption provided by S2LP stack
 * could save some flash but I was not able yet to determine the IV
 * used for this encryption and the result are not coherent with a
 * standard EAS128 encryption.
 * ==========================================================
 */
#include <string.h>
#include <it_sdk/config.h>

#if ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 || ( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 || ITSDK_WITH_SECURESTORE == __ENABLE
#include <it_sdk/itsdk.h>
#include <it_sdk/encrypt/encrypt.h>
#include <it_sdk/logger/logger.h>

#if false && ITSDK_SIGFOX_LIB == __SIGFOX_S2LP
	#include <drivers/s2lp/sigfox_retriever.h>
	#include <drivers/sigfox/sigfox_api.h>
#else
	#include <it_sdk/encrypt/tiny-AES-c/aes.h>
#endif

#include <it_sdk/time/time.h>


#if ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 || ( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0  || ITSDK_WITH_SIGFOX_LIB == __ENABLE
/**
 * Encrypt a 128B block of Data with the given key
 * The key is protected by the ITSDK_PROTECT_KEY
 */
void itsdk_aes_ctr_encrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint32_t  deviceId,				// 32b device ID
		uint16_t  seqId,				// 16b sequenceId (incremented for each of the frame)
		uint8_t   nonce,				// 8b  value you can update dynamically from backend
		uint32_t  sharedKey,			// 24b hardcoded value (hidden with ITSDK_PROTECT_KEY)
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
) {

	// Compose the 128b ctr element
	uint8_t ctr[16];
	ctr[0] = (seqId & 0xFF00) >> 8;
	ctr[1] = (seqId & 0xFF);
	ctr[2] = (deviceId & 0xFF000000 ) >> 24;
	ctr[3] = (deviceId & 0x00FF0000 ) >> 16;
	ctr[4] = (deviceId & 0x0000FF00 ) >> 8;
	ctr[5] = (deviceId & 0x000000FF );
	ctr[6] = ctr[0];
	ctr[7] = ctr[1];
	ctr[8] = ctr[2];
	ctr[9] = ctr[3];
	ctr[10] = ctr[4];
	ctr[11] = ctr[5];
	ctr[12] = nonce;
	ctr[13] = (((sharedKey ^ ITSDK_PROTECT_KEY) & 0x00FF0000 ) >> 16);
	ctr[14] = (((sharedKey ^ ITSDK_PROTECT_KEY) & 0x0000FF00 ) >> 8);
	ctr[15] = (((sharedKey ^ ITSDK_PROTECT_KEY) & 0x000000FF ));

	uint8_t aesResult[16];
	itsdk_encrypt_unCifferKey(masterKey,16);

//	log_info("ctr : [ ");
//	for ( int i = 0 ; i < 16 ; i++ ) log_info("%02X ",ctr[i]);
//	log_info("]\r\n");
//
//	log_info("masterK : [ ");
//	for ( int i = 0 ; i < 16 ; i++ ) log_info("%02X ",masterKey[i]);
//	log_info("]\r\n");

#if false && ITSDK_SIGFOX_LIB == __SIGFOX_S2LP
	// The STM32 EAS encryption lib seems to work strangely...
	enc_utils_encrypt(aesResult, ctr, 16, masterKey, CREDENTIALS_KEY_IN_ARGUMENT);
//	log_info("enc1 : [ ");
//	for ( int i = 0 ; i < 16 ; i++ ) log_info("%02X ",aesResult[i]);
//	log_info("]\r\n");

#else
	{
		// The AES library is about 1.2Kb flash & 176B Ram (in stack)
		// encryption init time is 0ms / encryption time is 2ms
		struct AES_ctx ctx;
		memcpy(aesResult,ctr,16);
		bzero(ctx.Iv,16);
//		uint8_t Iv[16]= {0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF };
//		memcpy(ctx.Iv,Iv,16);
		tiny_AES_init_ctx(&ctx,masterKey);
		tiny_AES_CBC_encrypt_buffer(&ctx, aesResult, 16);

//		log_info("enc2 : [ ");
//		for ( int i = 0 ; i < 16 ; i++ ) log_info("%02X ",aesResult[i]);
//		log_info("\r\n");

	}
#endif

	itsdk_encrypt_cifferKey(masterKey,16);
	bzero(ctr,16);

	// switch the clearData bits according to the computed value
	for ( int i = 0,k=0 ; i < dataLen ; i++, k=(k+1) & 0xF) {
		encryptedData[i] = clearData[i] ^ aesResult[k];
	}

	bzero(aesResult,16);
}

#endif

#if ITSDK_WITH_SIGFOX_LIB == __ENABLE

/**
 * CBC encryption with Iv = 0
 * clearData and encryptedData ca be the same buffer
 * support multiple block encryption
 */
void itsdk_aes_cbc_encrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted can be higher than 16B
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
) {
	uint8_t aesResult[16];
	itsdk_encrypt_unCifferKey(masterKey,16);
	struct AES_ctx ctx;
	bzero(ctx.Iv,16);
	tiny_AES_init_ctx(&ctx,masterKey);

	for ( int k = 0 ; k < dataLen/16 ; k++ ) {
		memcpy(aesResult,&clearData[16*k],16);
		tiny_AES_CBC_encrypt_buffer(&ctx, aesResult, 16);
		memcpy(&encryptedData[16*k],aesResult,16);
	}
	itsdk_encrypt_cifferKey(masterKey,16);
	bzero(aesResult,16);
	bzero(&ctx,sizeof(struct AES_ctx));
}

#endif


#if ITSDK_WITH_SECURESTORE == __ENABLE


/**
 * ECB encryption with Iv = 0
 * clearData and encryptedData ca be the same buffer
 */
void itsdk_aes_ecb_encrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
) {
	uint8_t aesResult[16];
	itsdk_encrypt_unCifferKey(masterKey,16);
	struct AES_ctx ctx;
	memcpy(aesResult,clearData,16);
	bzero(ctx.Iv,16);
	tiny_AES_init_ctx(&ctx,masterKey);
	tiny_AES_CBC_encrypt_buffer(&ctx, aesResult, 16);
	itsdk_encrypt_cifferKey(masterKey,16);
	memcpy(encryptedData,aesResult,16);
	bzero(aesResult,16);
	bzero(&ctx,sizeof(struct AES_ctx));
}

/**
 * ECB encryption with Iv = 0
 * clearData and encryptedData ca be the same buffer
 * Rq : it use CBC encryption but we just encrypt 1 block... so it is like ECB
 */
void itsdk_aes_ecb_decrypt_128B(
		uint8_t	* clearData,			// Data to be encrypted
		uint8_t * encryptedData,		// Can be the same as clearData
		uint8_t   dataLen,				// Size of data to be encrypted
		uint8_t * masterKey				// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
) {
	uint8_t aesResult[16];
	itsdk_encrypt_unCifferKey(masterKey,16);
	struct AES_ctx ctx;
	memcpy(aesResult,clearData,16);
	bzero(ctx.Iv,16);
	tiny_AES_init_ctx(&ctx,masterKey);
	tiny_AES_CBC_decrypt_buffer(&ctx, aesResult, 16);
	itsdk_encrypt_cifferKey(masterKey,16);
	memcpy(encryptedData,aesResult,16);
	bzero(aesResult,16);
}


#endif




#endif // ITSDK_SIGFOX_ENCRYPTION


