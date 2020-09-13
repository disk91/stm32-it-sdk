/* ==========================================================
 * tools.c - Encryption common tools
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
 * ==========================================================
 */

#include <string.h>
#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/eeprom/securestore.h>
#include <it_sdk/encrypt/encrypt.h>


/**
 * Protect inMemory key with a simple XOR with a hardcoded
 * 32b value. Not good at all but always better than clear
 * text key in memory.
 */
void itsdk_encrypt_cifferKey(uint8_t * key, int len) {

	if ( (len & 3 ) > 0 ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_ENCRYP_INVALID_DATALEN,(uint16_t)len);
	}
	for ( int i = 0 ; i < len ; i+=4 ) {
		key[i]   ^= (ITSDK_PROTECT_KEY & 0xFF000000) >> 24;
		key[i+1] ^= (ITSDK_PROTECT_KEY & 0x00FF0000) >> 16;
		key[i+2] ^= (ITSDK_PROTECT_KEY & 0x0000FF00) >> 8;
		key[i+3] ^= (ITSDK_PROTECT_KEY & 0x000000FF);
	}
}

/**
 * Un protect inMemory key.
 */
void itsdk_encrypt_unCifferKey(uint8_t * key, int len) {
	itsdk_encrypt_cifferKey(key,len);
}


// ==========================================================================================
// Set the encryption keys
// ==========================================================================================


/**
 * Configure the SecureStore with the Static values obtained from config.h
 * When force is false, the secure store will be refreshed only if there is no
 * configuration already setup.
 */
#if ITSDK_WITH_SECURESTORE == __ENABLE && (( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || (defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0)))
itsdk_encrypt_return_t itsdk_encrypt_resetFactoryDefaults(itsdk_bool_e force) {
	uint8_t buffer[16];
	if ( force == BOOL_TRUE || itsdk_secstore_readBlock(ITSDK_SS_AES_MASTERK, buffer) != SS_SUCCESS ) {

		uint64_t masterkey = ITSDK_ENCRYPT_AES_MASTERKEYH;
		for ( int i = 0 ; i < 8 ; i++ ) {
			buffer[i] = (masterkey >> ((64-8)-i*8) ) & 0xFF;
		}
		masterkey = ITSDK_ENCRYPT_AES_MASTERKEYL;
		for ( int i = 0 ; i < 8 ; i++ ) {
			buffer[i+8] = (masterkey >> ((64-8)-i*8) ) & 0xFF;
		}
		masterkey=0;
		itsdk_secstore_writeBlock(ITSDK_SS_AES_MASTERK, buffer);

		bzero(buffer,16);
		buffer[ITSDK_SECSTORE_CRYPT_NONCE_ID] = ITSDK_ENCRYPT_AES_INITALNONCE;
		uint32_t shared = ITSDK_ENCRYPT_AES_SHAREDKEY;
		for ( int i = 0 ; i < 4 ; i++ ) {
			buffer[ITSDK_SECSTORE_CRYPT_SHARED_ID+i] = ( shared >> ((32-8)-i*8) ) & 0xFF;
		}
		shared = 0;
		masterkey = ITSDK_ENCRYPT_SPECKKEY;
		for ( int i = 0 ; i < 8 ; i++ ) {
			buffer[ITSDK_SECSTORE_CRYPT_SPECK_ID+i] = (masterkey >> ((64-8)-i*8) ) & 0xFF;
		}
		masterkey=0;
		itsdk_secstore_writeBlock(ITSDK_SS_AES_SHARED_NONCE_SPECKKEY, buffer);
	}
	bzero(buffer,16);
	return ENCRYPT_RETURN_SUCESS;
}
#else
itsdk_encrypt_return_t itsdk_encrypt_resetFactoryDefaults(itsdk_bool_e force) {
	return ENCRYPT_RETURN_SUCESS;
}
#endif

// ==========================================================================================
// Get the encryption keys
// ==========================================================================================

/**
 * Return default nonce, this function is overloaded in the main program
 * to return a dynamic value
 */
__weak  itsdk_encrypt_return_t itsdk_encrypt_aes_getNonce(uint8_t * nonce) {
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[16];
	if ( itsdk_secstore_readBlock(ITSDK_SS_AES_SHARED_NONCE_SPECKKEY, d) != SS_SUCCESS ) {
		*nonce = ITSDK_ENCRYPT_AES_INITALNONCE;
	} else {
		*nonce = d[ITSDK_SECSTORE_CRYPT_NONCE_ID];
	}
#else
	*nonce = ITSDK_ENCRYPT_AES_INITALNONCE;
#endif
	return ENCRYPT_RETURN_SUCESS;
}

/**
 * Return default sharedKey, this function is overloaded in the main program
 * to return a dynamic value
 */
__weak itsdk_encrypt_return_t itsdk_encrypt_aes_getSharedKey(uint32_t * sharedKey) {
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[16];
	if ( itsdk_secstore_readBlock(ITSDK_SS_AES_SHARED_NONCE_SPECKKEY, d) != SS_SUCCESS ) {
		*sharedKey = ITSDK_ENCRYPT_AES_SHAREDKEY;
	} else {
		*sharedKey = 0;
		for (int i = 0 ; i<4 ; i++) {
			*sharedKey = (*sharedKey)*256 + d[ITSDK_SECSTORE_CRYPT_SHARED_ID+i];
		}
	}
#else
	*sharedKey = ITSDK_ENCRYPT_AES_SHAREDKEY;
#endif
	return ENCRYPT_RETURN_SUCESS;
}


/**
 * Return default masterKey (protected by ITSDK_PROTECT_KEY), this function is overloaded in the main program
 * to return a dynamic value when needed
 */
__weak itsdk_encrypt_return_t itsdk_encrypt_aes_getMasterKey(uint8_t * masterKey) {
	uint8_t tmp[16];
#if ITSDK_WITH_SECURESTORE == __ENABLE
	if ( itsdk_secstore_readBlock(ITSDK_SS_AES_MASTERK, tmp) != SS_SUCCESS ) {
#endif
		uint64_t h = ITSDK_ENCRYPT_AES_MASTERKEYH;
		uint64_t l = ITSDK_ENCRYPT_AES_MASTERKEYL;
		for ( int i = 0 ; i < 8 ; i++) {
			tmp[i] = (h >> ((8-i)*8-8)) & 0xFF;
		}
		for ( int i = 0 ; i < 8 ; i++) {
			tmp[8+i] = (l >> ((8-i)*8-8)) & 0xFF;
		}
#if ITSDK_WITH_SECURESTORE == __ENABLE
	}
#endif
	memcpy(masterKey,tmp,16);
	return ENCRYPT_RETURN_SUCESS;
}

/**
 * Return default speck Key ( protected by ITSDK_PROTECT_KEY), this function is overloaded in the main program
 * ro return a dynamic value when needed
 */
__weak itsdk_encrypt_return_t itsdk_encrypt_speck_getMasterKey(uint64_t * masterKey) {
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[16];
	if ( itsdk_secstore_readBlock(ITSDK_SS_AES_SHARED_NONCE_SPECKKEY, d) != SS_SUCCESS ) {
		*masterKey = ITSDK_ENCRYPT_SPECKKEY;
	} else {
		*masterKey = 0;
		for (int i = 0 ; i<8 ; i++) {
			*masterKey = (*masterKey)*256 + d[ITSDK_SECSTORE_CRYPT_SPECK_ID+i];
		}
	}
#else
	*masterKey = ITSDK_ENCRYPT_SPECKKEY;
#endif
	return ENCRYPT_RETURN_SUCESS;
}


