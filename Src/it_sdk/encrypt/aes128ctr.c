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
 *   this is reducing the security level. This key must be choosen to
 *   be uniq per device.
 *
 * ==========================================================
 */

#include <it_sdk/config.h>

#if ( ITSDK_SIGFOX_ENCRYPTION & __SIGFOX_ENCRYPT_AESCTR ) > 0
#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>

#if ITSDK_SIGFOX_LIB == __SIGFOX_S2LP

/**
 * Encrypt a 128B block of Data with the given key
 * The key is protected by the ITSDK_SIGFOX_PROTECT_KEY
 */
void itsdk_aes_cbc_encrypt_128B(
		uint8_t	* clearData,			// 128B Data to be encrypted
		uint8_t * encryptedData,		// 128B Can be the same as clearData
		uint8_t * key					// 128B key
) {

	//enc_utils_encrypt(encrypted_data, data_to_encrypt, aes_block_len, key, use_key);
}

#else
  #error "No AES library available for supporting AES-CTR encryption"
#endif




#endif // ITSDK_SIGFOX_ENCRYPTION


