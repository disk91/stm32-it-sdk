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

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>

/**
 * Protect inMemory key with a simple XOR with a hardcoded
 * 32b value. Not good at all but always better than clear
 * text key in memory.
 */
void itsdk_encrypt_cifferKey(uint8_t * key, int len) {

	if ( (len & 3 ) > 0 ) itsdk_error_handler(__FILE__,__LINE__);
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




