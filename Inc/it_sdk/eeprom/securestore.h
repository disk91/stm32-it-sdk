/* ==========================================================
 * securestore.h - secured / encryptage eeprom storage
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 9 févr. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2019 Disk91
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

#ifndef IT_SDK_EEPROM_SECURESTORE_H_
#define IT_SDK_EEPROM_SECURESTORE_H_

#include <it_sdk/config.h>

#if ITSDK_WITH_SECURESTORE == __ENABLE

// ================================================================================
// Function return

enum {
	SS_SUCCESS=0,
	SS_FAILED_NOTEXISTING,

	SS_FAILED
} itsdk_secStoreReturn;



// ================================================================================
// Block ID defined regarding the configuration


enum {
	ITSDK_SS_CONSOLEKEY = 0,
	ITSDK_SS_SIGFOXKEY,
	ITSDK_SS_LORA_ABP_NETIDDEVID,
	ITSDK_SS_LORA_ABP_NETKEYF,
	ITSDK_SS_LORA_ABP_NETKEYS,
	ITSDK_SS_LORA_ABP_NETSKEY,
	ITSDK_SS_LORA_ABP_APPSKEY,
	ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI,
	ITSDK_SS_LORA_OTAA_APPKEY,
	ITSDK_SS_LORA_OTAA_NWKKEY,
	ITSDK_SS_AES_MASTERK,
	ITSDK_SS_AES_SHARED_NONCE_SPECKKEY,
	ITSDK_SS_USER0,
	ITSDK_SS_USER1,
	ITSDK_SS_USER2,
	ITSDK_SS_USER3,
	ITSDK_SS_USER4,
	ITSDK_SS_USER5,
	ITSDK_SS_USER6,
	ITSDK_SS_USER7
} itsdk_secStoreBlocks;

#define ITSDK_SECSTORE_BLOCKSZ		16

typedef struct __itsdk_secureStoreHead_s {

	uint8_t		reserved1:4;				// reserved for later use
	uint8_t		blockCount:4;				// number of block initialized in the structure
	uint16_t	blockUsed;					// block initialization status bit field - bit 0 (low) = block0
	uint8_t		reserved2;					// reserved for later use;
	uint8_t		dynamicKey[12];				// dynamic key use for MasterKey generation
											// First block is starting at this address.

} __attribute__((packed)) itsdk_secureStoreHead_t;



// blocks
typedef struct {
		uint8_t 	consolePasswd[ITSDK_SECSTORE_BLOCKSZ];		// console password
	#if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
		uint8_t		sigfoxKey[ITSDK_SECSTORE_BLOCKSZ];
	#endif
	#if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		union {
			struct {
				uint8_t		netID_devID[ITSDK_SECSTORE_BLOCKSZ];			// NETID - 8B & DEVID - 8B
				uint8_t		netkeyf[ITSDK_SECSTORE_BLOCKSZ];				// Forwarding Network Key
				uint8_t		netkeys[ITSDK_SECSTORE_BLOCKSZ];				// Serving Network Key
				uint8_t		netSkey[ITSDK_SECSTORE_BLOCKSZ];				// Network Session Key
				uint8_t		appSkey[ITSDK_SECSTORE_BLOCKSZ];				// Application Session Key
			} abp;
			struct {
				uint8_t		devEUI_appAUI[ITSDK_SECSTORE_BLOCKSZ];			// DEVEUI - 8B & APPEUI - 8B
				uint8_t		appKey[ITSDK_SECSTORE_BLOCKSZ];				// App Key
				uint8_t		nwkKey[ITSDK_SECSTORE_BLOCKSZ];				// Network Key

			} otaa;
		} lorawan;
	#endif
    #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 ) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && (( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0) )
		uint8_t	aesMasterKey[ITSDK_SECSTORE_BLOCKSZ];						// AES-CTR Key
	#endif
    #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0 )) )
		uint8_t encryptSharedNonceSpeck[ITSDK_SECSTORE_BLOCKSZ];			// AES-CTR / Dynamic element for CTR (Shared 4B), (Nonce 1B) , 4B not used, 8B SPECK KEY
    #endif
	#if ITSDK_SECURESTORE_USRBLOCK > 0
		uint8_t user[ITSDK_SECSTORE_USRBLOCK][ITSDK_SECSTORE_BLOCKSZ];	// User defined blocks
	#endif
} itsdk_secStoreBlocks_t;


#endif // ITSDK_WITH_SECURESTORE

#endif /* IT_SDK_EEPROM_SECURESTORE_H_ */
