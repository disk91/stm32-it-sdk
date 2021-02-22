/* ==========================================================
 * securestore.c - encrypted eeprom storage
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 9 f�vr. 2019
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
 * The secured store is a group of 16B entries encrypted with
 * AES. Its is used to store critical elements like encryption
 * keys.
 * The data are secured with AES-ECB : the protection is not optimal
 * but access time is efficient and memory footprint is low.
 * The master key needs to be stored also in the eeprom creating another
 * weak point. The master key is protected by a static key composed from
 * different element : deviceId, static key and static customizable code.
 * ---
 * This protection is not perfect against physical attack on the device but
 * it makes the attack more complex and reduce the capacity to create a generic
 * attack.
 * ---
 *
 *
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_WITH_SECURESTORE == __ENABLE

#include <it_sdk/eeprom/securestore.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/eeprom/eeprom.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/encrypt/encrypt.h>
#include <string.h>

#if ITSDK_WITH_CONSOLE == __ENABLE
#include <it_sdk/console/console.h>
#endif

#if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
#include <it_sdk/lorawan/lorawan.h>
#endif

#if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
#include <it_sdk/sigfox/sigfox.h>
#endif

/**
 * Compute the offset of a block in the EEPROM Memory for a given
 * Entry ID.
 *
 */
static itsdk_secStoreReturn_e _itsdk_secstore_getOffset(uint32_t * offset, uint8_t * blockId, itsdk_secStoreBlocks_e block ) {
	itsdk_secStoreBlocks_t * fakeStore = 0;
	uint32_t _offset;
	switch (block) {
	case ITSDK_SS_CONSOLEKEY:
		_offset = (uint32_t)&fakeStore->consolePasswd;
		break;
	case ITSDK_SS_SIGFOXKEY:
	  #if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->sigfoxKey;
	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_ABP_NETIDDEVID:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.abp.netID_devID;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_ABP_NETKEYF:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.abp.netkeyf;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_ABP_NETKEYS:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.abp.netkeys;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_ABP_NETSKEY:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.abp.netSkey;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_ABP_APPSKEY:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.abp.appSkey;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.otaa.devEUI_appAUI;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_OTAA_APPKEY:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.otaa.appKey;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_LORA_OTAA_NWKKEY:
      #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
		_offset = (uint32_t)&fakeStore->lorawan.otaa.nwkKey;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_AES_MASTERK:
      #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 ) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && (( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0) )
		_offset = (uint32_t)&fakeStore->aesMasterKey;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_AES_SHARED_NONCE_SPECKKEY:
      #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0 ))
		_offset = (uint32_t)&fakeStore->encryptSharedNonceSpeck;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER0:
	  #if ITSDK_SECSTORE_USRBLOCK >= 1
		_offset = (uint32_t)&fakeStore->user[0];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER1:
	  #if ITSDK_SECSTORE_USRBLOCK >= 2
		_offset = (uint32_t)&fakeStore->user[1];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER2:
	  #if ITSDK_SECSTORE_USRBLOCK >= 3
		_offset = (uint32_t)&fakeStore->user[2];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER3:
	  #if ITSDK_SECSTORE_USRBLOCK >= 4
		_offset = (uint32_t)&fakeStore->user[3];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER4:
	  #if ITSDK_SECSTORE_USRBLOCK >= 5
		_offset = (uint32_t)&fakeStore->user[4];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER5:
	  #if ITSDK_SECSTORE_USRBLOCK >= 6
		_offset = (uint32_t)&fakeStore->user[5];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER6:
	  #if ITSDK_SECSTORE_USRBLOCK >= 7
		_offset = (uint32_t)&fakeStore->user[6];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER7:
	  #if ITSDK_SECSTORE_USRBLOCK >= 8
		_offset = (uint32_t)&fakeStore->user[7];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	default:
		return SS_FAILED_NOTEXISTING;

	}
	*blockId = (_offset/16);
	_offset+=sizeof(itsdk_secStoreHead_t);
	*offset=_offset;
	return SS_SUCCESS;
}


/**
 * Evaluate the number of block entries regarding the configuration
 * The size is a static value.
 */
static itsdk_secStoreReturn_e _itsdk_secstore_getEntries(uint8_t * entries) {
	uint8_t _entries=1;	// at lease the console
   #if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
	_entries++;
   #endif
   #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
   _entries+=5;
   #endif
   #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || (defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0))
	_entries+=2;
   #endif
	_entries+=ITSDK_SECSTORE_USRBLOCK;
	*entries=_entries;
	return SS_SUCCESS;
}

static itsdk_secStoreReturn_e _itsdk_secstore_controlHeader(itsdk_secStoreHead_t * _head) {
	// Read the header
	_eeprom_read(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET, (void *) _head, sizeof(itsdk_secStoreHead_t));
	// Control the header validity
	if ( _head->magic1 != ITSDK_SECSTORE_EEPROM_MAGIC ) return SS_FAILED_NOTINITIALIZED;
	uint8_t _count;
	_itsdk_secstore_getEntries(&_count);
	if ( _head->blockCount != _count) return SS_FAILED_NOTINITIALIZED;
	return SS_SUCCESS;
}


/**
 * Return the store size in byte.
 * This function is use to determine the configuration starting address => after the secureStore
 */
itsdk_secStoreReturn_e itsdk_secstore_getStoreSize(uint32_t * sz) {
	*sz=sizeof(itsdk_secStoreHead_t)+sizeof(itsdk_secStoreBlocks_t);
	return SS_SUCCESS;
}


// ============================================================================================================
// ACCESS THE STORE
// ============================================================================================================

/**
 * This function transforms the dynamicKey into a masterKey.
 * I recommend to override this function in your code to prevent
 * from a easy decoding of the secureStore. By changing this function
 * you force the attacker to have a specific reverse engineering for your
 * product instead using a generic one working for most of the device using this SDK.
 * For this reason the transformation executed here is going to be simple.
 * Params:
 *   dynamicKey is 12B long
 *   masterKey is 16B long
 */
__weak void itsdk_secstore_generateMasterKey(uint8_t * dynamicKey,uint8_t * masterKey) {

	// we are creating a 16B array from the elements available
	uint8_t _devId[4];
	itsdk_getUniqId(_devId,4);

	for ( int i = 0 ; i < 16 ; i++ ) {
		if ( i < 2 ) {											// 0, 1
			masterKey[i] = _devId[i];
			masterKey[i] ^= dynamicKey[i];
			masterKey[i] += (ITSDK_PROTECT_KEY >> 9) & 0xFF;
		} else if ( i < 14 ) {									// 2 .. 13
			masterKey[i] = dynamicKey[i-2];
			masterKey[i] ^= _devId[i & 3];
			masterKey[i] ^= (ITSDK_PROTECT_KEY >> 13) & 0xFF;
			masterKey[i] ^= masterKey[i-1];
		} else {												 // 14 .. 15
			masterKey[i] = _devId[3-(15-i)];
			masterKey[i] ^= dynamicKey[i-5];
			masterKey[i] -= (ITSDK_PROTECT_KEY >> 6) & 0xFF;
		}
	}
	itsdk_encrypt_cifferKey(masterKey,16);
}

/**
 * Read the given block and returns the decrypted value into the buffer
 */
itsdk_secStoreReturn_e itsdk_secstore_readBlock(itsdk_secStoreBlocks_e blockType, uint8_t * buffer) {
	itsdk_secStoreHead_t	_head;

	// Control Header validity
	if ( _itsdk_secstore_controlHeader(&_head) != SS_SUCCESS ) return SS_FAILED_NOTINITIALIZED;

	// Control the blockId validity
	uint32_t _offset = 0;
	uint8_t  _id = 0;
	if ( _itsdk_secstore_getOffset(&_offset,&_id, blockType) != SS_SUCCESS ) return SS_FAILED_NOTEXISTING;

	// Control the blockId have been initialized
	if ( (_head.blockUsed & ( 1 << _id )) == 0 ) return SS_FAILED_NOTSET;

	// Read block
	_eeprom_read(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET+_offset, (void *) buffer, ITSDK_SECSTORE_BLOCKSZ);

	// Generate the Master key
	uint8_t masterKey[16];
	itsdk_secstore_generateMasterKey(_head.dynamicKey,masterKey);

	// Decode with AES-128
	itsdk_aes_ecb_decrypt_128B(buffer,buffer,ITSDK_SECSTORE_BLOCKSZ,masterKey);

	return SS_SUCCESS;
}

/**
 * Encrypt and Write the given block into the store
 */
itsdk_secStoreReturn_e itsdk_secstore_writeBlock(itsdk_secStoreBlocks_e blockType, uint8_t * buffer) {
	itsdk_secStoreHead_t	_head;

	// Control header validity
	if ( _itsdk_secstore_controlHeader(&_head) != SS_SUCCESS ) return SS_FAILED_NOTINITIALIZED;

	// Control the blockId validity
	uint32_t _offset = 0;
	uint8_t  _id = 0;
	if ( _itsdk_secstore_getOffset(&_offset,&_id, blockType) != SS_SUCCESS ) return SS_FAILED_NOTEXISTING;

	// Generate the Master key
	uint8_t masterKey[16];
	itsdk_secstore_generateMasterKey(_head.dynamicKey,masterKey);

	// Encode with AES-128
	itsdk_aes_ecb_encrypt_128B(buffer,buffer,ITSDK_SECSTORE_BLOCKSZ,masterKey);

	// Write block
	_eeprom_write(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET+_offset, (void *) buffer, ITSDK_SECSTORE_BLOCKSZ);

	// Update the header
	if ( (_head.blockUsed & ( 1 << _id )) == 0 ) {
		_head.blockUsed |= ( 1 << _id );
		_eeprom_write(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET, (void *) &_head, sizeof(itsdk_secStoreHead_t));
	}

	return SS_SUCCESS;
}



/**
 * Init the Secure Store - create the store structure with the default values
 */
itsdk_secStoreReturn_e itsdk_secstore_init() {
	// Create the header
	itsdk_secStoreHead_t	_head;
	_head.magic1 = ITSDK_SECSTORE_EEPROM_MAGIC;
	uint8_t	count;
	_itsdk_secstore_getEntries(&count);
	_head.blockCount=count;
	_head.blockUsed = 0x1;
	uint8_t _buff[12] = ITSDK_SECSTORE_DEFKEY;
	memcpy(_head.dynamicKey,_buff,12);

	// Store it
	_eeprom_write(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET, (void *) &_head, sizeof(itsdk_secStoreHead_t));

	// Init the console login
	uint8_t _buff2[16] = ITSDK_SECSTORE_CONSOLEKEY;
	itsdk_secstore_writeBlock(ITSDK_SS_CONSOLEKEY,_buff2);
	bzero(_buff2,16);

	return SS_SUCCESS;
}

/**
 * Return SS_SUCCESS when the secure Store is already initialized
 */
itsdk_secStoreReturn_e itsdk_secstore_isInit() {
	itsdk_secStoreHead_t	_head;
	// Control header validity
	return _itsdk_secstore_controlHeader(&_head);
}


// ===========================================================================================
// CONSOLE EXTENSION
// ===========================================================================================

#if ITSDK_WITH_CONSOLE == __ENABLE

#define __console_print_hex(b,off,sz) {												\
								   	    for ( int __i = off; __i<off+sz ; __i++ ) {	\
										     _itsdk_console_printf("%02X",b[__i]);	\
									    }											\
									    _itsdk_console_printf("\r\n");				\
								      }

/**
 * Write a block with a specified MasterKey
 */
static itsdk_secStoreReturn_e _itsdk_secstore_writeBlockKey(itsdk_secStoreBlocks_e blockType, uint8_t * buffer, uint8_t * masterKey) {

	// Control the blockId validity
	uint32_t _offset = 0;
	uint8_t  _id = 0;
	if ( _itsdk_secstore_getOffset(&_offset,&_id, blockType) != SS_SUCCESS ) return SS_FAILED_NOTEXISTING;

	// Encode with AES-128
	itsdk_aes_ecb_encrypt_128B(buffer,buffer,ITSDK_SECSTORE_BLOCKSZ,masterKey);

	// Write block
	_eeprom_write(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET+_offset, (void *) buffer, ITSDK_SECSTORE_BLOCKSZ);

	return SS_SUCCESS;
}


/**
 * Change the dyn key and update all the encrypted elements
 */
static itsdk_console_return_e _itsk_secstore_rekey(uint8_t * newKey){

	itsdk_secStoreHead_t	_head;
	uint8_t _b[ITSDK_SECSTORE_BLOCKSZ];

	// Control Header validity & load previous header
	if ( _itsdk_secstore_controlHeader(&_head) != SS_SUCCESS ) {
		_itsdk_console_printf("KO\r\n");
		return ITSDK_CONSOLE_FAILED;
	}

	// Generate the Master key
	uint8_t masterKey[16];
	itsdk_secstore_generateMasterKey(newKey,masterKey);

	if ( itsdk_secstore_readBlock(ITSDK_SS_CONSOLEKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_CONSOLEKEY,_b,masterKey);
	}
#if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
	if ( itsdk_secstore_readBlock(ITSDK_SS_SIGFOXKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_SIGFOXKEY,_b,masterKey);
	}
#endif
#if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
	// we have more ABP in the UNION when Staticly compiled
	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_NETIDDEVID, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_ABP_NETIDDEVID,_b,masterKey);
	}

	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_NETKEYF, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_ABP_NETKEYF,_b,masterKey);
	}

	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_NETKEYS, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_ABP_NETKEYS,_b,masterKey);
	}

	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_NETSKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_ABP_NETSKEY,_b,masterKey);
	}

	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_APPSKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_ABP_APPSKEY,_b,masterKey);
	}
	#if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC)
	// in this case the secure store have two different parts for storing ABP & OTAA, otherwise one fit all
	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI,_b,masterKey);
	}

	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_APPKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_OTAA_APPKEY,_b,masterKey);
	}

	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_NWKKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_LORA_OTAA_NWKKEY,_b,masterKey);
	}
	#endif

	// ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI
	// ITSDK_SS_LORA_OTAA_APPKEY
	// ITSDK_SS_LORA_OTAA_NWKKEY
#endif
#if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || (defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0))
	if ( itsdk_secstore_readBlock(ITSDK_SS_AES_MASTERK, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_AES_MASTERK,_b,masterKey);
	}
	if ( itsdk_secstore_readBlock(ITSDK_SS_AES_SHARED_NONCE_SPECKKEY, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_AES_SHARED_NONCE_SPECKKEY,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 1
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER0, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER0,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 2
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER1, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER1,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 3
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER2, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER2,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 4
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER3, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER3,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 5
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER4, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER4,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 6
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER5, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER5,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 7
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER6, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER6,_b,masterKey);
	}
#endif
#if ITSDK_SECSTORE_USRBLOCK >= 8
	if ( itsdk_secstore_readBlock(ITSDK_SS_USER7, _b) != SS_FAILED_NOTSET ) {
		_itsdk_secstore_writeBlockKey(ITSDK_SS_USER7,_b,masterKey);
	}
#endif

	// Write Header
	for ( int i = 0 ; i < 12 ; i++) {
		_head.dynamicKey[i] = newKey[i];
	}
	_eeprom_write(ITDT_EEPROM_BANK0, ITSDK_SECSTORE_EEPROM_OFFSET, (void *) &_head, sizeof(itsdk_secStoreHead_t));
	_itsdk_console_printf("OK\r\n");
	return ITSDK_CONSOLE_SUCCES;
}

/**
 * convert and verify a char * hex string into a uint8_t array
 * verify : size of the string regarding the sz
 * verify : format of the char to match Hex number
 */
static bool __checkAndConvert(char * str,uint8_t start,uint8_t stop,uint8_t sz,uint8_t * buf) {
	if ( (stop - start) < 2*sz ) return false;
	int k = 0;
	for ( int i = start ; i < stop ; i+=2 ) {
		if ( itdt_isHexChar(str[i],false) && itdt_isHexChar(str[i+1],false) ) {
			buf[k] = itdt_convertHexChar2Int(&str[i]);
			k++;
		} else return false;
	}
	return true;
}

static itsdk_console_return_e __updateField(char * buffer, uint8_t sz, uint8_t *b, itsdk_secStoreBlocks_e type) {
	if ( __checkAndConvert(buffer,5,sz,16,b) ) {
		if ( type == ITSDK_SS_SIGFOXKEY ) {
		   itsdk_encrypt_cifferKey(b,16);
		}
		if ( itsdk_secstore_writeBlock(type, b) == SS_SUCCESS ) {
			_itsdk_console_printf("OK\r\n");
			return ITSDK_CONSOLE_SUCCES;
		} else {
			_itsdk_console_printf("KO\r\n");
			return ITSDK_CONSOLE_FAILED;
		}
	} else {
		_itsdk_console_printf("KO\r\n");
		return ITSDK_CONSOLE_FAILED;
	}
}

static itsdk_console_return_e __updateField2(char * buffer, uint8_t sz, uint8_t *b, itsdk_secStoreBlocks_e type, uint8_t offset,uint8_t size) {
	if ( __checkAndConvert(buffer,5,sz,size,b) ) {
		uint8_t _b[ITSDK_SECSTORE_BLOCKSZ];
		if ( itsdk_secstore_readBlock(type, _b) != SS_SUCCESS ) {
			_itsdk_console_printf("KO\r\n");
			return ITSDK_CONSOLE_FAILED;
		}
		for ( int i = 0 ; i < size ; i++) {
			_b[i+offset] = b[i];
		}
		if ( itsdk_secstore_writeBlock(type, _b) == SS_SUCCESS ) {
			_itsdk_console_printf("OK\r\n");
			return ITSDK_CONSOLE_SUCCES;
		} else {
			_itsdk_console_printf("KO\r\n");
			return ITSDK_CONSOLE_FAILED;
		}
	} else {
		_itsdk_console_printf("KO\r\n");
		return ITSDK_CONSOLE_FAILED;
	}
}

/**
 * Extends the console function
 */
static itsdk_console_return_e _itsdk_secStore_consolePriv(char * buffer, uint8_t sz) {
	if ( sz == 1 ) {
		switch(buffer[0]){
		case '?':
			// help
			_itsdk_console_printf("--- SecureStore\r\n");
			_itsdk_console_printf("ss:R       : restore all SS to factory default\r\n");
			_itsdk_console_printf("SS:0:xxxx  : change the secure store dyn Key (12B)\r\n");
			_itsdk_console_printf("SS:1:xxxx  : change the console password (max 15 char)\r\n");
		 #if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
			_itsdk_console_printf("ss:S       : Sigfox key restore factory setting\r\n");
			_itsdk_console_printf("SS:2:xxxx  : change the Sigfox key (16B hex)\r\n");
		 #endif
		 #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || (defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0))
			_itsdk_console_printf("ss:Y       : Encryption restore factory setting\r\n");
		 #endif
		 #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
			_itsdk_console_printf("ss:Z       : LoRa restore factory setting\r\n");
		   #if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_STATIC && ITSDK_LORAWAN_ACTIVATION == __LORAWAN_ABP) || ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC
			_itsdk_console_printf("ss:3       : LoRa ABP print NetworkID\r\n");
			_itsdk_console_printf("SS:3:xxxx  : LoRa ABP change NetworkID (8B hex)\r\n");
			_itsdk_console_printf("ss:4       : LoRa ABP print DevID\r\n");
			_itsdk_console_printf("SS:4:xxxx  : LoRa ABP change DevID (8B hex)\r\n");
			_itsdk_console_printf("SS:5:xxxx  : LoRa ABP change Forwarding Network K (16B hex)\r\n");
			_itsdk_console_printf("SS:6:xxxx  : LoRa ABP change Serving Network K (16B hex)\r\n");
			_itsdk_console_printf("SS:7:xxxx  : LoRa ABP change Session Network K (16B hex)\r\n");
			_itsdk_console_printf("SS:8:xxxx  : LoRa ABP change Session Application K (16B hex)\r\n");
		   #endif
		   #if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_STATIC && ITSDK_LORAWAN_ACTIVATION == __LORAWAN_OTAA )  || ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC
			_itsdk_console_printf("ss:9       : LoRa OTAA print DevEUI (8B hex)\r\n");
			_itsdk_console_printf("SS:9:xxxx  : LoRa OTAA change DevEUI (8B hex)\r\n");
			_itsdk_console_printf("ss:A       : LoRa OTAA print AppEUI (8B hex)\r\n");
			_itsdk_console_printf("SS:A:xxxx  : LoRa OTAA change AppEUI (8B hex)\r\n");
			_itsdk_console_printf("SS:B:xxxx  : LoRa OTAA change AppKey (16B hex)\r\n");
			_itsdk_console_printf("SS:C:xxxx  : LoRa OTAA change NwkKey (16B hex)\r\n");
			_itsdk_console_printf("SS:D:xxxx  : LoRa OTAA change Nwk+App (16B hex)\r\n");
           #endif
		  #endif
		  #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 ) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && (( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0) )
			_itsdk_console_printf("SS:E:xxxx  : Encrypt change AES Master Key (16B hex)\r\n");
		  #endif
		  #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0 ))
			_itsdk_console_printf("SS:F:xxxx  : Encrypt change Shared Key (4B hex)\r\n");
			_itsdk_console_printf("SS:G:xxxx  : Encrypt change Nonce (1B hex)\r\n");
			_itsdk_console_printf("SS:H:xxxx  : Encrypt change Speck Key (8B hex)\r\n");
		  #endif
		  #if ITSDK_SECSTORE_USRBLOCK >= 1
			_itsdk_console_printf("SS:I:xxxx  : User change key 0 (16B hex)\r\n");
		  #endif
 		  #if ITSDK_SECSTORE_USRBLOCK >= 2
			_itsdk_console_printf("ss:J       : User print key 1 (16B)\r\n");
			_itsdk_console_printf("SS:J:xxxx  : User change key 1 (16B hex)\r\n");
		  #endif
		  #if ITSDK_SECSTORE_USRBLOCK >= 3
			_itsdk_console_printf("SS:K:xxxx  : User change key 2 (16B hex)\r\n");
		  #endif
 	 	  #if ITSDK_SECSTORE_USRBLOCK >= 4
			_itsdk_console_printf("SS:L:xxxx  : User change key 3 (16B hex)\r\n");
		  #endif
 		  #if ITSDK_SECSTORE_USRBLOCK >= 5
			_itsdk_console_printf("SS:M:xxxx  : User change key 4 (16B hex)\r\n");
		  #endif
		  #if ITSDK_SECSTORE_USRBLOCK >= 6
			_itsdk_console_printf("SS:N:xxxx  : User change key 5 (16B hex)\r\n");
		  #endif
		  #if ITSDK_SECSTORE_USRBLOCK >= 7
			_itsdk_console_printf("SS:O:xxxx  : User change key 6 (16B hex)\r\n");
		  #endif
		  #if ITSDK_SECSTORE_USRBLOCK >= 8
			_itsdk_console_printf("SS:P:xxxx  : User change key 7 (16B hex)\r\n");
		  #endif
		  return ITSDK_CONSOLE_SUCCES;
		  break;
		default:
			break;
		}
	} else if ( sz >= 4 ) {
		uint8_t b[ITSDK_SECSTORE_BLOCKSZ];
		// READ CASE
		if ( buffer[0] == 's' && buffer[1] == 's' && buffer[2] == ':' ) {
			switch(buffer[3]) {
			case 'R':
				// all config factory default
				{
					uint8_t ret = 0;
					#if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
					  if ( itsdk_sigfox_resetFactoryDefaults(true) != SIGFOX_INIT_SUCESS ) ret=1;
					#endif
					#if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || (defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0))
					  if ( itsdk_encrypt_resetFactoryDefaults(BOOL_TRUE) != ENCRYPT_RETURN_SUCESS ) ret =1;
					#endif
					#if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
					  if ( itsdk_lorawan_resetFactoryDefaults(true) != LORAWAN_RETURN_SUCESS ) ret=1;
					#endif
					if ( ret == 0 ) {
					  _itsdk_console_printf("OK\r\n");
					  return ITSDK_CONSOLE_SUCCES;
				    } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
					}
				}
				break;

			#if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
			case 'S':
				if ( itsdk_sigfox_resetFactoryDefaults(true) == SIGFOX_INIT_SUCESS ) {
					  _itsdk_console_printf("OK\r\n");
					  return ITSDK_CONSOLE_SUCCES;
   			    } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				}
				break;
			 #endif
			 #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || (defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0))
			  case 'Y':
				  if ( itsdk_encrypt_resetFactoryDefaults(BOOL_TRUE) == ENCRYPT_RETURN_SUCESS ) {
					  _itsdk_console_printf("OK\r\n");
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
				break;
			 #endif
			 #if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
			  case 'Z':
				  if ( itsdk_lorawan_resetFactoryDefaults(true) == LORAWAN_RETURN_SUCESS ) {
					  _itsdk_console_printf("OK\r\n");
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
              #if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_STATIC && ITSDK_LORAWAN_ACTIVATION == __LORAWAN_ABP) || ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC
			  case '3':
				  // ITSDK_SS_LORA_ABP_NETIDDEVID
				  if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_NETIDDEVID, b) == SS_SUCCESS ) {
					  __console_print_hex(b,0,8);
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
			  case '4':
				  // ITSDK_SS_LORA_ABP_NETIDDEVID
				  if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_ABP_NETIDDEVID, b) == SS_SUCCESS ) {
					  __console_print_hex(b,8,8);
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
              #endif
              #if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_STATIC && ITSDK_LORAWAN_ACTIVATION == __LORAWAN_OTAA )  || ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC
			  case '9':
				  // ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI
				  if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, b) == SS_SUCCESS ) {
					  __console_print_hex(b,0,8);
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
			  case 'A':
			  case 'a':
				  // ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI
				  if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, b) == SS_SUCCESS ) {
					  __console_print_hex(b,8,8);
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
				  return ITSDK_CONSOLE_SUCCES;
			  #endif
			 #endif
			 #if ITSDK_SECSTORE_USRBLOCK >= 2
			  case 'J':
			  case 'j':
				  // ITSDK_SS_USER1
				  if ( itsdk_secstore_readBlock(ITSDK_SS_USER1, b) == SS_SUCCESS ) {
					  __console_print_hex(b,0,16);
					  return ITSDK_CONSOLE_SUCCES;
				  } else {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				  }
				  return ITSDK_CONSOLE_SUCCES;
			  #endif
			  default:
				  break;
			}
		}
		// WRITE CASE
		if ( buffer[0] == 'S' && buffer[1] == 'S' && buffer[2] == ':' && buffer[4] == ':' ) {
			int ssz = sz-5;
			switch(buffer[3]) {
			case '0':
				// DYNKEY
				if ( __checkAndConvert(buffer,5,sz,12,b) ) {
					return _itsk_secstore_rekey(b);
				} else {
					_itsdk_console_printf("KO\r\n");
					return ITSDK_CONSOLE_FAILED;
				}
			case '1':
				// ITSDK_SS_CONSOLEKEY
				if ( ssz > 15 ) {
					  _itsdk_console_printf("KO\r\n");
					  return ITSDK_CONSOLE_FAILED;
				}
				for ( int i = 0 ; i < ssz ; i++) {
					b[i] = buffer[i+5];
				}
				for ( int i = ssz ; i < ITSDK_SECSTORE_BLOCKSZ ; i++) {
					b[i] = 0;
				}
				if ( itsdk_secstore_writeBlock(ITSDK_SS_CONSOLEKEY, b) == SS_SUCCESS ) {
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
				} else {
					_itsdk_console_printf("KO\r\n");
					return ITSDK_CONSOLE_FAILED;
				}
				return ITSDK_CONSOLE_SUCCES;
	#if defined(ITSDK_WITH_SIGFOX_LIB) && ITSDK_WITH_SIGFOX_LIB == __ENABLE
			case '2':
				// ITSDK_SS_SIGFOXKEY
				return __updateField(buffer, sz, b, ITSDK_SS_SIGFOXKEY);
	#endif
	#if defined(ITSDK_WITH_LORAWAN_LIB) && ITSDK_WITH_LORAWAN_LIB == __ENABLE
         #if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_STATIC && ITSDK_LORAWAN_ACTIVATION == __LORAWAN_ABP) || ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC
			case '3':
				// ITSDK_SS_LORA_ABP_NETIDDEVID
				return __updateField2(buffer,sz,b,ITSDK_SS_LORA_ABP_NETIDDEVID,0,8);
			case '4':
				// ITSDK_SS_LORA_ABP_NETIDDEVID
				return __updateField2(buffer,sz,b,ITSDK_SS_LORA_ABP_NETIDDEVID,8,8);
			case '5':
				// ITSDK_SS_LORA_ABP_NETKEYF
				return __updateField(buffer, sz, b, ITSDK_SS_LORA_ABP_NETKEYF);
			case '6':
				// ITSDK_SS_LORA_ABP_NETKEYS
				return __updateField(buffer, sz, b, ITSDK_SS_LORA_ABP_NETKEYS);
			case '7':
				// ITSDK_SS_LORA_ABP_NETSKEY
				return __updateField(buffer, sz, b, ITSDK_SS_LORA_ABP_NETSKEY);
			case '8':
				// ITSDK_SS_LORA_ABP_APPSKEY
				return __updateField(buffer, sz, b, ITSDK_SS_LORA_ABP_APPSKEY);
		#endif
		#if (ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_STATIC && ITSDK_LORAWAN_ACTIVATION == __LORAWAN_OTAA )  || ITSDK_LORAWAN_ACTTYPE == __LORAWAN_ACTIVATION_DYNAMIC
			case '9':
				// ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI
				return __updateField2(buffer,sz,b,ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI,0,8);
			case 'a':
			case 'A':
				// ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI
				return __updateField2(buffer,sz,b,ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI,8,8);
			case 'b':
			case 'B':
				// ITSDK_SS_LORA_OTAA_APPKEY
				return __updateField(buffer, sz, b, ITSDK_SS_LORA_OTAA_APPKEY);
			case 'c':
			case 'C':
				// ITSDK_SS_LORA_OTAA_NWKKEY
				return __updateField(buffer, sz, b, ITSDK_SS_LORA_OTAA_NWKKEY);
			case 'd':
			case 'D': {
				// ITSDK_SS_LORA_OTAA_APPKEY + ITSDK_SS_LORA_OTAA_NWKKEY
				itsdk_console_return_e ret;
				if ( (ret = __updateField(buffer, sz, b, ITSDK_SS_LORA_OTAA_NWKKEY)) == ITSDK_CONSOLE_SUCCES ) {
					ret = __updateField(buffer, sz, b, ITSDK_SS_LORA_OTAA_APPKEY);
				}
				return ret;
			}
		#endif
	#endif
	#if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 ) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && (( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0) )
			case 'e':
			case 'E':
				// ITSDK_SS_AES_MASTERK
				return __updateField(buffer, sz, b, ITSDK_SS_AES_MASTERK);
	#endif
	#if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0 ))
			case 'f':
			case 'F':
				// ITSDK_SS_AES_SHARED_NONCE_SPECKKEY
				return __updateField2(buffer,sz,b,ITSDK_SS_AES_SHARED_NONCE_SPECKKEY,0,4);
			case 'g':
			case 'G':
				// ITSDK_SS_AES_SHARED_NONCE_SPECKKEY
				return __updateField2(buffer,sz,b,ITSDK_SS_AES_SHARED_NONCE_SPECKKEY,4,1);
			case 'h':
			case 'H':
				// ITSDK_SS_AES_SHARED_NONCE_SPECKKEY
				return __updateField2(buffer,sz,b,ITSDK_SS_AES_SHARED_NONCE_SPECKKEY,8,8);
	#endif
	#if ITSDK_SECSTORE_USRBLOCK >= 1
			case 'i':
			case 'I':
				// ITSDK_SS_USER0
				return __updateField(buffer, sz, b, ITSDK_SS_USER0);
	#endif
	 #if ITSDK_SECSTORE_USRBLOCK >= 2
			case 'j':
			case 'J':
				// ITSDK_SS_USER1
				return __updateField(buffer, sz, b, ITSDK_SS_USER1);
	#endif
	#if ITSDK_SECSTORE_USRBLOCK >= 3
			case 'k':
			case 'K':
				// ITSDK_SS_USER2
				return __updateField(buffer, sz, b, ITSDK_SS_USER2);
	#endif
	  #if ITSDK_SECSTORE_USRBLOCK >= 4
			case 'l':
			case 'L':
				// ITSDK_SS_USER3
				return __updateField(buffer, sz, b, ITSDK_SS_USER3);
	#endif
	#if ITSDK_SECSTORE_USRBLOCK >= 5
			case 'm':
			case 'M':
				// ITSDK_SS_USER4
				return __updateField(buffer, sz, b, ITSDK_SS_USER4);
	#endif
	#if ITSDK_SECSTORE_USRBLOCK >= 6
			case 'n':
			case 'N':
				// ITSDK_SS_USER5
				return __updateField(buffer, sz, b, ITSDK_SS_USER5);
	#endif
	#if ITSDK_SECSTORE_USRBLOCK >= 7
			case 'o':
			case 'O':
				// ITSDK_SS_USER6
				return __updateField(buffer, sz, b, ITSDK_SS_USER6);
	#endif
	#if ITSDK_SECSTORE_USRBLOCK >= 8
			case 'p':
			case 'P':
				// ITSDK_SS_USER7
				return __updateField(buffer, sz, b, ITSDK_SS_USER7);
	#endif
			} // switch
		} // Write case end
  } //Sz > 4
  return ITSDK_CONSOLE_NOTFOUND;
}
static itsdk_console_chain_t __console_secStore;

#endif // ITSDK_WITH_CONSOLE

itsdk_secStoreReturn_e itsdk_secStore_RegisterConsole() {
#if ITSDK_WITH_CONSOLE == __ENABLE
	__console_secStore.console_private = _itsdk_secStore_consolePriv;
	__console_secStore.console_public = NULL;
	__console_secStore.next = NULL;
	itsdk_console_registerCommand(&__console_secStore);
#endif
	return SS_SUCCESS;
}


#endif // ITSDK_WITH_SECURESTORE
