/* ==========================================================
 * securestore.c - encrypted eeprom storage
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
   #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0 ) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && (( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR ) > 0) )
	_entries++;
   #endif
   #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0 ))
	_entries++;
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
itsdk_secStoreReturn_e itsdk_secstore_getStoreSize(uint16_t * sz) {
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
		if ( i < 2 ) {
			masterKey[i] = _devId[i];
			masterKey[i] ^= dynamicKey[i];
			masterKey[i] += (ITSDK_PROTECT_KEY >> 9) & 0xFF;
		} else if ( i < 14 ) {
			masterKey[i] = dynamicKey[i];
			masterKey[i] ^= _devId[i & 3];
			masterKey[i] ^= (ITSDK_PROTECT_KEY >> 13) & 0xFF;
			masterKey[i] ^= masterKey[i-1];
		} else {
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


#endif // ITSDK_WITH_SECURESTORE
