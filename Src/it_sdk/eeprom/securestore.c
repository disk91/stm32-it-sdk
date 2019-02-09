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
#include <it_sdk/eeprom/securestore.h>

#if ITSDK_WITH_SECURESTORE == __ENABLE


/**
 * Compute the offset of a block in the EEPROM Memory for a given
 * Entry ID.
 *
 */
static itsdk_secStoreReturn itsdk_secstore_getOffset(uint32_t * offset, itsdk_secStoreBlocks block ) {
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
      #if ( defined(ITSDK_SIGFOX_ENCRYPTION) && ( ITSDK_SIGFOX_ENCRYPTION > 0 )) || ( defined(ITSDK_LORAWAN_ENCRYPTION) && ( ITSDK_LORAWAN_ENCRYPTION > 0 )) )
		_offset = (uint32_t)&fakeStore->encryptSharedNonceSpeck;
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER0:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 1
		_offset = (uint32_t)&fakeStore->user[0];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER1:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 2
		_offset = (uint32_t)&fakeStore->user[1];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER2:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 3
		_offset = (uint32_t)&fakeStore->user[2];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER3:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 4
		_offset = (uint32_t)&fakeStore->user[3];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER4:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 5
		_offset = (uint32_t)&fakeStore->user[4];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER5:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 6
		_offset = (uint32_t)&fakeStore->user[5];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER6:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 7
		_offset = (uint32_t)&fakeStore->user[6];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	case ITSDK_SS_USER7:
	  #if ITSDK_SECURESTORE_USRBLOCK >= 8
		_offset = (uint32_t)&fakeStore->user[7];
  	  #else
		return SS_FAILED_NOTEXISTING;
	  #endif
		break;
	default:
		return SS_FAILED_NOTEXISTING;

	}
	_offset+=sizeof(itsdk_secureStoreHead_t);
	*offset=_offset;
	return SS_SUCCESS;
}


#endif // ITSDK_WITH_SECURESTORE
