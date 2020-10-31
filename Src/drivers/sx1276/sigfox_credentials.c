/* ==========================================================
 * sigfox_credentials.c - Driver for Sigfox SX1276
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 18 may 2019
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
 * Some peaces of that code directly comes from ST Libraries
 * and identified with << COPYRIGHT(c) 2019 STMicroelectronics >>
 *
 * ==========================================================
 */
#include <stdint.h>
#include <string.h>
#include <it_sdk/config.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276)
#include <it_sdk/configSigfox.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/encrypt/encrypt.h>
#include <drivers/sx1276/sigfox_sx1276.h>
#include <drivers/sx1276/sgfx_credentials.h>
#include <drivers/sigfox/se_nvm.h>

/*
#define MANUF_DEVICE_ID_LENGTH     4
#define MANUF_SIGNATURE_LENGTH     16
#define MANUF_VER_LENGTH           1
#define MANUF_SPARE_1              3
#define MANUF_DEVICE_KEY_LENGTH    16
#define MANUF_PAC_LENGTH           8
#define MANUF_SPARE_2   14
#define MANUF_CRC_LENGTH           2


typedef struct manuf_device_info_s
{
    // 16bits block 1
    sfx_u8 dev_id[MANUF_DEVICE_ID_LENGTH];
    sfx_u8 pac[MANUF_PAC_LENGTH];
    sfx_u8 ver[MANUF_VER_LENGTH];
    sfx_u8 spare1[MANUF_SPARE_1];
    // 16bits block 2
    sfx_u8 dev_key[MANUF_DEVICE_KEY_LENGTH];
    // 16bits block 3
    sfx_u8 spare2[MANUF_SPARE_2];
    sfx_u8 crc[MANUF_CRC_LENGTH];
} manuf_device_info_t;
*/
                                      

#define SIGNATURE_LEN 16
static uint8_t session_key[SIGNATURE_LEN]={0};


static const char sgfxSeeLibVersion[]="CRED v1.1";

//

/* Private function prototypes -----------------------------------------------*/
static void CREDENTIALS_get_key (uint8_t* key, sfx_key_type_t KeyType );


/* Public function definition -----------------------------------------------*/


/**
 * Encrypt a bloc with aes-ecb (eq CBC with iv = 0)
 */
sfx_error_t CREDENTIALS_aes_128_cbc_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint8_t blocks)
{
  LOG_DEBUG_SFXSX1276((">> CREDENTIALS_aes_128_cbc_encrypt\r\n"));

  //uint8_t iv[N_BLOCK] = {0x00};
  uint8_t key[AES_KEY_LEN];
  sfx_key_type_t KeyType = SE_NVM_get_key_type();
  CREDENTIALS_get_key ( key, KeyType );
  itsdk_aes_cbc_encrypt_128B(
		  data_to_encrypt,
		  encrypted_data,
		  16*blocks,
		  key
  );
  bzero(key,AES_KEY_LEN);
  return SFX_ERR_NONE;
}

/**
 * Encrypt a data block with the session key previously calculated
 * Session key is already protected in memory, no need to ciffer it
 */
sfx_error_t CREDENTIALS_aes_128_cbc_encrypt_with_session_key(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint8_t blocks) {
  LOG_DEBUG_SFXSX1276((">> CREDENTIALS_aes_128_cbc_encrypt_with_session_key\r\n"));
  itsdk_aes_cbc_encrypt_128B(
		  data_to_encrypt,
		  encrypted_data,
		  16*blocks,
		  session_key
  );

  return SFX_ERR_NONE;
}

/**
 * Generate a session_key from a dataset and the encryption key
 * Then this session key can be used as a key for future encryption
 */
sfx_error_t CREDENTIALS_wrap_session_key( uint8_t* data, uint8_t blocks) {
  LOG_DEBUG_SFXSX1276((">> CREDENTIALS_wrap_session_key\r\n"));

  uint8_t key[AES_KEY_LEN];
  CREDENTIALS_get_key ( key, CREDENTIALS_KEY_PRIVATE);
  itsdk_aes_cbc_encrypt_128B(
		  data,
		  session_key,
		  16*blocks,
		  key
  );
  itsdk_encrypt_cifferKey(session_key,16);		// we keep the session_key more secure in ram
  
  return SFX_ERR_NONE;
}

/**
 * Returns the credential lib version
 */
const char* CREDENTIALS_get_version( void )
{
  LOG_DEBUG_SFXSX1276((">> CREDENTIALS_get_version\r\n"));
  return sgfxSeeLibVersion;
}

/**
 * Returns the device Id
 */
void CREDENTIALS_get_dev_id( uint8_t* dev_id)
{
    LOG_DEBUG_SFXSX1276((">> CREDENTIALS_get_dev_id\r\n"));
    itsdk_sigfox_device_is_t devId;
    itsdk_sigfox_getDeviceId(&devId);
    for (int i = 0 ; i < 4 ; i++) {
    	dev_id[3-i]=(devId >> ((32-8)-8*i)) & 0xFF;
    }
}

/**
 * Returns the initial Pac
 */
void CREDENTIALS_get_initial_pac( uint8_t* pac)
{
    LOG_DEBUG_SFXSX1276((">> CREDENTIALS_get_initial_pac\r\n"));
    itsdk_sigfox_getInitialPac(pac);
}

/**
 * Returns the sigfox encryption status
 */
__attribute__((weak)) sfx_bool CREDENTIALS_get_payload_encryption_flag(void)
{
    LOG_DEBUG_SFXSX1276((">> CREDENTIALS_get_payload_encryption_flag\r\n"));
    #if (defined ITSDK_SIGFOX_ENCRYPTION) && (( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_SIGFOX) > 0)
      return SFX_TRUE;
    #else
      return SFX_FALSE;
    #endif
}

/**
 * Access the Sigfox KEY for internal use
 * The key is stored in memory with ciffer protection
 */
static void CREDENTIALS_get_key(uint8_t* key, sfx_key_type_t KeyType){
    LOG_DEBUG_SFXSX1276((">> CREDENTIALS_get_key\r\n"));

	switch (KeyType) {
    case CREDENTIALS_KEY_PUBLIC: {
      uint8_t pkey[AES_KEY_LEN] = ITSDK_SIGFOX_KEY_PUBLIC;
      memcpy(key, pkey, AES_KEY_LEN);
      itsdk_encrypt_cifferKey(key,16);
    }
    break;
    case CREDENTIALS_KEY_PRIVATE: {
      itsdk_sigfox_getKEY(key);
    }
    break;
    default:
        LOG_ERROR_SFXSX1276(("   Invalid type of key\r\n"));
      break;
  }
}


#endif

