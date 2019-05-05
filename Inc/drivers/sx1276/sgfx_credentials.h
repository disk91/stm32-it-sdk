/******************************************************************************
 * @file    sgfx_credentials.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   Header for driver sgfx_credentials.c module
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CREDENTIALS_H__
#define __CREDENTIALS_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define AES_KEY_LEN 16 //bytes
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

#include <stdint.h>
#include <drivers/sigfox/sigfox_types.h>



/**
 * @brief get the cmac library version
 * @retval return a string containing the library version
 */
const char* CREDENTIALS_get_version( void );

/**
 * @brief get the loaded dev_id 
 * @param[out] return the dev_id in the pointer 
 */
void CREDENTIALS_get_dev_id( uint8_t* dev_id);

/**
 * @brief get the loaded pac 
 * @param[out] return the pac in the pointer 
 */  
void CREDENTIALS_get_initial_pac( uint8_t* pac);

/**
 * @brief get the payload_encryption_flag
 * @retval  the payload_encryption_flag
 */ 
sfx_bool CREDENTIALS_get_payload_encryption_flag(void);

/**
 * @brief encrypts the data_to_encrypt with aes secrete Key
 * @param[out] the encrypted data
 * @param[in] the data_to_encrypt
 * @param[in] the number of aes blocks
 */ 
sfx_error_t CREDENTIALS_aes_128_cbc_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint8_t block_len);

/**
 * @brief encrypts the data_to_encrypt with aes session Key
 * @param[out] the encrypted data
 * @param[in] the data_to_encrypt
 * @param[in] the number of aes blocks
 */ 
sfx_error_t CREDENTIALS_aes_128_cbc_encrypt_with_session_key(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint8_t block_len);

/**
 * @brief wraps the session Key
 * @param[in] the arguments used to generate the session Key
 * @param[in] the number of aes blocks
 */ 
sfx_error_t CREDENTIALS_wrap_session_key( uint8_t* data, uint8_t blocks);

#ifdef __cplusplus
}
#endif

#endif /* __SGFX_CREDENTIALS_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

