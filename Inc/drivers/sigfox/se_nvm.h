/******************************************************************************
 * @file    se_nvm.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   header to emulated SE nvm datas
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
#ifndef __SE_NVM_H__
#define __SE_NVM_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <drivers/sigfox/sigfox_types.h>
#include <drivers/sigfox/sigfox_api.h>

/* Exported types ------------------------------------------------------------*/
   
typedef enum
{ 
  CREDENTIALS_KEY_PRIVATE= 0,    /* private key */ 
  CREDENTIALS_KEY_PUBLIC = 1,    /* public key  <i> 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF</i>.*/ 
}sfx_key_type_t;

typedef enum
{
    SE_NVMEM_SEQNUM         = 0,                 /*!< Index of nv memory for PN */
    SE_NVMEM_RCSYNC_PERIOD  = 2,                 /*!< Index of nv memory for dedicated FH information */
    SE_NVMEM_ROLLOVER       = 4,
} se_nvmem_t;

/* Exported constants --------------------------------------------------------*/
#define SFX_SE_NVMEM_BLOCK_SIZE  5
#define SE_ERR_API_SE_NVM                 (sfx_u8)(0x4A) 
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/**
 * @brief This function reads data pointed by read_data to non
 * volatile memory
 *
 * @param [IN] sfx_u8 read_data[SFX_SE_NVMEM_BLOCK_SIZE] Pointer to the data bloc to write with the data stored in memory
 * @retval SFX_ERR_NONE:                      No error
 * @retval SE_ERR_API_SE_NVM:                 SE nvmem error
 */
sfx_u8 SE_NVM_get(sfx_u8 read_data[SFX_SE_NVMEM_BLOCK_SIZE]);

/**
 * @brief This function write data pointed by data_to_write to non
 * volatile memory
 *
 * @param [IN] sfx_u8 data_to_write[SFX_SE_NVMEM_BLOCK_SIZE] Pointer to the data bloc to write with the data stored in memory
 * @retval SFX_ERR_NONE:                      No error
 * @retval SE_ERR_API_SE_NVM:                 SE nvmem error
 */
sfx_u8 SE_NVM_set(sfx_u8 data_to_write[SFX_SE_NVMEM_BLOCK_SIZE]);

/**
 * @brief Get the active encryption key
 * @retval  sfx_key_type_t key: public or private
 */
sfx_key_type_t SE_NVM_get_key_type( void );

/**
 * @brief Set the active encryption key
 * @param [IN]  sfx_key_type_t key: public or private
 */
void  SE_NVM_set_key_type( sfx_key_type_t keyType );
  
#ifdef __cplusplus
}
#endif

#endif /* __SE_NVM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

