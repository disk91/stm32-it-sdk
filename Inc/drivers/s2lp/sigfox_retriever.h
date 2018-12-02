/**
* @file    sigfox_retriever.h
* @author  LowPower RF BU - AMG
* @version 1.3.0
* @date    Jul 3, 2018
* @brief   This is used to retrieve the SigFox data as ID, PAC and AES-KEY.
*          The AES-KEY is a private variable and is not returned to the user.
* @details
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
*/

#include <stdint.h>


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIGFOX_RETRIEVER_H
#define __SIGFOX_RETRIEVER_H

#ifdef __cplusplus
 extern "C" {
#endif
   
   
/* Retriever error codes definition */
typedef uint8_t      retr_error_t;

typedef enum {  
  KEY_ENC_FIXED = 0x01, 
  KEY_ENC_VARIABLE = 0x02,
  KEY_ENC_NONE = 0x03
} KEY_ENC_MODES;

#ifndef UID_BASE

#ifdef STM32L152xE
#define UID_BASE	((uint32_t)0x1FF800D0U)
#endif

#ifdef STM32F072xB
#define UID_BASE	((uint32_t)0x1FFFF7ACU)
#endif

#ifdef STM32L053xx
#define UID_BASE	((uint32_t)0x1FF80050U)
#endif

#ifdef STM32F401xE
#define UID_BASE	0x1FFF7A10U
#endif

#if (defined(BLUENRG2_DEVICE) || defined(BLUENRG1_DEVICE))
#define UID_BASE 0x100007F4 //First 6 byte + 55AA
#endif

#endif // UID_BASE

/*!
 * \defgroup ST_SIGFOX_RETRIEVER
 *
 *  @{
 */

/*!
 * \defgroup RETRIEVER_ERR_CODES Return Error codes definition for the ST_SIGFOX_RETRIEVER
 *
 *  @{
 */
#define RETR_OK         0     /* no error */
#define RETR_ERR        1     /* error */

/** @}*/

/*!******************************************************************
 * \fn retr_error_t enc_utils_retrieve_data(uint32_t *id, uint8_t *pac, uint8_t *rcz)
 * \brief Retrieve the ID, PAC and RCZ number of the board and returns it to the caller.
 *        The ID should be used when opening the library. The PAC is used to register the node on the backend.
 * \param[in] id: pointer to the 32bits word variable where the ID of the board must be stored.
 * \param[in] pac: pointer to the 8bytes array where the PAC of the board must be stored.
 * \param[in] rcz: pointer to the byte where the RCZ number of this board must be stored.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_retrieve_data(uint32_t *id, uint8_t *pac, uint8_t *rcz);

/*!******************************************************************
 * \fn retr_error_t enc_utils_retrieve_data_from_flash(uint32_t *id, uint8_t *pac, uint8_t *rcz, int32_t *freqOffset, int8_t *rssiOffset)
 * \brief Retrieve the ID, PAC and RCZ number of the board and returns it to the caller.
 *        The ID should be used when opening the library. The PAC is used to register the node on the backend.
 * \param[in] id: pointer to the 32bits word variable where the ID of the board must be stored.
 * \param[in] pac: pointer to the 8bytes array where the PAC of the board must be stored.
 * \param[in] rcz: pointer to the byte where the RCZ number of this board must be stored.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_retrieve_data_from_flash(uint32_t *id, uint8_t *pac, uint8_t *rcz, int32_t *freqOffset, int8_t *rssiOffset);

/*!******************************************************************
 * \fn retr_error_t enc_utils_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint16_t data_len)
 * \brief Perform the AES128-CBC encryption using the AES KEY associated to the board.
 * \param[in] uint8_t* encrypted_data: pointer to the destination buffer where the encrypted data must be stored.
 * \param[in] uint8_t* data_to_encrypt: pointer to the source buffer where the data to encrypt are stored.
 * \param[in] uint16_t data_len: length of the data buffer to encrypt.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint16_t data_len, uint8_t* key, uint8_t useExternalKey);

/*!******************************************************************
 * \fn retr_error_t enc_utils_set_public_key(uint8_t en)
 * \brief Switch the encryption key to the public key: <i>0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF</i>.
 * \param[in] uint8_t en: if 1 switch to the public key, else restore the one associated with the board.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_set_public_key(uint8_t en);

/*!******************************************************************
 * \fn retr_error_t enc_utils_set_test_key(uint8_t en)
 * \brief Switch the to the test KEY <i>0x0123456789ABCDEF0123456789ABCDEF</i>.
 * \param[in] uint8_t en: if 1 switch to the test KEY, else restore the one associated with the board.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_set_test_key(uint8_t en);

/*!******************************************************************
 * \fn retr_error_t enc_utils_set_test_id(uint8_t en)
 * \brief Switch the to the test ID <i>0xFEDCBA98</i>.
 * \param[in] uint8_t en: if 1 switch to the test ID, else restore the one associated with the board.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_set_test_id(uint8_t en);

/*!******************************************************************
 * \fn retr_error_t enc_utils_get_id(uint8_t *id)
 * \brief Gets the ID from the EEPROM on the board.
 * \param[in] uint8_t *id: pointer to the array where the ID should be stored.
 * \note: The function \ref enc_utils_retrieve_data should be called in the 
 *  initialization phase of the board.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_get_id(uint8_t *id);

/*!******************************************************************
 * \fn retr_error_t enc_utils_get_initial_pac(uint8_t *pac)
 * \brief Gets the PAC from the EEPROM on the board.
 * \param[in] uint8_t *pac: pointer to the array where the PAC should be stored.
 * \note: The function \ref enc_utils_retrieve_data should be called in the 
 *  initialization phase of the board.
 * \retval Error code: RETR_OK (0) or RETR_ERR (1)
 *******************************************************************/
retr_error_t enc_utils_get_initial_pac(uint8_t *pac);


/** @}*/

#ifdef __cplusplus
}
#endif

#endif /* __SIGFOX_RETRIEVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
