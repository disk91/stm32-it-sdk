/*!
 \if SIGFOX PATTERN
 ----------------------------------------------

   _____   _   _____   _____   _____  __    __      
  /  ___/ | | /  ___| |  ___| /  _  \ \ \  / /      
  | |___  | | | |     | |__   | | | |  \ \/ /       
  \___  \ | | | |  _  |  __|  | | | |   }  {        
   ___| | | | | |_| | | |     | |_| |  / /\ \
  /_____/ |_| \_____/ |_|     \_____/ /_/  \_\

  ----------------------------------------------

    !!!!  DO NOT MODIFY THIS FILE !!!!

  ----------------------------------------------
 \endif
  ----------------------------------------------*/
/*!
 * \file mcu_api.h
 * \brief Sigfox manufacturer functions
 * \author $(SIGFOX_LIB_AUTHOR)
 * \version $(SIGFOX_LIB_VERSION)
 * \date $(SIGFOX_LIB_DATE)
 * \copyright Copyright (c) 2011-2015 SIGFOX, All Rights Reserved. This is unpublished proprietary source code of SIGFOX.
 *
 * This file defines the manufacturer's MCU functions to be implemented
 * for library usage.
 */

/********************************************************
 * External API dependencies to link with this library.
 *
 * Error codes of the MCU API functions are described below.
 * The Manufacturer can add more error code taking care of the limits defined.
 * 
 ********************************************************/

/*!
 * \defgroup MCU_ERR_API_xx codes Return Error codes definition for MCU API
 *
 * \brief Can be customized to add new error codes.
 * All MCU_API_ error codes will be piped with SIGFOX_API_xxx return code.<BR>
 *
 * IMPORTANT : SFX_ERR_NONE return code is mandatory when no error for each MCU_API_xxx RF_API_xxx REPEATER_API_xxx or SE_API_xxx
 * functions.
 *
 *  @{
 */
#ifndef IT_SDK_DRIVERS_S2LP_MCU_API_H_
#define IT_SDK_DRIVERS_S2LP_MCU_API_H_
#include <stdbool.h>
#include <it_sdk/config.h>
#include <drivers/sigfox/sigfox_types.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/s2lp/s2lp.h>

#define MCU_API_VER		"v2.3.5"



/* ---------------------------------------------------------------- */
/* Make ST Code cleaner										        */
/* ---------------------------------------------------------------- */
#define ST_TRUE  1
#define ST_FALSE 0

#if defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32F0XX_NUCLEO)
#define IRQ_PRIORITY 0x00
#else
#define IRQ_PRIORITY 0x0A
#endif
/* ---------------------------------------------------------------- */
/* Bytes reserved for MCU API ERROR CODES : From 0x10 to 0x2F       */
/* ---------------------------------------------------------------- */

#define MCU_ERR_API_MALLOC                          (sfx_u8)(0x11) /*!< Error on MCU_API_malloc */
#define MCU_ERR_API_FREE                            (sfx_u8)(0x12) /*!< Error on MCU_API_free */
#define MCU_ERR_API_VOLT_TEMP                       (sfx_u8)(0x13) /*!< Error on MCU_API_get_voltage_temperature */
#define MCU_ERR_API_DLY                             (sfx_u8)(0x14) /*!< Error on MCU_API_delay */
#define MCU_ERR_API_AES                             (sfx_u8)(0x15) /*!< Error on MCU_API_aes_128_cbc_encrypt */
#define MCU_ERR_API_GETNVMEM                        (sfx_u8)(0x16) /*!< Error on MCU_API_get_nv_mem */
#define MCU_ERR_API_SETNVMEM                        (sfx_u8)(0x17) /*!< Error on MCU_API_set_nv_mem */
#define MCU_ERR_API_TIMER_START                     (sfx_u8)(0x18) /*!< Error on MCU_API_timer_start */
#define MCU_ERR_API_TIMER_START_CS                  (sfx_u8)(0x19) /*!< Error on MCU_API_timer_start_carrier_sense */
#define MCU_ERR_API_TIMER_STOP_CS                   (sfx_u8)(0x1A) /*!< Error on MCU_API_timer_stop_carrier_sense */
#define MCU_ERR_API_TIMER_STOP                      (sfx_u8)(0x1B) /*!< Error on MCU_API_timer_stop */
#define MCU_ERR_API_TIMER_END                       (sfx_u8)(0x1C) /*!< Error on MCU_API_timer_wait_for_end */
#define MCU_ERR_API_TEST_REPORT                     (sfx_u8)(0x1D) /*!< Error on MCU_API_report_test_result */
#define MCU_ERR_API_GET_VERSION                     (sfx_u8)(0x1E) /*!< Error on MCU_API_get_version */

#define MCU_ERR_API_GET_ID_PAYLOAD_ENCR_FLAG        (sfx_u8)(0x1F) /*!< Error on MCU_API_get_device_id_and_payload_encryption_flag */
#define MCU_ERR_API_GET_PAC                         (sfx_u8)(0x20) /*!< Error on MCU_API_get_initial_pac */

#define MCU_ERR_API_AES_GET_CRYPT           		(sfx_u8)(0x21) /*!< Error on MCU_API_get_encrypted_info */
#define MCU_ERR_API_AES_DECRYPT             		(sfx_u8)(0x22) /*!< Error on MCU_API_aes_128_cbc_decrypt */


/* ---------------------------------------------------------------- */
/* Bytes reserved for RF API ERROR CODES : From 0x30 to 0x3F        */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Bytes reserved for SE API ERROR CODES : From 0x40 to 0x5F        */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Bytes reserved for REPEATER API ERROR CODES : From 0x60 to 0x7F  */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Bytes reserved for MONARCH API ERROR CODES : From 0x80 to 0x8F   */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------------- */
/* Bytes reserved for DEVICE CONFIG API ERROR CODES : From 0x90 to 0x9A   */
/* ---------------------------------------------------------------------- */

/** @}*/


/*!******************************************************************
 * \fn sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
 * \brief Allocate memory for library usage (Memory usage = size (Bytes))
 * This function is only called once at the opening of the Sigfox Library ( SIGF
 *
 * IMPORTANT NOTE:
 * --------------
 * The address reported need to be aligned with architecture of the microprocessor used.
 * For a Microprocessor of:
 *   - 8 bits  => any address is allowed
 *   - 16 bits => only address multiple of 2 are allowed
 *   - 32 bits => only address multiple of 4 are allowed
 *
 * \param[in] sfx_u16 size                  size of buffer to allocate in bytes
 * \param[out] sfx_u8 **returned_pointer    pointer to buffer (can be static)
 *
 * \retval SFX_ERR_NONE:              No error
 * \retval MCU_ERR_API_MALLOC         Malloc error
 *******************************************************************/
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_free(sfx_u8 *ptr)
 * \brief Free memory allocated to library
 *
 * \param[in] sfx_u8 *ptr                        pointer to buffer
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_FREE:                     Free error
 *******************************************************************/
sfx_u8 MCU_API_free(sfx_u8 *ptr);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle, sfx_u16 *voltage_tx, sfx_s16 *temperature)
 * \brief Get voltage and temperature for Out of band frames
 * Value must respect the units bellow for <B>backend compatibility</B>
 *
 * \param[in] none
 * \param[out] sfx_u16 *voltage_idle             Device's voltage in Idle state (mV)
 * \param[out] sfx_u16 *voltage_tx               Device's voltage in Tx state (mV) - for the last transmission
 * \param[out] sfx_s16 *temperature              Device's temperature in 1/10 of degrees celcius
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_VOLT_TEMP:                Get voltage/temperature error
 *******************************************************************/
sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle,
                                       sfx_u16 *voltage_tx,
                                       sfx_s16 *temperature);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_delay(sfx_delay_t delay_type)
 * \brief Inter stream delay, called between each RF_API_send
 * - SFX_DLY_INTER_FRAME_TX  : 0 to 2s in Uplink DC
 * - SFX_DLY_INTER_FRAME_TRX : 500 ms in Uplink/Downlink FH & Downlink DC 
 * - SFX_DLY_OOB_ACK :         1.4s to 4s for Downlink OOB
 * - SFX_CS_SLEEP :            delay between several trials of Carrier Sense (for the first frame only) 
 *
 * \param[in] sfx_delay_t delay_type             Type of delay to call
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_DLY:                      Delay error
 *******************************************************************/
sfx_u8 MCU_API_delay(sfx_delay_t delay_type);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_aes_128_cbc_encrypt(sfx_u8 *encrypted_data, sfx_u8 *data_to_encrypt, sfx_u8 aes_block_len, sfx_u8 key[16], sfx_credentials_use_key_t use_key)
 * \brief Encrypt a complete buffer with Secret or Test key.<BR>
 * The secret key corresponds to the private key provided from the CRA.
 * <B>These keys must be stored in a secure place.</B> <BR>
 * Can be hardcoded or soft coded (iv vector contains '0')
 *
 * \param[out] sfx_u8 *encrypted_data            Result of AES Encryption
 * \param[in] sfx_u8 *data_to_encrypt            Input data to Encrypt
 * \param[in] sfx_u8 aes_block_len               Input data length (should be a multiple of an AES block size, ie. AES_BLOCK_SIZE bytes)
 * \param[in] sfx_u8 key[16]                     Input key
 * \param[in] sfx_credentials_use_key_t use_key  Key to use - private key or input key
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_AES:                      AES Encryption error
 *******************************************************************/
sfx_u8 MCU_API_aes_128_cbc_encrypt(sfx_u8 *encrypted_data,
                                   sfx_u8 *data_to_encrypt,
                                   sfx_u8 aes_block_len,
                                   sfx_u8 key[16],
                                   sfx_credentials_use_key_t use_key);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE]) 
 * \brief This function copies the data read from non volatile memory
 * into the buffer pointed by read_data.<BR>
 * The size of the data to read is \link SFX_NVMEM_BLOCK_SIZE \endlink
 * bytes.
 * CAREFUL : this value can change according to the features included
 * in the library (covered zones, etc.)
 *
 * \param[in] none
 * \param[out] sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE] Pointer to the data bloc to write with the data stored in memory
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GETNVMEM:                 Read nvmem error
 *******************************************************************/
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE]);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE]) 
 * \brief This function writes data pointed by data_to_write to non
 * volatile memory.<BR> It is strongly recommanded to use NV memory
 * like EEPROM since this function is called at each SIGFOX_API_send_xxx.
 * The size of the data to write is \link SFX_NVMEM_BLOCK_SIZE \endlink
 * bytes.
 * CAREFUL : this value can change according to the features included
 * in the library (covered zones, etc.)
 *
 * \param[in] sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE] Pointer to data bloc to be written in memory
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_SETNVMEM:                 Write nvmem error
 *******************************************************************/
sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE]);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
 * \brief Start timer for :
 * - carrier sense maximum window (used in ARIB standard)
 *
 * \param[in] sfx_u16 time_duration_in_ms        Timer value in milliseconds
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_START_CS:           Start CS timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
 * \brief Start timer for in second duration 
 *
 * \param[in] sfx_u32 time_duration_in_s         Timer value in seconds
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_START:              Start timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_stop(void)
 * \brief Stop the timer (started with MCU_API_timer_start)
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_STOP:               Stop timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_stop(void);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_stop_carrier_sense(void)
 * \brief Stop the timer (started with MCU_API_timer_start_carrier_sense)
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_STOP_CS:            Stop timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_stop_carrier_sense(void);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_wait_for_end(void)
 * \brief Blocking function to wait for interrupt indicating timer
 * elapsed.<BR> This function is only used for the 20 seconds wait
 * in downlink.
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_END:                Wait end of timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_wait_for_end(void);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
 * \brief To report the result of Rx test for each valid message
 * received/validated by library.<BR> Manufacturer api to show the result
 * of RX test mode : can be uplink radio frame or uart print or
 * gpio output.
 * RSSI parameter is only used to report the rssi of received frames (downlink test)
 *
 * \param[in] sfx_bool status                    Is SFX_TRUE when result ok else SFX_FALSE
 *                                               See SIGFOX_API_test_mode summary
 * \param[out] rssi                              RSSI of the received frame 
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TEST_REPORT:              Report test result error
 *******************************************************************/
sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size)
 * \brief Returns current MCU API version
 *
 * \param[out] sfx_u8 **version                  Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                      Size of the byte array pointed by *version
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_VERSION:              Get Version error 
 *******************************************************************/
sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(sfx_u8 dev_id[ID_LENGTH], sfx_bool *payload_encryption_enabled)
 * \brief This function copies the device ID in dev_id, and
 * the payload encryption flag in payload_encryption_enabled.
 *
 * \param[in]  none
 * \param[out] sfx_u8 dev_id[ID_LENGTH]          Pointer on the device ID
 * \param[out] sfx_bool *payload_encryption_enabled  Payload is encrypted if SFX_TRUE, not encrypted else
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_ID_PAYLOAD_ENCR_FLAG: Error when getting device ID or payload encryption flag
 *******************************************************************/
sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(sfx_u8 dev_id[ID_LENGTH], sfx_bool *payload_encryption_enabled);

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_initial_pac(sfx_u8 initial_pac[PAC_LENGTH])
 * \brief Get the value of the initial PAC stored in the device. This
 * value is used when the device is registered for the first time on
 * the backend.
 *
 * \param[in]  none
 * \param[out] sfx_u8 *initial_pac               Pointer to initial PAC
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_PAC:                  Error when getting initial PAC
 *******************************************************************/
sfx_u8 MCU_API_get_initial_pac(sfx_u8 initial_pac[PAC_LENGTH]);

void ST_MCU_API_SetEncryptionPayload(uint8_t ePayload);

typedef enum
{
  NVM_RW_OK = 0,
  NVM_WRITE_ERROR,
  NVM_READ_ERROR,
  NVM_WRITE_RECORD_ERROR,
  NVM_READ_RECORD_ERROR,
  NVM_WRITE_HEADER_ERROR,
  NVM_NO_RECORDS
} NVM_RW_RESULTS;

typedef struct
{
  uint32_t id;
  uint32_t rcz;
  uint8_t  pac[8];
  uint8_t  key[16];
  uint8_t  aux[16];
  int32_t freqOffset;
  int32_t rssiOffset;
  uint32_t checkSum;
} NVM_BoardDataType;

NVM_RW_RESULTS NVM_ReadBoardData(NVM_BoardDataType *data);
NVM_RW_RESULTS NVM_Read(uint32_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer);

void enc_utils_retrieve_key(uint8_t * key);


#endif //  IT_SDK_DRIVERS_S2LP_MCU_API_H_
