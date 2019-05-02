/******************************************************************************
 * @file    se_api.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   interface to se_api.c module
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
 * \file se_api.h
 * \brief Sigfox secure element functions
 * \author $(SIGFOX_LIB_AUTHOR)
 * \version $(SIGFOX_LIB_VERSION)
 * \date $(SIGFOX_LIB_DATE)
 * \copyright Copyright (c) 2011-2015 SIGFOX, All Rights Reserved. This is unpublished proprietary source code of SIGFOX.
 *
 * This file defines the functions relative to secure element, and called by the Sigfox library.
 */

/********************************************************
 * External API dependencies to link with this library.
 *
 * Error codes of the SE API functions are described below.
 * The Manufacturer can add more error code taking care of the limits defined.
 * 
 ********************************************************/

/*!
 * \defgroup SE_ERR_API_xx codes Return Error codes definition for SE API
 *
 * \brief Can be customized to add new error codes.
 * All SE_API_ error codes will be piped with SIGFOX_API_xxx return code.<BR>
 * SFX_ERR_NONE implementation is mandatory for each MCU_API_xxx RF_API_xxx REPEATER_API_xxx or SE_API_xxx
 * functions.
 *
 *  @{
 */
#include <drivers/sigfox/sigfox_types.h>
#include <drivers/sigfox/sigfox_api.h>


/* ---------------------------------------------------------------- */
/* Bytes reserved for MCU API ERROR CODES : From 0x10 to 0x2F       */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Bytes reserved for RF API ERROR CODES : From 0x30 to 0x3F        */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Bytes reserved for SE API ERROR CODES : From 0x40 to 0x5F        */
/* ---------------------------------------------------------------- */
#define SE_ERR_API_INIT                       (sfx_u8)(0x40) /*!< Error on SE_API_init */
#define SE_ERR_API_OPEN                       (sfx_u8)(0x41) /*!< Error on SE_API_open */
#define SE_ERR_API_CLOSE                      (sfx_u8)(0x42) /*!< Error on SE_API_close */
#define SE_ERR_API_SECURE_UP_MSG              (sfx_u8)(0x43) /*!< Error on SE_API_secure_uplink_message */
#define SE_ERR_API_VERIFY_DL_MSG              (sfx_u8)(0x44) /*!< Error on SE_API_verify_downlink_message */
#define SE_ERR_API_GET_ID                     (sfx_u8)(0x45) /*!< Error on SE_API_get_device_id */
#define SE_ERR_API_GET_PAC                    (sfx_u8)(0x46) /*!< Error on SE_API_get_initial_pac */
#define SE_ERR_API_GET_VERSION                (sfx_u8)(0x47) /*!< Error on SE_API_get_version */
#define SE_ERR_API_RC_PERIOD                  (sfx_u8)(0x48) /*!< Error: set RCSync period but SE is not encrypting data */
#define SE_ERR_API_GEN_RCSYNC                 (sfx_u8)(0x49) /*!< Error: RCSync frame required but SE is not encrypting data */

/* ---------------------------------------------------------------- */
/* Bytes reserved for REPEATER API ERROR CODES : From 0x60 to 0x7F  */
/* ---------------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* Bytes reserved for MCU MONARCH API ERROR CODES : From 0x80 to 0x8F  */
/* ------------------------------------------------------------------- */


/** @}*/

/********************************
 * \enum sfx_se_frame_type_t
 * \brief Frame type used within the SE to 
 * identify the type of frame which has to 
 * be build. 
 *******************************/
typedef enum
{
    SFX_SE_FRAME,        /*!< All Frame types except RC SYNC  */
    SFX_SE_RC_SYNC,      /*!< RC SYNC Frame type  */
} sfx_se_frame_type_t;

/*!******************************************************************
 * \fn sfx_u8 SE_API_get_version(sfx_u8 **version, sfx_u8 *size)
 * \brief Returns current SE API version
 *
 * \param[out] sfx_u8 **version                 Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                     Size of the byte array pointed by *version
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_GET_VERSION:              Get Version error 
 *******************************************************************/
sfx_u8 SE_API_get_version(sfx_u8 **version, sfx_u8 *size);

/*!******************************************************************
 * \fn sfx_u8 SE_API_init(void)
 * \brief Initialization of the interface between the MCU and the SE.
 * (configuring interface timings, SE address on bus, etc.)
 *
 * \param[in]  none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_INIT:                     Error on SE_API_init
 *******************************************************************/
sfx_u8 SE_API_init(void);

/*!******************************************************************
 * \fn sfx_u8 SE_API_open(void)
 * \brief Starting communication with SE. SE is powered up.
 * SE must be operational when SE_API_open returns (ie. SE boot time
 * must be considered here).
 *
 * \param[in]  none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_OPEN:                     Error on SE_API_open
 *******************************************************************/
sfx_u8 SE_API_open(void);

/*!******************************************************************
 * \fn sfx_u8 SE_API_close(void)
 * \brief Stopping communication with SE. SE is powered down.
 *
 * \param[in]  none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_CLOSE:                    Error on SE_API_close
 *******************************************************************/
sfx_u8 SE_API_close(void);

/*!******************************************************************
 * \fn sfx_u8 SE_API_get_device_id(sfx_u8 dev_id[ID_LENGTH])
 * \brief This function copies the device ID in dev_id.
 *
 * \param[in]  none
 * \param[out] sfx_u8 dev_id[ID_LENGTH]         Pointer where to write the device ID
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_GET_ID:                   Error when getting device ID
 *******************************************************************/
sfx_u8 SE_API_get_device_id(sfx_u8 dev_id[ID_LENGTH]);

/*!******************************************************************
 * \fn sfx_u8 SE_API_get_initial_pac(sfx_u8 *initial_pac)
 * \brief Get the initial PAC contained in the secure element
 *
 * \param[in]  none
 * \param[out] sfx_u8 *initial_pac              Pointer where to write the initial PAC
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_GET_PAC:                  Error when getting initial PAC
 *******************************************************************/
sfx_u8 SE_API_get_initial_pac(sfx_u8 *initial_pac);

/*!******************************************************************
 * \fn sfx_u8 SE_API_secure_uplink_message(sfx_u8 *customer_data, sfx_u8 customer_data_length, sfx_bool initiate_downlink_frame, sfx_se_frame_type_t frame_type, sfx_bool *send_rcsync, sfx_u8 *frame_ptr, sfx_u8 *frame_length)
 * \brief Generation of an uplink frame bitstream
 *
 * \param[in]  sfx_u8 *customer_data            Pointer to customer data to send
 * \param[in]  sfx_u8 customer_data_length      Length of customer data
 * \param[in]  sfx_bool initiate_downlink_frame SFX_TRUE if a downlink response is requested, SFX_FALSE else
 * \param[in]  sfx_se_frame_type_t              When set to RC_SYNC, the frame must NOT be encrypted and rollover counter has to be added. Otherwise, treat it as a classic frame with encryption payload if needed.
 * \param[out] sfx_bool * send_rcsync           SFX_TRUE if a rsync frame needs to be sent, SFX_FALSE else
 * \param[out] sfx_u8 *frame_ptr                Pointer to the buffer to fill with built bitstream
 * \param[out] sfx_u8 *frame_length             Length of built bistream (including synchro bit/frame)
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_SECURE_UP_MSG             Error on SE_API_secure_uplink_message
 *******************************************************************/
sfx_u8 SE_API_secure_uplink_message(sfx_u8 *customer_data,
                                    sfx_u8 customer_data_length,
                                    sfx_bool initiate_downlink_frame,
                                    sfx_se_frame_type_t frame_type,
                                    sfx_bool *send_rcsync,
                                    sfx_u8 *frame_ptr,
                                    sfx_u8 *frame_length);

/*!******************************************************************
 * \fn sfx_u8 SE_API_verify_downlink_message(sfx_u8 *frame_ptr, sfx_bool *valid)
 * \brief Authentification (and payload decryption) of a received message
 *
 * \param[in] sfx_u8 *frame_ptr                 Pointer to the received frame
 * \param[out] sfx_bool *valid                  SFX_TRUE when frame is correctly authenticated, else SFX_FALSE
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_VERIFY_DL_MSG             Error on SE_API_verify_downlink_message
 *******************************************************************/
sfx_u8 SE_API_verify_downlink_message(sfx_u8 *frame_ptr, sfx_bool *valid);

/*!******************************************************************
 * \fn sfx_u8 SE_API_set_rc_sync_period(sfx_u16 period)
 * \brief Set the rcsync frame send periodicity.
 *
 * \param[in] sfx_u16 period                    Rcsync frame send period
 *
 * \retval SFX_ERR_NONE:                        No error
 * \retval SE_ERR_API_RC_PERIOD:                Set RCSync period but SE is not encrypting data
 *******************************************************************/
sfx_u8 SE_API_set_rc_sync_period(sfx_u16 rc_sync_period);

