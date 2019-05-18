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
    \mainpage Sigfox Protocol V1 library API documentation

    Library implementing Sigfox protocol V1.
    The picture below describes the architecture of a Sigfox Device organized as software Layers.<br>
 
    \image html sw_architecture.png 
*/
/*!
 * \file sigfox_api.h
 * \brief Sigfox user functions
 * \author $(SIGFOX_LIB_AUTHOR)
 * \version $(SIGFOX_LIB_VERSION)
 * \date $(SIGFOX_LIB_DATE)
 * \copyright Copyright (c) 2011-2015 SIGFOX, All Rights Reserved. This is unpublished proprietary source code of SIGFOX.
 *
 * This file includes the user's functions to send data on sigfox's network,
 * such as sending a bit, a frame or a sigfox out of band message.
 */

/* ################################## VERSION INFORMATION ################################## */
/* This library supports FH, DC and LBT spectrum access to be compliant with ETSI, FCC and ARIB standards. */


/* Warning : this library supports the Monarch feature */

/* This library implements the payload encryption feature. */

/* ######################################################################################### */


/*!
 * \defgroup SFX_ERR_CODES Return Error codes definition for SIGFOX APIs
 *
 *  @{
 */

/* ------------------------------------------------------------------------------------------------------------------------------------- 
                                            IMPORTANT NOTE on ERROR CODES                             
   ------------------------------------------------------------------------------------------------------------------------------------- 

   ALL SIGFOX_API_xxx or SIGFOX_REPEATER_API_xxx functions returns an error type sfx_error_t

   The sfx_error_t is composed of the following :

     MSB_____________________________________________LSB
     15                      8|7                     0
      |                       |                      |
      |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
      |_______________________|______________________|


   MANUF_ERROR_CODE :
      - Represents the error codes for MCU_API_xx, RF_API_xx, SE_API_xx, REPEATER_API_xx
      - All MCU_API_xx, RF_API_xx, SE_API_xx, REPEATER_API_xx have to return SFX_ERR_NONE when no error
      - Minimum set of error codes are defined into the mcu_api.h, rf_api.h, se_api.h and repeater_api.h : Manufacturer can define more error codes !

   SIGFOX_ERROR_CODE :
       - Represents the error codes of the SIGFOX API or internal functions
       - All the error codes can be found in this file ( sigfox_api.h ) 
 
   ------------------------------------------------------------------------------------------------------------------------------------- */
#ifndef IT_SDK_DRIVERS_SIGFOX_API_H_
#define IT_SDK_DRIVERS_SIGFOX_API_H_

#include <it_sdk/config.h>
#include <drivers/sigfox/sigfox_types.h>


#define SFX_ERR_NONE                                             (sfx_u8)(0x00) /*!< No error - to be used for MANUF_ERROR_CODES and SIGFOX_ERROR_CODES */

/* ---------------------------------------------------------------- */
/* Bytes reserved for SIGFOX ERROR CODES : From 0x10 to 0xBF        */
/* ---------------------------------------------------------------- */

#define SFX_ERR_API_OPEN                                         (sfx_u8)(0x10) /*!< Error occurs during the opening of the Sigfox Library : check the manuf error code */
#define SFX_ERR_API_OPEN_STATE                                   (sfx_u8)(0x11) /*!< State is not IDLE, library should be closed before */
#define SFX_ERR_API_OPEN_GET_NVMEM_MEMORY_OVERLAP                (sfx_u8)(0x12) /*!< MCU_API_get_nv_mem overlap the memory */
#define SFX_ERR_API_OPEN_RC_PTR                                  (sfx_u8)(0x13) /*!< RC pointer is NULL */
#define SFX_ERR_API_OPEN_MACRO_CHANNEL_WIDTH                     (sfx_u8)(0x14) /*!< This macro channel width is not authorized by Sigfox Lib, check your RC configuration */

#define SFX_ERR_API_CLOSE_FREE                                   (sfx_u8)(0x20) /*!< Error occurs during the closing of the Sigfox Library : error on MCU_API_free */
#define SFX_ERR_API_CLOSE_STATE                                  (sfx_u8)(0x21) /*!< Error occurs during the closing of the Sigfox Library : error on library state */

#define SFX_ERR_API_SEND_FRAME_DATA_LENGTH                       (sfx_u8)(0x30) /*!< Customer data length > 12 Bytes */
#define SFX_ERR_API_SEND_FRAME_RESPONSE_PTR                      (sfx_u8)(0x31) /*!< Response data pointer NULL in case of downlink */
#define SFX_ERR_API_SEND_FRAME_DELAY_OOB_ACK                     (sfx_u8)(0x32) /*!< Error on MCU_API_delay w/ SFX_DLY_OOB_ACK (Downlink) */
#define SFX_ERR_API_SEND_FRAME_DATA_PTR                          (sfx_u8)(0x33) /*!< Customer data pointer NULL */

#define SFX_ERR_API_SEND_BIT_RESPONSE_PTR                        (sfx_u8)(0x34) /*!< Response data pointer NULL in case of downlink */
#define SFX_ERR_API_SEND_OOB_TYPE                                (sfx_u8)(0x35) /*!< Wrong enum value for the OOB type */


#define SFX_ERR_API_SET_STD_CONFIG_CARRIER_SENSE_CONFIG          (sfx_u8)(0x40) /*!< Error on the carrier sense configuration -check the config words */
#define SFX_ERR_API_SET_STD_CONFIG_FH_CHANNELS                   (sfx_u8)(0x41) /*!< Config word empty whereas they should configure channels for Frequency Hopping */

#define SFX_ERR_API_SEND_TEST_FRAME_DEVICE_ID                    (sfx_u8)(0x50) /*!< Device Id that should be used in SIGFOX_API_send_test_frame function must be 0xFEDCBA98 */
#define SFX_ERR_API_SEND_TEST_FRAME_STATE                        (sfx_u8)(0x51) /*!< State is not READY - Should open the library */
#define SFX_ERR_API_SEND_TEST_FRAME_DATA_LENGTH                  (sfx_u8)(0x52) /*!< Customer data length > 12 Bytes */
#define SFX_ERR_API_SEND_TEST_FRAME_DATA_PTR                     (sfx_u8)(0x53) /*!< Customer data pointer NULL */
#define SFX_ERR_API_SEND_TEST_STORE_NVM                          (sfx_u8)(0x54) /*!< Error occurs during the NVM Storage : check the manuf error code to get the error */

#define SFX_ERR_API_RECEIVE_TEST_FRAME_DEVICE_ID                 (sfx_u8)(0x55) /*!< Device Id that should be used in SIGFOX_API_send_test_frame function must be 0xFEDCBA98 */
#define SFX_ERR_API_RECEIVE_TEST_FRAME_STATE                     (sfx_u8)(0x56) /*!< State is not READY - Should open the library */

#define SFX_ERR_API_START_CONTINUOUS_TRANSMISSION                (sfx_u8)(0x57) /*!< Error occurs during the start continuous transmission : check the manuf error code to get the error */
#define SFX_ERR_API_START_CONTINUOUS_TRANSMISSION_STATE          (sfx_u8)(0x58) /*!< State is not idle, library should be closed before */
#define SFX_ERR_API_STOP_CONTINUOUS_TRANSMISSION                 (sfx_u8)(0x59) /*!< Error occurs during the stop continuous transmission : check the manuf error code to get the error */
#define SFX_ERR_API_STOP_CONTINUOUS_TRANSMISSION_STATE           (sfx_u8)(0x5A) /*!< State is not TX, function SIGFOX_API_start_continuous_tranmission has to be called before */

#define SFX_ERR_API_GET_INITIAL_PAC                              (sfx_u8)(0x5B) /*!< Error occurs when trying to retrieve the PAC : check the manuf error code to get the error */ 
#define SFX_ERR_API_GET_VERSION                                  (sfx_u8)(0x5C) /*!< Error occurs when trying to retrieve the version : check the manuf error code to get the error */ 
#define SFX_ERR_API_GET_VERSION_WRONG_TYPE                       (sfx_u8)(0x5D) /*!< Error occurs when trying to retrieve the version : wrong version type - see the enum sfx_version_type_t */
#define SFX_ERR_API_SWITCH_PUBLIC_KEY                            (sfx_u8)(0x5E) /*!< Error occurs when switching device key: state is not READY - Should open the library */

#define SFX_ERR_INT_EXECUTE_COM_SEQUENCE_STATE                   (sfx_u8)(0x60) /*!< State is not READY, library should be opened before */
#define SFX_ERR_INT_EXECUTE_COM_SEQUENCE_NVM_STORAGE_MESSAGE     (sfx_u8)(0x61) /*!< Error occurs during the nvm storage used for uplink transmission : check the manuf error code  */
#define SFX_ERR_INT_EXECUTE_COM_SEQUENCE_NVM_STORAGE_ACK         (sfx_u8)(0x62) /*!< Error occurs during the nvm storage used for ack transmission : check the manuf error code  */
#define SFX_ERR_INT_EXECUTE_COM_SEQUENCE_NVM_STORAGE_RCSYNC      (sfx_u8)(0x63) /*!< Error occurs during the nvm storage used for rc sync transmission : check the manuf error code  */
#define SFX_ERR_INT_EXECUTE_COM_SEQUENCE_DELAY_OOB_ACK           (sfx_u8)(0x64) /*!< Error occurs when setting the delay between downlink and ack : check the manuf error code   */

#define SFX_ERR_INT_PROCESS_UPLINK_START_TIMER_FH_IN_DL          (sfx_u8)(0x70) /*!< Error when calling MCU_API_timer_start for FH : check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_WAIT_FOR_END_TIMER_FH_IN_DL   (sfx_u8)(0x71) /*!< Error when calling MCU_API_timer_stop for FH  : check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_TIMER_FH                      (sfx_u8)(0x72) /*!< Error when starting the timer after the first frame to respect the FCC regulation - timer_enable config set to 1 */
#define SFX_ERR_INT_PROCESS_UPLINK_WAIT_FOR_END_TIMER_FH         (sfx_u8)(0x73) /*!< Error when stoping the timer after the first frame */
#define SFX_ERR_INT_PROCESS_UPLINK_DELAY_INTERFRAME              (sfx_u8)(0x74) /*!< Error when executing the interframe delay : check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_TIMER_DOWNLINK                (sfx_u8)(0x75) /*!< Error when starting the timer after the first frame to prepare the downlink */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_RETRY                      (sfx_u8)(0x76) /*!< Error when executing the Carrier Sense for the first frame : check the manuf error code to get the error */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_RETRY_START_TIMER          (sfx_u8)(0x77) /*!< Error Carrier Sense for the first frame on start timer: check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_RETRY_STOP_TIMER           (sfx_u8)(0x78) /*!< Error Carrier Sense for the first frame on stop timer: check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_RETRY_DELAY_ATTEMPT        (sfx_u8)(0x79) /*!< Error on executing the delay between several attempts of the first frame : check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_REPETITION                 (sfx_u8)(0x7A) /*!< Error Carrier Sense for frame 2 and 3: check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_REPETITION_START_TIMER     (sfx_u8)(0x7B) /*!< Error Carrier Sense for starting the timer for CS on frame 2 and 3: check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_REPETITION_STOP_TIMER      (sfx_u8)(0x7C) /*!< Error Carrier Sense for stoping the timer for CS on frame 2 and 3: check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_REPETITION_STOP_TIMER_2    (sfx_u8)(0x7D) /*!< Error Carrier Sense for stoping the timer for CS on frame 2 and 3: check the manuf error code */
#define SFX_ERR_INT_PROCESS_UPLINK_CS_TIMEOUT                    (sfx_u8)(0x7E) /*!< Error Carrier Sense unsuccessful */

#define SFX_ERR_INT_BUILD_FRAME_SE                               (sfx_u8)(0x90) /*!< Error occurs when building the frame with a SE : check the manuf error code */
#define SFX_ERR_INT_BUILD_FRAME                                  (sfx_u8)(0x91) /*!< Error occurs when building the frame : check the manuf error code */
#define SFX_ERR_INT_BUILD_FRAME_OOB_SERVICE                      (sfx_u8)(0x92) /*!< Error occurs when building the OOB Frame in MCU_API_get_voltage_temperature : check the manuf error code */
#define SFX_ERR_INT_BUILD_FRAME_OOB_DOWNLINK_ACK                 (sfx_u8)(0x93) /*!< Error occurs when building the OOB downlink frame in MCU_API_get_voltage_temperature : check the manuf error code */
#define SFX_ERR_INT_BUILD_FRAME_OOB_REPEATER_STATUS              (sfx_u8)(0x94) /*!< Error occurs when building the OOB REPEATER_FRAME in REPEATER_API_get_voltage : check the manuf error code */            
#define SFX_ERR_INT_BUILD_FRAME_OOB_RC_SYNC                      (sfx_u8)(0x95) /*!< Error occurs when building the OOB RCSYNC frame : check the manuf error code */            
#define SFX_ERR_INT_BUILD_FRAME_PAYLOAD_CRYPTED                  (sfx_u8)(0x96) /*!< Error occurs when building the encrypted frame : check the manuf error code */            

#define SFX_ERR_INT_SEND_SINGLE_FRAME                            (sfx_u8)(0x97)  /*!< Error when sending a frame : check the manuf error code */
#define SFX_ERR_INT_PROCESS_DOWNLINK                             (sfx_u8)(0x98)  /*!< Error when starting the downlink : in MCU_API_timer_wait_for_end : check the manuf error code */

#define SFX_ERR_INT_GET_DEVICE_ID                                (sfx_u8)(0x99)  /*!< Error when retrieving the device ID : check the manuf error code */
#define SFX_ERR_INT_GET_RECEIVED_FRAMES                          (sfx_u8)(0x9A)  /*!< Error occurs when trying to receive frames : check the manuf error code */
#define SFX_ERR_INT_GET_RECEIVED_FRAMES_TIMEOUT                  (sfx_u8)(0x9B)  /*!< Timeout on frame reception */
#define SFX_ERR_INT_GET_RECEIVED_FRAMES_WAIT_NOT_EXECUTED        (sfx_u8)(0x9C)  /*!< State return by the RF_API_wait_frame is downlink reception not executed */

#define SFX_ERR_INT_GET_DEVICE_INFO                              (sfx_u8)(0x9D)  /*!< Error when retrieving the device info : check the manuf error code */
#define SFX_ERR_INT_GET_DEVICE_INFO_CRC                          (sfx_u8)(0x9E)  /*!< Error when checking the validty of the device info - CRC is bad */
#define SFX_ERR_INT_GET_DEVICE_INFO_CERTIFICATE                  (sfx_u8)(0x9F)  /*!< Error when checking the validty of the device info - Certificate is not the appropriate one */


#define SFX_ERR_API_SET_RC_SYNC_PERIOD                                     (sfx_u8)(0xB0) /*!< Set RC Sync frame transmission period failed */
#define SFX_ERR_API_SET_RC_SYNC_PERIOD_VALUE                               (sfx_u8)(0xB1) /*!< Error in the RC Sync period value */

#define SFX_ERR_MONARCH_API_EXECUTE_RC_SCAN_STATE                           (sfx_u8)(0xB2) /*!< State != IDLE : the library needs to be closed */
#define SFX_ERR_MONARCH_API_EXECUTE_RC_SCAN                                 (sfx_u8)(0xB3) /*!< Error when executing the RC scan : check the manuf error code - call the stop_rc_scan to come back in IDLE state */
#define SFX_ERR_MONARCH_API_EXECUTE_RC_SCAN_NULL_CALLBACK                   (sfx_u8)(0xB4) /*!< Application Callback handler is null  */
#define SFX_ERR_MONARCH_API_STOP_RC_SCAN_STATE                              (sfx_u8)(0xB5) /*!< The device is not currently in scan state : there is no reason to call the stop function */
#define SFX_ERR_MONARCH_API_STOP_RC_SCAN                                    (sfx_u8)(0xB6) /*!< Error when executing the stop RC scan : check the manuf error code */
#define SFX_ERR_CALLBACK_MONARCH_SCAN_TIMEOUT_CB_STATE                      (sfx_u8)(0xB7) /*!< The device is not currently in scan mode : there is no reason to call the timeout timer function */
#define SFX_ERR_CALLBACK_MONARCH_SCAN_TIMEOUT                               (sfx_u8)(0xB8) /*!< Error when executing the callback of the timer expiration : check the manuf error code - call the stop_rc_scan to come back in IDLE state */
#define SFX_ERR_CALLBACK_MONARCH_PATTERN_FREQUENCY_RESULT_STATE             (sfx_u8)(0xB9) /*!< The device is not currently in search pattern mode: there is no reason to call the pattern frequency result function */
#define SFX_ERR_CALLBACK_MONARCH_PATTERN_FREQUENCY_RESULT                   (sfx_u8)(0xBA) /*!< Error when executing the callback function of the pattern frequency result : check the manuf error code - call the stop_rc_scan to come back in IDLE state */ 
#define SFX_ERR_CALLBACK_MONARCH_PATTERN_FREQUENCY_RESULT_WRONG_PATTERN     (sfx_u8)(0xBB) /*!< The pattern is not one of the requested frequencies sent by the lib - call the stop_rc_scan to come back in IDLE state*/
#define SFX_ERR_CALLBACK_MONARCH_PATTERN_FREQUENCY_RESULT_WRONG_FREQ        (sfx_u8)(0xBC)  /*!< The frequency is not one of the requested frequencies sent by the lib - call the stop_rc_scan to come back in IDLE state*/


/* ----------------------------------------------------------------------------- */
/* Bytes reserved for SIGFOX VERIFIED APP ERROR CODES : From 0xC0 to 0xCF        */
/* ----------------------------------------------------------------------------- */

 
#define SFX_ERR_INT_DOWNLINK_CONFIGURATION                                  (sfx_u8)(0xE0)  /*!< Error occurs when trying to configure downlink : check the manuf error code */


/** @}*/


/*!
 * \defgroup SIGFOX_RC_CONFIGURATIONS Defines the SIGFOX Radio Configurations
 *
 *  @{
 */

/* Define Radio Configuration */
#define NA                                    0 /* Used to store Not applicable values in config words or open parameters */

#define RC1_OPEN_UPLINK_CENTER_FREQUENCY      (sfx_u32)(868130000) /* Hz */
#define RC1_OPEN_DOWNLINK_CENTER_FREQUENCY    (sfx_u32)(869525000) /* Hz */
#define RC1_MACRO_CHANNEL_WIDTH               (sfx_u32)(192000)    /* Hz */
#define RC1_UPLINK_MODULATION                 SFX_DBPSK_100BPS 
#define RC1_UPLINK_SPECTRUM_ACCESS            SFX_DC

#define RC2_OPEN_UPLINK_START_OF_TABLE        (sfx_u32)(902200000) /* Hz, The center frequency of RC2 is defined by the activated channels in config words */
#define RC2_OPEN_DOWNLINK_CENTER_FREQUENCY    (sfx_u32)(905200000) /* Hz */
#define RC2_MACRO_CHANNEL_WIDTH               (sfx_u32)(192000)    /* Hz */
#define RC2_UPLINK_MODULATION                 SFX_DBPSK_600BPS 
#define RC2_UPLINK_SPECTRUM_ACCESS            SFX_FH 
#define RC2_SET_STD_CONFIG_LM_WORD_0          (sfx_u32)0x000001FF  /* LM = Long Message */
#define RC2_SET_STD_CONFIG_LM_WORD_1          (sfx_u32)0x00000000
#define RC2_SET_STD_CONFIG_LM_WORD_2          (sfx_u32)0x00000000
#define RC2_SET_STD_TIMER_ENABLE              (sfx_bool)(SFX_TRUE) /* Enable Timer for FH duty cycle*/
#define RC2_SET_STD_TIMER_DISABLE             (sfx_bool)(SFX_FALSE)/* Disable timer feature*/
#define RC2_SET_STD_CONFIG_SM_WORD_0          (sfx_u32)0x00000001  /* SM = Short message */
#define RC2_SET_STD_CONFIG_SM_WORD_1          (sfx_u32)0x00000000
#define RC2_SET_STD_CONFIG_SM_WORD_2          (sfx_u32)0x00000000

#define RC3A_OPEN_CS_CENTER_FREQUENCY         (sfx_u32)(923200000) /* Hz */
#define RC3A_OPEN_CS_BANDWIDTH                (sfx_u32)(200000)    /* Hz */
#define RC3A_OPEN_UPLINK_CENTER_FREQUENCY     (sfx_u32)(923200000) /* Hz */
#define RC3A_OPEN_DOWNLINK_CENTER_FREQUENCY   (sfx_u32)(922200000) /* Hz */
#define RC3A_MACRO_CHANNEL_WIDTH              (sfx_u32)(36000)     /* Hz */
#define RC3A_UPLINK_MODULATION                SFX_DBPSK_100BPS 
#define RC3A_UPLINK_SPECTRUM_ACCESS           SFX_LBT 
#define RC3A_CS_THRESHOLD                     (sfx_s8)(-80) /* dBm */

#define RC3C_OPEN_CS_CENTER_FREQUENCY         (sfx_u32)(923200000) /* Hz */
#define RC3C_OPEN_CS_BANDWIDTH                (sfx_u32)(200000)    /* Hz */
#define RC3C_OPEN_UPLINK_CENTER_FREQUENCY     (sfx_u32)(923200000) /* Hz */
#define RC3C_OPEN_DOWNLINK_CENTER_FREQUENCY   (sfx_u32)(922200000) /* Hz */
#define RC3C_MACRO_CHANNEL_WIDTH              (sfx_u32)(192000)    /* Hz */
#define RC3C_UPLINK_MODULATION                SFX_DBPSK_100BPS 
#define RC3C_UPLINK_SPECTRUM_ACCESS           SFX_LBT 
#define RC3C_CS_THRESHOLD                     (sfx_s8)(-80) /* dBm */

#define RC4_OPEN_UPLINK_START_OF_TABLE        (sfx_u32)(902200000) /* Hz, The center frequency of RC4 is defined by the activated channels in config words */
#define RC4_OPEN_DOWNLINK_CENTER_FREQUENCY    (sfx_u32)(922300000) /* Hz */
#define RC4_MACRO_CHANNEL_WIDTH               (sfx_u32)(192000)    /* Hz */
#define RC4_UPLINK_MODULATION                 SFX_DBPSK_600BPS 
#define RC4_UPLINK_SPECTRUM_ACCESS            SFX_FH 
#define RC4_SET_STD_CONFIG_LM_WORD_0          (sfx_u32)0x00000000  /* LM = Long Message */
#define RC4_SET_STD_CONFIG_LM_WORD_1          (sfx_u32)0xF0000000
#define RC4_SET_STD_CONFIG_LM_WORD_2          (sfx_u32)0x0000001F
#define RC4_SET_STD_TIMER_ENABLE              (sfx_bool)(SFX_TRUE) /* Enable Timer for FH duty cycle*/
#define RC4_SET_STD_TIMER_DISABLE             (sfx_bool)(SFX_FALSE)/* Disable timer feature*/
#define RC4_SET_STD_CONFIG_SM_WORD_0          (sfx_u32)0x00000000  /* SM = Short message */
#define RC4_SET_STD_CONFIG_SM_WORD_1          (sfx_u32)0x40000000
#define RC4_SET_STD_CONFIG_SM_WORD_2          (sfx_u32)0x00000000

#define RC5_OPEN_CS_CENTER_FREQUENCY          (sfx_u32)(923300000) /* Hz */
#define RC5_OPEN_CS_BANDWIDTH                 (sfx_u32)(200000)    /* Hz */
#define RC5_OPEN_UPLINK_CENTER_FREQUENCY      (sfx_u32)(923250000) /* Hz */
#define RC5_OPEN_DOWNLINK_CENTER_FREQUENCY    (sfx_u32)(922250000) /* Hz */
#define RC5_MACRO_CHANNEL_WIDTH               (sfx_u32)(96000)     /* Hz */ 
#define RC5_UPLINK_MODULATION                 SFX_DBPSK_100BPS 
#define RC5_UPLINK_SPECTRUM_ACCESS            SFX_LBT 
#define RC5_CS_THRESHOLD                      (sfx_s8)(-65) /* dBm */

#define RC101_OPEN_UPLINK_CENTER_FREQUENCY    (sfx_u32)(68862500)  /* Hz */
#define RC101_OPEN_DOWNLINK_CENTER_FREQUENCY  (sfx_u32)(72912500)  /* Hz */
#define RC101_MACRO_CHANNEL_WIDTH             (sfx_u32)(12500)     /* Hz */
#define RC101_UPLINK_MODULATION               SFX_DBPSK_100BPS 
#define RC101_UPLINK_SPECTRUM_ACCESS          SFX_DC 


/* ---------------------------------------------- 
   IMPORTANT INFORMATION :
   ----------------------------------------------
   The SIGFOX Library needs to be opened with 
   one of the below configurations.
   ----------------------------------------------
*/
#define RC1   { RC1_OPEN_UPLINK_CENTER_FREQUENCY,   RC1_OPEN_DOWNLINK_CENTER_FREQUENCY,   RC1_MACRO_CHANNEL_WIDTH,   RC1_UPLINK_MODULATION,   RC1_UPLINK_SPECTRUM_ACCESS, {NA,NA,NA} }
#define RC2   { RC2_OPEN_UPLINK_START_OF_TABLE,     RC2_OPEN_DOWNLINK_CENTER_FREQUENCY,   RC2_MACRO_CHANNEL_WIDTH,   RC2_UPLINK_MODULATION,   RC2_UPLINK_SPECTRUM_ACCESS, {NA,NA,NA} }
#define RC3A  { RC3A_OPEN_UPLINK_CENTER_FREQUENCY,  RC3A_OPEN_DOWNLINK_CENTER_FREQUENCY,  RC3A_MACRO_CHANNEL_WIDTH,  RC3A_UPLINK_MODULATION,  RC3A_UPLINK_SPECTRUM_ACCESS, {RC3A_OPEN_CS_CENTER_FREQUENCY,RC3A_OPEN_CS_BANDWIDTH, RC3A_CS_THRESHOLD} }
#define RC3C  { RC3C_OPEN_UPLINK_CENTER_FREQUENCY,  RC3C_OPEN_DOWNLINK_CENTER_FREQUENCY,  RC3C_MACRO_CHANNEL_WIDTH,  RC3C_UPLINK_MODULATION,  RC3C_UPLINK_SPECTRUM_ACCESS, {RC3C_OPEN_CS_CENTER_FREQUENCY,RC3C_OPEN_CS_BANDWIDTH, RC3C_CS_THRESHOLD} }

#define RC4   { RC4_OPEN_UPLINK_START_OF_TABLE,     RC4_OPEN_DOWNLINK_CENTER_FREQUENCY,   RC4_MACRO_CHANNEL_WIDTH,   RC4_UPLINK_MODULATION,   RC4_UPLINK_SPECTRUM_ACCESS, {NA,NA,NA} }
#define RC5   { RC5_OPEN_UPLINK_CENTER_FREQUENCY,   RC5_OPEN_DOWNLINK_CENTER_FREQUENCY,   RC5_MACRO_CHANNEL_WIDTH,   RC5_UPLINK_MODULATION,   RC5_UPLINK_SPECTRUM_ACCESS, {RC5_OPEN_CS_CENTER_FREQUENCY,RC5_OPEN_CS_BANDWIDTH, RC5_CS_THRESHOLD} }

#define RC101 { RC101_OPEN_UPLINK_CENTER_FREQUENCY, RC101_OPEN_DOWNLINK_CENTER_FREQUENCY, RC101_MACRO_CHANNEL_WIDTH, RC101_UPLINK_MODULATION, RC101_UPLINK_SPECTRUM_ACCESS, {NA,NA,NA} }

#define RC2_LM_CONFIG  { RC2_SET_STD_CONFIG_LM_WORD_0, RC2_SET_STD_CONFIG_LM_WORD_1, RC2_SET_STD_CONFIG_LM_WORD_2 }   /*!< Config for full RC2 hopping */
#define RC4_LM_CONFIG  { RC4_SET_STD_CONFIG_LM_WORD_0, RC4_SET_STD_CONFIG_LM_WORD_1, RC4_SET_STD_CONFIG_LM_WORD_2 }   /*!< Config for full RC4 hopping */

#define RC2_SM_CONFIG  { RC2_SET_STD_CONFIG_SM_WORD_0, RC2_SET_STD_CONFIG_SM_WORD_1, RC2_SET_STD_CONFIG_SM_WORD_2 }   /*!< Config for normal RC2 operations */
#define RC4_SM_CONFIG  { RC4_SET_STD_CONFIG_SM_WORD_0, RC4_SET_STD_CONFIG_SM_WORD_1, RC4_SET_STD_CONFIG_SM_WORD_2 }   /*!< Config for normal RC4 operations */

#define RC3A_CONFIG    {0x00000003,0x00001388,0x00000000}  /*!< Config word default value: 3 retries before 1st frame , 0x1288=5000ms max time between each frame>*/
#define RC3C_CONFIG    {0x00000003,0x00001388,0x00000000}  /*!< Config word default value: 3 retries before 1st frame , 0x1288=5000ms max time between each frame>*/
#define RC5_CONFIG     {0x00000003,0x00001388,0x00000000}  /*!< Config word default value: 3 retries before 1st frame , 0x1288=5000ms max time between each frame>*/


#define ID_LENGTH      (sfx_u8)(4)            /* Size of device identifier */
#define PAC_LENGTH     (sfx_u8)(8)            /* Size of device initial PAC */

/********************************
 * \enum sfx_spectrum_access_t
 * \brief Data type for Spectrum Access
 * Define as bit mask value so that we can combine
 * them if needed (not yet implemented)
 *******************************/
typedef enum
{
    SFX_FH   = 1,                             /*!< Index of Frequency Hopping */ 
    SFX_LBT  = 2,                             /*!< Index of Listen Before Talk */
    SFX_DC   = 4,                             /*!< Index of Duty Cycle */
} sfx_spectrum_access_t;

/********************************
 * \enum sfx_version_type_t
 * \brief Enum to be used in SIGFOX_API_get_version 
 *******************************/
typedef enum
{
    VERSION_SIGFOX    = 0,                    /*!< Sigfox Version */
    VERSION_MCU       = 1,                    /*!< MCU Version */
    VERSION_RF        = 2,                    /*!< RF Version */
    VERSION_MONARCH   = 5,                    /*!< MONARCH Version */
    VERSION_DEVICE_CONFIG = 6,                /*!< DEVICE CONFIG */

} sfx_version_type_t;

typedef enum
{
    AUTHENTICATION_OFF   = 0,                 /*!< No authentication of the Received frame, but check a specific pattern */ 
    AUTHENTICATION_ON    = 1,                 /*!< Authentication of the Received frame */ 
} sfx_authentication_mode_t;

typedef enum
{
    DL_TIMEOUT        = 0,
    DL_PASSED         = 1,
} sfx_rx_state_enum_t;

/********************************
 * \enum sfx_oob_enum_t 
 * \brief This enum contains all the OOB frame types
 * that can be send by the user application 
 *******************************/
typedef enum
{
    SFX_OOB_SERVICE = 0,
    SFX_OOB_RC_SYNC,
    SFX_MAX_OOB_LIST_SIZE,
} sfx_oob_enum_t;

/********************************
 * \enum sfx_state_t
 * \brief State Machine constants
 * The state machine will never
 * returns automatically to SFX_STATE_READY
 * if an error has occured during
 * frame transmissions
 *******************************/
typedef enum
{
    SFX_STATE_IDLE             = 0,           /*!< Uninitialized */
    SFX_STATE_NOT_CONFIGURED   = 1,           /*!< Not Configured */
    SFX_STATE_READY            = 2,           /*!< Initialized */
    SFX_STATE_UPLINK           = 3,           /*!< Uplink preparation on going */
    SFX_STATE_DOWNLINK         = 4,           /*!< Downlink preparation on going */
    SFX_STATE_MONARCH_LISTENING_SWEEP   = 6,  /*!< Monarch Listening Sweep on going */
    SFX_STATE_MONARCH_LISTENING_WINDOW  = 7,  /*!< Monarch Listening Window on going */
} sfx_state_t;

/********************************
 * \enum sfx_modulation_type_t
 * \brief Uplink Modulation type with baudrate 
 *******************************/
typedef enum
{
    SFX_NO_MODULATION = 0,                    /*!< Do not set the modulation : signal is a pure carrier  */
    SFX_DBPSK_100BPS  = 1,                    /*!< Set DBPSK Modulation with 100 bps baudrate */
    SFX_DBPSK_600BPS  = 2,                    /*!< Set DBPSK Modulation with 600 bps baudrate */
} sfx_modulation_type_t;

/********************************
 * \enum sfx_nvmem_t
 * \brief Data type for Nv memory access
 * Saving PN and Sequence are mandatory
 * for backend compatibility
 *******************************/
/* FH information required, seq_num stored in nv_mem */
#if ( ITSDK_SIGFOX_EXTENSIONS & __SIGFOX_MONARCH) > 0
typedef enum
{
    SFX_NVMEM_PN         = 0,                 /*!< Index of nv memory for PN */
    SFX_NVMEM_SEQ_NUM    = 2,                 /*!< Index of nv memory for Sequence Number */
    SFX_NVMEM_FH         = 4,                 /*!< Index of nv memory for dedicated FH information */
    SFX_NVMEM_RL         = 6,                 /*!< Index of nv memory for rollover counter */
    SFX_NVMEM_BLOCK_SIZE = 7,
} sfx_nvmem_t;
#else
typedef enum
{
    SFX_NVMEM_PN         = 0,                 /*!< Index of nv memory for PN */
    SFX_NVMEM_FH         = 2,                 /*!< Index of nv memory for dedicated FH information */
    SFX_NVMEM_BLOCK_SIZE = 4,
} sfx_nvmem_t;
#endif

/********************************
 * \enum sfx_rf_mode_t
 * \brief Functionnal mode for RF chip
 *******************************/
typedef enum
{
    SFX_RF_MODE_TX = 0,                       /*!< Set RF chip as transmitter */
    SFX_RF_MODE_RX = 1,                       /*!< Set RF chip as receiver */
    SFX_RF_MODE_CS200K_RX = 2,                /*!< Set RF chip as receiver for Carrier Sense on 200KHz */
    SFX_RF_MODE_CS300K_RX = 3,                /*!< Set RF chip as receiver for Carrier Sense on 300KHz */
    SFX_RF_MODE_MONARCH   = 4,                /*!< Set RF chip as Monarch Configuration */              
} sfx_rf_mode_t;

/********************************
 * \enum sfx_delay_t
 * \brief Delay type
 *******************************/
typedef enum
{
    SFX_DLY_INTER_FRAME_TRX = 0,              /*!< Delay inter frames in TX/RX (send frame with initiate_downlink_flag = SFX_TRUE) + FH Uplink : (500ms) */
    SFX_DLY_INTER_FRAME_TX  = 1,              /*!< Delay inter frames in TX only (0-2000ms) */
    SFX_DLY_OOB_ACK         = 2,              /*!< Delay between frame reception and send followed out of band message (1400ms-4000ms) */
    SFX_DLY_CS_SLEEP        = 3,              /*!< Delay between attempts of carrier sense for the first frame */
} sfx_delay_t;

/********************************
 * \enum sfx_credentials_use_key_t
 * \brief Key to use for MANUF_API_aes_128_cbc_encrypt()
 *******************************/
typedef enum
{
    CREDENTIALS_PRIVATE_KEY = 0,
    CREDENTIALS_KEY_IN_ARGUMENT,
} sfx_credentials_use_key_t;

/********************************
 * \struct sfx_rc_specific_t
 * \brief Radio configuration substruture for specific radio configuration
 * All parameters requested to define
 * radio configuration subsctructure.
 *******************************/
typedef struct sfx_rc_specific_t
{
    sfx_u32 open_cs_frequency;                /*!< carrier sense center frequency : can be equal to uplink center frequency */
    sfx_u32 open_cs_bandwidth;                /*!< carrier sense bandwidth to apply carrier sensing */
    sfx_s8  cs_threshold;                     /*!< LBT threshold defined in the standards related to the RC */
} sfx_rc_specific_t;

/********************************
 * \struct sfx_rc_t
 * \brief Radio configuration structure
 * All parameters requested to define
 * radio configuration.
 *******************************/
typedef struct sfx_rc_t
{
    sfx_u32 open_tx_frequency;                /*!< Uplink frequency (Hz) used to open the library
                                                   This is not necessary the Transmitter center frequency in Hz
                                                   as it may depends on the values set in Config Words */

    sfx_u32 open_rx_frequency;                /*!< Downlink frequency (Hz) used to open the library */
    sfx_u32 macro_channel_width;              /*!< Macro channel = SIGFOX Operational radio band */
    sfx_modulation_type_t modulation;         /*!< Uplink modulation and baudrate */
    sfx_spectrum_access_t spectrum_access;    /*!< Spectrum access : can be Duty Cycle, Frequency Hopping or Listen Before Talk */
    sfx_rc_specific_t specific_rc;            /*!< Specific radio conf for LBT feature */
} sfx_rc_t;

/****************************************/
/*          Sigfox Library API          */
/****************************************/

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_open(sfx_rc_t *rc)
 * \brief This function initialises library (mandatory). The SIGFOX_API_open function will :
 *  - Allocate memory for library
 *  - Save the input parameters once (Can't be changed till SIGFOX_API_close call)
 *  - Read the non volatile memory content
 *  - Set the global state to SFX_STATE_READY
 *
 * \param[in] sfx_rc_t *rc                      Pointer on the Radio Configuration Zone: it is mandatory 
 *                                              to use already existing RCx define.
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_API_OPEN_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_open(sfx_rc_t *rc);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_close(void)
 * \brief This function closes the library (Free the allocated memory
 * of SIGFOX_API_open and close RF)
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_API_CLOSE_xx
 * 
 *******************************************************************/
sfx_error_t SIGFOX_API_close(void);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_send_frame(sfx_u8 *customer_data, sfx_u8 customer_data_length, sfx_u8 *customer_response, sfx_u8 tx_repeat, sfx_bool initiate_downlink_flag)
 * \brief Send a standard SIGFOX frame with customer payload. Customer
 * payload cannot exceed 12 Bytes.<BR>
 * If initiate_downlink_flag is set, the frame will be send as
 * many times as (tx_repeat + 1). tx repeat cannot exceed 2.<BR>
 * If initiate_downlink_flag is unset, the tx_repeat is forced to 2.
 *  - In downlink :
 *      * Send uplink frames (1 to 3)
 *      * Receive downlink frame
 *      * Send out of band frame (Voltage, temperature and RSSI)
 *      .
 *  - In uplink :
 *      * Send uplink frames (3)
 *      .
 * \param[in] sfx_u8 *customer_data               Data to transmit
 * \param[in] sfx_u8 customer_data_length         Data length in Bytes
 * \param[out] sfx_u8 *customer_response          Returned 8 Bytes data in case of downlink
 * \param[in] sfx_u8 tx_repeat                    Number of repetition (value between 1 and 3 included - for downlink only)
 * \param[in] sfx_bool initiate_downlink_flag     Flag to initiate a downlink response
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_API_SEND_FRAME_xx  or  SFX_ERR_INT_xx
 * 
 *******************************************************************/
sfx_error_t SIGFOX_API_send_frame(sfx_u8 *customer_data,
                                  sfx_u8 customer_data_length,
                                  sfx_u8 *customer_response,
                                  sfx_u8 tx_repeat,
                                  sfx_bool initiate_downlink_flag);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_send_bit(sfx_bool bit_value, sfx_u8 *customer_response, sfx_u8 tx_repeat, sfx_bool initiate_downlink_flag)
 * \brief Send a standard SIGFOX frame with null customer payload.
 * This frame is the shortest that SIGFOX library can generate.
 * Data is contained on 1 sfx_bool
 * If initiate_downlink_flag is set, the frame will be send as
 * many times as (tx_repeat + 1). tx repeat cannot exceed 2. If
 * initiate_downlink_flag is unset, the tx_repeat is forced to 2.
 *  - In downlink :
 *      * Send uplink frames (1 to 3)
 *      * Receive downlink frame
 *      * Send out of band frame (Voltage, temperature and RSSI)
 *      .
 *  - In uplink :
 *      * Send uplink frames (3)
 *      .
 *  .
 *
 * \param[in] sfx_bool bit_value                Bit state (SFX_TRUE or SFX_FALSE)
 * \param[out] sfx_u8 *customer_response        Returned 8 Bytes data in case of downlink
 * \param[in] sfx_u8 tx_repeat                  Number of repetition (downlink only)
 * \param[in] sfx_bool initiate_downlink_flag   Flag to initiate a downlink response
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_API_SEND_BIT_xx  or  SFX_ERR_INT_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_send_bit(sfx_bool bit_value,
                                sfx_u8 *customer_response,
                                sfx_u8 tx_repeat,
                                sfx_bool initiate_downlink_flag);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_send_outofband(sfx_oob_enum_t oob_type)
 * \brief Send an out of band SIGFOX frame which the type is passed as parameter
 * of the function.<BR>
 * Data is composed of information about the chip itself (Voltage,
 * Temperature).<BR>
 *  - In uplink :
 *      * Send uplink frames (3)
 *      .
 *  .
 * This function must be called by application every 24 hours maximum
 * or never if application has some energy critical constraints with 
 * the SFX_OOB_SERVICE enum value
 *
 * If Payload encryption is supported and activated, the user can
 * synchronize the device and the backend with sending SFX_OOB_RC_SYNC
 *
 * In case REPEATER feature is present, the enum SFX_OOB_REPEATER_STATUS
 * can be sent. This frame contains the counters related to the repeater operations.
 * The counters are part of a shared memory between the Application
 * and the Sigfox Library.
 *.
 *
 * \param[in] sfx_oob_enum_t oob_type    Type of the OOB frame to send 
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_INT_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_send_outofband(sfx_oob_enum_t oob_type);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_set_std_config(sfx_u32 config_words[3], sfx_bool timer_enable)
 * \brief This function must be used to configure specific variables for standard.
 *        It is mandatory to call this function after SIGFOX_API_open() for FH and LBT.
 *
 * <B> FH (Frequency Hopping )</B>: config words to enable/disable 192KHz macro channels authorized for
 * transmission.<BR>Each macro channel is separated from another of 300 kHz<BR>
 * At least 9 macro channel must be enabled to ensure the
 * minimum of 50 FCC channels (9*6 = 54).<BR> 
 * <B>WARNING : This function should be called each time you open the library
 * or your FCC configuration will not be applied</B><BR>
 *
 * Use the RC defined above to configure the macro channels
 *
 * <B>The example below shows you the Long Message configuration
 * <B>Example :</B> To enable Macro channel 1 to 9, that is to say 902.2MHz
 * to 904.6MHz with 902.2MHz as main Macro channel, you must set :<BR>
 * <B>config_words[0]</B> = [0x000001FF]<BR>
 * <B>config_words[1]</B> = [0x00000000]<BR>
 * <B>config_words[2]</B> = [0x00000000]<BR>
 *
 * \verbatim
   Macro Channel Value MHz : | 902.2MHz | 902.5MHz | 902.8MHz | 903.1MHz | 903.4MHz | 903.7MHz | 904.0MHz | 904.3MHz | 904.6MHz | 904.9MHz | 905.2MHz | ...     ...      | 911.5MHz |
   Macro Channel Value     : | Chn 1    | Chn 2    | Chn 3    | Chn 4    | Chn 5    | Chn 6    | Chn 7    | Chn 8    | Chn 9    | Chn 10   | Chn 11   | ...     ...      | Chn 32   |
   config_words[0] bit     : | bit 0    | bit 1    | bit 2    | bit 3    | bit 4    | bit 5    | bit 6    | bit 7    | bit 8    | bit 9    | bit 10   | ...     ...      | bit 31   |

   Macro Channel Value MHz : | 911.8MHz | 912.1MHz | 912.4MHz | 912.7MHz | 913.0MHz | 913.3MHz | 913.6MHz | 913.9MHz | 914.2MHz | 914.5MHz | 914.8MHz | ...     ...      | 921.1MHz |
   Macro Channel Value     : | Chn 33   | Chn 34   | Chn 35   | Chn 36   | Chn 37   | Chn 38   | Chn 39   | Chn 40   | Chn 41   | Chn 42   | Chn 43   | ...     ...      | Chn 64   |
   config_words[1] bit     : | bit 0    | bit 1    | bit 2    | bit 3    | bit 4    | bit 5    | bit 6    | bit 7    | bit 8    | bit 9    | bit 10   | ...     ...      | bit 31   |

   Macro Channel Value MHz : | 921.4MHz | 921.7MHz | 922.0MHz | 922.3MHz | 922.6MHz | 922.9MHz | 923.2MHz | 923.5MHz | 923.8MHz | 924.1MHz | 924.4MHz | ... | 927.7MHz |
   Macro Channel Value     : | Chn 65   | Chn 66   | Chn 67   | Chn 68   | Chn 69   | Chn 70   | Chn 71   | Chn 72   | Chn 73   | Chn 74   | Chn 75   | ... | Chn 86   |
   config_words[2] bit     : | bit 0    | bit 1    | bit 2    | bit 3    | bit 4    | bit 5    | bit 6    | bit 7    | bit 8    | bit 9    | bit 10   | ... | bit 21   |
   \endverbatim
 *
 * <B>DC (Duty Cycle)</B>: This function has no effect in DC spectrum access ( used for the ETSI standard ).</B><BR>
 * 
 * <B>LBT (Listen Before Talk)</B> : Carrier Sense feature for the First frame can be configured.
 *           - config_word[0] : number of attempts to send the first frame [ has to be greater or equal to 1]
 *           - config_word[1] : maximum carrier sense sliding window (in ms) [ has to be greater than 6 ms ( CS_MIN_DURATION_IN_MS + 1 ) ]
 *           - config_word[2] :
 *                  . bit 8   : set the value to 1 to indicate that the device will use the full operationnal radio band.( 192kHz )
 *                  . bit 7-3 : number of Carrier Sense attempts.
 *                  . bit 2-0 : number of frames sent.
 *           - timer_enable : unused
 * <BR><BR>
 * The delay between several attempts of Carrier Sense for the first frame is set by SFX_DLY_CS_SLEEP
 *
 * \param[in] sfx_u32 config_words[3]           Meaning depends on the standard (as explained above)
 * \param[in] sfx_bool timer_enable             Enable timer feature for FH
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_API_SET_CONFIG_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_set_std_config(sfx_u32 config_words[3],
                                      sfx_bool timer_enable);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_start_continuous_transmission(sfx_u32 frequency, sfx_modulation_type_t type);
 * \brief Executes a continuous wave or modulation depending on the parameter type 
 *        SIGFOX_API_stop_continuous_transmission has to be called to stop the continuous transmission.           
 *
 * \param[in] sfx_u32 frequency                Frequency at which the signal has to be generated
 * \param[in] sfx_modulation_type_t type       Type of modulation to use in continuous mode.
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_API_START_CONTINUOUS_TRANSMISSION_xx
 *******************************************************************/
sfx_error_t SIGFOX_API_start_continuous_transmission ( sfx_u32 frequency, sfx_modulation_type_t type);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_stop_continuous_transmission(void);
 * \brief Stop the current continuous transmission
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_API_STOP_CONTINUOUS_TRANSMISSION_xx
 *******************************************************************/
sfx_error_t SIGFOX_API_stop_continuous_transmission (void);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_send_test_frame(sfx_u32 frequency, sfx_u8 * customer_data, sfx_u8 customer_data_length, sfx_bool initiate_downlink_flag )
 * Send only 1 repetition
 *
 * \brief This function builds a Sigfox Frame with the customer payload and send it at a specific frequency 
 *
 *
 * \param[in] sfx_u32 frequency                 Frequency at which the wave is generated
 * \param[in] sfx_u8 *customer_data             Data to transmit
 * \param[in] sfx_u8 customer_data_length       Data length in Bytes
 * \param[in] sfx_bool initiate_downlink_flag   Flag to initiate a downlink response
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_API_SEND_TEST_FRAME_xx
 *******************************************************************/
sfx_error_t SIGFOX_API_send_test_frame(sfx_u32 frequency, sfx_u8 *customer_data, sfx_u8 customer_data_length, sfx_bool initiate_downlink_flag);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_receive_test_frame( sfx_u32 frequency, sfx_authentication_mode_t mode, sfx_u8 * buffer, sfx_u8 timeout, sfx_s16 * rssi  );

 * \brief This function waits for a valid downlink frame during timeout time and return in customer_data the data received.
 *
 *
 * \param[in] sfx_u32 frequency                 Frequency at which the wave is generated
 * \param[in] sfx_authentication_mode_t         Mode ( AUTHENTICATION_ON or AUTHENTICATION_OFF)
 * \param[in/out] buffer                        Depends of the Authentication mode : 
 *                                               - if AUTHENTICATION_OFF : buffer is used as input to check the bit stream of the received frame
 *                                               - if AUTHENTICATION_ON  : buffer is used as output to get the received Payload
 * \param[in] sfx_u8 timeout                    Timeout for the reception of a valid downlink frame
 * \param[in] sfx_s16 * rssi                    RSSI of the received frame ( only valid for AUTHENTICATION_ON as in AUTHENTICATION_OFF, the rssi of the frames received
 *                                              are returned through the MCU_API_report_test_result function) 
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and SFX_ERR_API_RECEIVE_TEST_FRAME_xx
 *******************************************************************/
sfx_error_t SIGFOX_API_receive_test_frame( sfx_u32 frequency, sfx_authentication_mode_t mode, sfx_u8 * buffer, sfx_u8 timeout, sfx_s16 * rssi  );

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_get_version(sfx_u8 **version, sfx_u8 *size, sfx_version_type_t type)
 * \brief Returns current SIGFOX library version, or RF Version, or MCU version etc ..., in ASCII format ( depending on the type )
 *
 * \param[out] sfx_u8 **version                 Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                     Size of the byte array pointed by *version
 * \param[in]  sfx_version_type_t type          Type of the version ( MCU, RF, ... ) 
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE and  SFX_ERR_API_GET_VERSION_xx
 *******************************************************************/
sfx_error_t SIGFOX_API_get_version(sfx_u8 **version, sfx_u8 *size, sfx_version_type_t type);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_get_info(sfx_u8* returned_info);
 * \brief This function is to return info on send frame depending on
 * the mode you're using.<BR>
 * <B> In DC  :</B> returned_info is always 0.<BR>
 * <B> In FH  :</B> returned_info[bit 3 - 0] = 1 when the current FCC marco channel
 * is NOT the default_sigfox_channel set in SIGFOX_API_set_std_conf.<BR>
 * returned_info[bit 7 - 4] = number of free micro channel in current FCC
 * macro channel.<BR>
 * <B> In LBT :</B> returned_info = bit[7-3]: Carrier Sense attempts
 * and bit[2-0]: Number of frames sent 
 *
 * \param[out] sfx_u8* returned_info            Returned value by library
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_get_info(sfx_u8* returned_info);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_get_device_id(sfx_u8 *dev_id)
 * \brief This function copies the ID of the device to the pointer given in parameter.
 * The ID is \link ID_LENGTH \endlink bytes length and is in binary format.
 *
 * \param[in] none
 * \param[out] sfx_u8* dev_id                   Pointer where to write the device ID
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE or SFX_ERR_API_GET_DEVICE_ID_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_get_device_id(sfx_u8 *dev_id);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_get_initial_pac(sfx_u8 *initial_pac);
 * \brief Get the value of the PAC stored in the device. This value is
 * used when the device is registered for the first time on the backend.
 *
 * \param[in] none
 * \param[out] sfx_u8* initial_pac              Pointer to initial PAC
 *
 * \retval  The sfx_error_t is composed of the following :
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE or SFX_ERR_API_GET_INITIAL_PAC_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_get_initial_pac(sfx_u8 *initial_pac);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_switch_public_key(sfx_bool use_public_key)
 * \brief Switch device on public or private key.
 *
 * \param[in] sfx_bool use_public_key        Switch to public key if SFX_TRUE, private key else
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE or SFX_ERR_API_SWITCH_PUBLIC_KEY_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_switch_public_key(sfx_bool use_public_key);

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_API_set_rc_sync_period(sfx_u16 rc_sync_period)
 * \brief Set the period for transmission of RC Sync frame
 * By default, when payload is encrypted, a RC Sync frame is transmitted by the device
 * every 4096 messages transmissions (ie. when the sequence number loops to 0) to
 * 're-synchronize' the device with the backend (from a payload encryption point of view).
 * This transmission period could be reduced through this function. Then, a RC Sync frame
 * will be transmitted every 'rc_sync_period' transmissions of a 'normal' frame.
 * The value 0 corresponds to the default behavior, ie a RC Sync frame transmitted every
 * 4096 messages transmissions.
 * As sequence number is on 12 bits, setting a rc_sync_period with a value more than (4095 - 1)
 * will return an error.
 *
 * \param[in] sfx_u16 rollover_counter_period    Transmission period of the RC Sync frame (in number of 'normal' frames)
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE or SFX_ERR_API_SET_RC_SYNC_PERIOD_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_API_set_rc_sync_period(sfx_u16 rc_sync_period);

#endif // IT_SDK_DRIVERS_SIGFOX_API_H_
