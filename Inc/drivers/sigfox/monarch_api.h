
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
 * \file monarch_api.h
 * \brief Sigfox manufacturer functions
 * \author $(SIGFOX_LIB_AUTHOR)
 * \version $(SIGFOX_LIB_VERSION)
 * \date $(SIGFOX_LIB_DATE)
 * \copyright Copyright (c) 2011-2015 SIGFOX, All Rights Reserved. This is unpublished proprietary source code of SIGFOX.
 *
 * This file defines the manufacturer's MONARCH functions to be implemented
 * for library usage.
 */

/********************************************************
* External API dependencies to link with this library.
*
* Error codes of the MCU MONARCH API functions are described below.
* The Manufacturer can add more error code taking care of the limits defined.
*
********************************************************/

/*!
 * \defgroup MONARCH_ERR_API_xx codes Return Error codes definition for MCU MONARCH API
 *
 * \brief Can be customized to add new error codes.
 * All MONARCH_API_ error codes will be piped with SIGFOX_MONARCH_API_xxx return code.<BR>
 *
 * IMPORTANT : SFX_ERR_NONE return code is mandatory when no error for each MCU_API_xxx MONARCH_API_xxx RF_API_xxx REPEATER_API_xxx or SE_API_xxx
 * functions.
 *
 *  @{
 */

#include "sigfox_types.h"
#include "sigfox_monarch_api.h"

/*
 * ----------------------------------------------------------------
 * Bytes reserved for MCU API ERROR CODES : From 0x10 to 0x2F
 * ----------------------------------------------------------------
 */

/*
 * ----------------------------------------------------------------
 * Bytes reserved for RF API ERROR CODES : From 0x30 to 0x3F
 * ----------------------------------------------------------------
 */

/*
 * ----------------------------------------------------------------
 * Bytes reserved for SE API ERROR CODES : From 0x40 to 0x5F
 * ----------------------------------------------------------------
 */

/*
 * ----------------------------------------------------------------
 * Bytes reserved for REPEATER API ERROR CODES : From 0x60 to 0x7F
 * ----------------------------------------------------------------
 */

/*
 * ----------------------------------------------------------------
 * Bytes reserved for MONARCH API ERROR CODES : From 0x80 to 0x8F
 * ----------------------------------------------------------------
 */
#define MONARCH_ERR_API_MALLOC                   (sfx_u8)(0x80) /*!< Error on MONARCH_API_malloc */
#define MONARCH_ERR_API_FREE                     (sfx_u8)(0x81) /*!< Error on MONARCH_API_free */
#define MONARCH_ERR_API_TIMER_START              (sfx_u8)(0x82) /*!< Error on MONARCH_API_timer_start */
#define MONARCH_ERR_API_TIMER_STOP               (sfx_u8)(0x83) /*!< Error on MONARCH_API_timer_stop */
#define MONARCH_ERR_API_CONFIGURE_SEARCH_PATTERN (sfx_u8)(0x84) /*!< Error on MONARCH_API_configure_search_pattern */
#define MONARCH_ERR_API_STOP_SEARCH_PATTERN      (sfx_u8)(0x85) /*!< Error on MONARCH_API_stop_search_pattern */
#define MONARCH_ERR_API_GET_VERSION              (sfx_u8)(0x86) /*!< Error on MONARCH_API_get_version */

/** @}*/

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
 * \brief Allocate memory for MONARCH library usage (Memory usage = size (Bytes))
 * This function is only called once at RC Scan.
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
 * \retval SFX_ERR_NONE:                 No error
 * \retval MONARCH_ERR_API_MALLOC        Malloc error
 *******************************************************************/
sfx_u8 MONARCH_API_malloc(sfx_u16 size, sfx_u8** returned_pointer);

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_free(sfx_u8 *ptr)
 * \brief Free memory allocated to library with the MONARCH_API_malloc
 *
 * \param[in] sfx_u8 *ptr                        pointer to buffer
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MONARCH_ERR_API_FREE:                 Free error
 *******************************************************************/
sfx_u8 MONARCH_API_free(sfx_u8* ptr);

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_timer_start (sfx_u16 timer_value, sfx_timer_unit_enum_t unit, sfx_error_t  (* sigfox_callback_timeout_handler ) ( void ) )
 * \brief This function starts a timer based on timer_value and the units. When the timer expires
 *        the manufacturer has to call the timer callback function (sigfox_callback_timeout_handler)
 *
 * \param[in] sfx_u16 timer_value               Scan duration value ( with the unit parameter information )
 * \param[in] sfx_timer_unit_enum_t  unit       Unit to be considered for the scan time computation
 * \param[in] timeout_callback_handler          This is the function that needs to be called when the timer expires
 *                                              ( either when RC Found or when Timeout )
 *
 * \retval SFX_ERR_NONE:                             No error
 * \retval MONARCH_ERR_API_TIMER_START:              Timer Start error
 *******************************************************************/
sfx_u8 MONARCH_API_timer_start(sfx_u16 timer_value, sfx_timer_unit_enum_t unit, sfx_error_t (* timeout_callback_handler)(void));

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_timer_stop (void)
 * \brief This function stops the timer
 *
 * \retval SFX_ERR_NONE:                              No error
 * \retval MONARCH_ERR_API_TIMER_STOP:                Stop Timer error
 *******************************************************************/
sfx_u8 MONARCH_API_timer_stop(void);

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_configure_search_pattern ( sfx_monarch_pattern_search_t list_freq_pattern[], sfx_u8 size, sfx_monarch_listening_mode_t mode, sfx_error_t  (* monarch_pattern_freq_result_callback_handler ) (sfx_u32 freq, sfx_pattern_enum_t pattern, sfx_s16 rssi));
 *
 * \brief This function is used to configure a search pattern action from on the MCU/RF side.
 *        The list of frequencies to scan and associated pattern is given as parameter.
 *        When a pattern is found or the timeout ( set with MONARCH_API_timer_start ) occurs,
 *        the callback function has to be called.
 *
 *        There are 2 modes that can be used : LISTENING_SWEEP and LISTENING_WINDOW modes.
 *        They are detailed in the sfx_monarch_listening_mode_t description
 *
 * \param[in] sfx_monarch_pattern_search_t list_freq_pattern[]       Tab of the frequencies / patterns to check
 * \param[in] sfx_u8  size                                      Size of the list (list_freq_pattern)
 * \param[in] sfx_monarch_listening_mode_t  mode                Mode of the scan
 * \param[in] monarch_pattern_freq_result_callback_handler      Callback function when a pattern is found or when the timer expires
 *            with the following parameters :
 *                \param[in] sfx_u32 freq                       Report the Frequency on which the pattern has been found
 *                \param[in] sfx_pattern_enum_t pattern         Pattern which has been found
 *                \param[in] sfx_s16 rssi                       RSSI level of the found pattern
 *
 *
 * \retval SFX_ERR_NONE:                                  No error
 * \retval MONARCH_ERR_API_CONFIGURE_SEARCH_PATTERN:      Search pattern error
 *******************************************************************/
sfx_u8 MONARCH_API_configure_search_pattern(sfx_monarch_pattern_search_t list_freq_pattern[],
                                            sfx_u8 size,
                                            sfx_monarch_listening_mode_t mode,
                                            sfx_error_t (* monarch_pattern_freq_result_callback_handler)(sfx_u32 freq, sfx_pattern_enum_t pattern, sfx_s16 rssi));

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_stop_search_pattern (void)
 * \brief This function stops the scan
 *
 * \retval SFX_ERR_NONE:                              No error
 * \retval MONARCH_ERR_API_STOP_SEARCH_PATTERN:       Stop seach pattern error
 *******************************************************************/
sfx_u8 MONARCH_API_stop_search_pattern(void);

/*!******************************************************************
 * \fn sfx_u8 MONARCH_API_get_version(sfx_u8 **version, sfx_u8 *size)
 * \brief This function returns current MONARCH API version
 *
 * \param[out] sfx_u8 **version                  Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                      Size of the byte array pointed by *version
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MONARCH_ERR_API_GET_VERSION:          Get Version error
 *******************************************************************/
sfx_u8 MONARCH_API_get_version(sfx_u8** version, sfx_u8* size);

