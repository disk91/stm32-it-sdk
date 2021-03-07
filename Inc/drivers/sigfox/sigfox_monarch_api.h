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
 * \file sigfox_monarch_api.h
 * \brief Sigfox user functions
 * \author $(SIGFOX_LIB_AUTHOR)
 * \version $(SIGFOX_LIB_VERSION)
 * \date $(SIGFOX_LIB_DATE)
 * \copyright Copyright (c) 2011-2018 SIGFOX, All Rights Reserved. This is unpublished proprietary source code of SIGFOX.
 *
 * This file includes the user's functions for the Monarch feature allowing to identify the coverage of the Radio Configuration
 * based on beacon sent by the network.
 */

#ifndef SIGFOX_MONARCH_API_H
#define SIGFOX_MONARCH_API_H

#include "sigfox_types.h"

/********************************
 * \enum sfx_timer_unit_enum_t
 * \brief value used to identify the unit of the timer
 *******************************/
typedef enum
{
    SFX_TIME_MS = 0, /*!< Millisecond unit */
    SFX_TIME_S, /*!< Second unit */
    SFX_TIME_M, /*!< Minute unit */
    SFX_TIME_H, /*!< Hour unit */
}sfx_timer_unit_enum_t;

/********************************
 * \enum sfx_bitfield_rc_enum_t
 * \brief value used to identify the RC supported by the device
 *        or the RC found after the monarch scan.
 *******************************/
typedef enum
{
    SFX_BITFIELD_SHIFT_RC1 = 0, /*!< RC1 bitfield shift value */
    SFX_BITFIELD_SHIFT_RC2 = 1, /*!< RC2 bitfield shift value */
    SFX_BITFIELD_SHIFT_RC3 = 2, /*!< RC3 bitfield shift value */
    SFX_BITFIELD_SHIFT_RC4 = 3, /*!< RC4 bitfield shift value */
    SFX_BITFIELD_SHIFT_RC5 = 4, /*!< RC5 bitfield shift value */
    SFX_BITFIELD_SHIFT_RC6 = 5, /*!< RC6 bitfield shift value */
    SFX_BITFIELD_SHIFT_RC7 = 6, /*!< RC7 bitfield shift value */

    /* Keep the below line still at the end */
    SFX_MAX_SIGFOX_RC, /*!< Maximum number of RC supported by Sigfox */

}sfx_bitfield_rc_enum_t;

/********************************
 * \enum sfx_pattern_enum_t
 * \brief This enum describes all the
 *        patterns that can be part of the
 *        Sigfox beacon, in first or second
 *        position of the beacon.
 *******************************/
typedef enum
{
    NO_MONARCH_PATTERN = 0, /*!< No pattern defined                */
    MONARCH_DELTA_F1   = 1, /*!< Delta F1 pattern :        1024 Hz */
    MONARCH_DELTA_F2   = 2, /*!< Delta F2 pattern :   1260.3077 Hz */
    MONARCH_DELTA_F3   = 3, /*!< Delta F3 pattern : 1489.454545 Hz */

    /* Keep this line at the end */
    LAST_MONARCH_PATTERN = MONARCH_DELTA_F3, /*!< Last supported pattern */
}sfx_pattern_enum_t;

#define MAX_MONARCH_PATTERN_PER_FREQUENCY_SEARCH 2 /* On 1 frequency, we can at the maximum ask for the research of 2 different patterns */

/********************************
 * \struct sfx_monarch_pattern_search_t
 * \brief This structure describes
 *        for a single frequency, the patterns
 *        that need to be searched
 *******************************/
typedef struct
{
    sfx_u32 freq;
    sfx_pattern_enum_t patterns[MAX_MONARCH_PATTERN_PER_FREQUENCY_SEARCH];
}sfx_monarch_pattern_search_t;

/********************************
 * \enum sfx_monarch_listening_mode_t
 * \brief value used to define the monarch
 *        listening modes.
 *******************************/
typedef enum
{
    MONARCH_LISTENING_SWEEP, /*!< Sweep mode is used to scan all the frequencies given by the Sigfox Lib
                              *   In that mode, when the check on pattern (or patterns) is identify as negative,
                              *   it has to go to the next frequency and continue to search for the
                              *   patterns of that frequency */

    MONARCH_LISTENING_WINDOW, /*!< Listening Window mode is used to scan the frequency during a specific time.
                               *   Check on one or two pattern can be executed till the end of the window. */

}sfx_monarch_listening_mode_t;

/*!******************************************************************
 * \fn sfx_error_t sfx_error_t SIGFOX_MONARCH_API_execute_rc_scan (sfx_u8 rc_capabilities_bit_mask, sfx_u16 timer, sfx_timer_unit_enum_t unit, sfx_u8  (* app_callback_handler ) ( sfx_u8 rc_bit_mask, sfx_s16 rssi ) )
 * \brief This function executes a scan of the air to detect a Sigfox Beacon.
 *        It will return the RC enum value corresponding to the beacon found and its RSSI level.
 *        The scan is executed during the specific timer / unit time
 *
 *        NOTE : there is no need to open the sigfox Library to run this function.
 *
 * \param[in] sfx_u8 rc_capabilities_bit_mask   Bit Mask (format below ) of the RCx on which the scan has to be executed :
 *
 *                                              ---------------------------------------------------------
 *                                              | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
 *                                              ---------------------------------------------------------
 *                                              |  -   |  RC7 | RC6  | RC5  | RC4  | RC3  | RC2  |  RC1 |
 *                                              ---------------------------------------------------------
 *
 * \param[in] sfx_u16 timer                     Scan duration value ( with the unit parameter information )
 * \param[in] sfx_timer_unit_enum_t  unit       Unit to be considered for the scan time computation
 * \param[in] app_callback_handler              This is the function that will be called by the Sigfox Library when the scan is completed.
 *                                              ( either when RC Found or when Timeout )
 *                 parameter of this callback are :
 *                     \param[out] sfx_u8       rc  Value of the RC found. There could be only 1 RC or 0 ( not found )
 *                     \param[out] rssi         RSSI value of the RC found. if rc = 0, rssi is not valid ( is set to 0 too )
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE or SFX_ERR_MONARCH_API_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_MONARCH_API_execute_rc_scan(sfx_u8 rc_capabilities_bit_mask, sfx_u16 timer, sfx_timer_unit_enum_t unit, sfx_u8 (* app_callback_handler)(sfx_u8 rc_bit_mask, sfx_s16 rssi));

/*!******************************************************************
 * \fn sfx_error_t SIGFOX_MONARCH_API_stop_rc_scan ( void )
 * \brief This function stops a RC scan which is on going
 *
 *        NOTE : there is no need to open the sigfox Library to run this function.
 *
 *    MSB_____________________________________________LSB
 *    15                      8|7                     0
 *     |                       |                      |
 *     |   MANUF_ERROR_CODE    |  SIGFOX_ERROR_CODE   |
 *     |_______________________|______________________|
 *
 *  SIGFOX_ERROR_CODE for this function : SFX_ERR_NONE or SFX_ERR_MONARCH_API_xx
 *
 *******************************************************************/
sfx_error_t SIGFOX_MONARCH_API_stop_rc_scan(void);

#endif
