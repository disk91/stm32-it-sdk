/* ==========================================================
 * sx1276Sigfox.h - Sigfox implementation on sx1276
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 1 may 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
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
 * ==========================================================
 */
#ifndef IT_SDK_DRIVERS_SX1276_SIGFOX_H_
#define IT_SDK_DRIVERS_SX1276_SIGFOX_H_

#include <it_sdk/config.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276)
#include <it_sdk/logger/logger.h>
#include <drivers/sx1276/sigfox_lowlevel.h>

typedef enum {
	SX1276_SIGFOX_ERR_NONE = 0,
	SX1276_SIGFOX_ERR_BREAK,			// Force to break a wait loop
	SX1276_SIGFOX_ERR_LIBINIT,			// Impossible to open Sigfox Lib
	SX1276_SIGFOX_ERR_CONFIG			// Error on Set Std Config


} sx1276_sigfox_ret_t;

sx1276_sigfox_ret_t sx1276_sigfox_init( void );
sx1276_sigfox_ret_t sx1276_sigfox_deinit( void );
sx1276_sigfox_ret_t sx1276_sigfox_idle( void );
sx1276_sigfox_ret_t sx1276_sigfox_getRssi(int16_t * rssi);
sx1276_sigfox_ret_t sx1276_sigfox_getSeqId( uint16_t * seqId );
sx1276_sigfox_ret_t sx1276_sigfox_setPower( int8_t power );

// Function you can override
sx1276_sigfox_ret_t sx1276_sigfox_idle_used( void );

/* ----------------------------------------------------------------
 * Misc defines
 */
#define SIGFOX_MCU_API_VER				"MCU_API_V1.0"
#define SIGFOX_MAX_CS_RSSI_AVG 			8
/* ----------------------------------------------------------------
 * Logging
 */

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
#define LOG_INFO_SFXSX1276(x)		log_info x
#define LOG_WARN_SFXSX1276(x) 		log_warn x
#define LOG_ERROR_SFXSX1276(x)		log_error x
#define LOG_DEBUG_SFXSX1276(x)		log_debug x
#else
#define LOG_INFO_SFXSX1276(x)
#define LOG_WARN_SFXSX1276(x)
#define LOG_ERROR_SFXSX1276(x)
#define LOG_DEBUG_SFXSX1276(x)
#endif

/* ----------------------------------------------------------------
 * Lib State
 */
typedef enum {
	SIGFOX_LPMODE_AUTHORIZED = 0,
	SIGFOX_LPMODE_PROHIBITED
} sigfox_lowpower_mode_t;

#define SIGFOX_EVENT_CLEAR 	0
#define SIGFOX_EVENT_SET	1

typedef struct {
	int16_t					meas_rssi_dbm;				// Computed Rssi
	STLL_flag   			rxPacketReceived;
	STLL_flag   			rxCarrierSenseFlag;
	sigfox_lowpower_mode_t	lowPowerAuthorised:1;		// do device can be switched LP mode

	volatile uint8_t		timerEvent:1;				// Timer event
	volatile uint8_t		endOfTxEvent:1;				// Frame sent

	uint32_t				lastHseSwitch_S;				// Last Hse/Hsi switch timestamp in S

} sx1276_sigfox_state_t;

extern sx1276_sigfox_state_t	sx1276_sigfox_state;

#endif // SIGFOX SX1276 ENABLE

#endif // IT_SDK_DRIVERS_SX1276_SIGFOX_H_
