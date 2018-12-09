/* ==========================================================
 * sigfox.h - Sigfox communication abstraction layer
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 04 nov. 2018
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
 *
 * ==========================================================
 */
#ifndef IT_SDK_SIGFOX_H_
#define IT_SDK_SIGFOX_H_

#include <stdbool.h>

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>

#ifdef ITSDK_WITH_SIGFOX_LIB


typedef enum {
	SIGFOX_INIT_SUCESS = 0,
	SIGFOX_INIT_FAILED,
	SIGFOX_INIT_NOCHANGE,
	SIGFOX_INIT_PARAMSERR
} itsdk_sigfox_init_t;

typedef enum {
	SIGFOX_TRANSMIT_SUCESS = 0,				// Uplink success
	SIGFOX_TXRX_NO_DOWNLINK,				// Uplink / Downlink success, backend did not report downlink value
	SIGFOX_TXRX_DOWLINK_RECEIVED,			// Uplink / Downlink success, backend retruned downlink value
	SIGFOX_ERROR_PARAMS,					// Wrong parameters used when calling the function
	SIGFOX_TXRX_ERROR,						// Underlaying sigfox stack returned an error
} itdsk_sigfox_txrx_t;

typedef enum {
	SIGFOX_SPEED_DEFAULT = 0,
	SIGFOX_SPEED_100 = 100,
	SIGFOX_SPEED_600 = 600
} itdsk_sigfox_speed_t;

typedef enum
{
    SIGFOX_OOB_SERVICE = 0,
    SIGFOX_OOB_RC_SYNC
} itdsk_sigfox_oob_t;

typedef enum {												// Encryption mode are cumulative
	SIGFOX_ENCRYPT_NONE = __SIGFOX_ENCRYPT_NONE,			// Clear text payload
	SIGFOX_ENCRYPT_SIGFOX = __SIGFOX_ENCRYPT_SIGFOX,		// Sigfox native encryption
	SIGFOX_ENCRYPT_AESCTR = __SIGFOX_ENCRYPT_AESCTR,		// Software AES-CTR (like sigfox) encryption
	SIGFOX_ENCRYPT_SPECK = __SIGFOX_ENCRYPT_SPECK			// SPECK32 encryption
} itdsk_sigfox_encrypt_t;

typedef uint32_t itsdk_sigfox_device_is_t;


typedef struct {
	bool		initialized;
	uint8_t		rcz;
	uint8_t		default_power;
	uint16_t	default_speed;
} itsdk_sigfox_state;

#define SIGFOX_POWER_DEFAULT	-1

// --------------------------------------------------------------------
// Public Functions
// --------------------------------------------------------------------

itsdk_sigfox_init_t itsdk_sigfox_setup();
itsdk_sigfox_init_t itsdk_sigfox_setTxPower(uint8_t power);
itsdk_sigfox_init_t itsdk_sigfox_setTxSpeed(itdsk_sigfox_speed_t speed);
itsdk_sigfox_init_t itsdk_sigfox_getDeviceId(itsdk_sigfox_device_is_t * devId);
itsdk_sigfox_init_t itsdk_sigfox_getInitialPac(uint8_t * pac);
itsdk_sigfox_init_t itsdk_sigfox_getLastRssi(int16_t * rssi);
itsdk_sigfox_init_t itsdk_sigfox_switchPublicKey();
itsdk_sigfox_init_t itsdk_sigfox_switchPrivateKey();
itsdk_sigfox_init_t itsdk_sigfox_setRcSyncPeriod(uint16_t numOfFrame);
itsdk_sigfox_init_t itsdk_sigfox_getLastSeqId(uint16_t * seqId);
itsdk_sigfox_init_t itsdk_sigfox_getNextSeqId(uint16_t * seqId);

itdsk_sigfox_txrx_t itsdk_sigfox_sendFrame(
		uint8_t * buf,
		uint8_t len,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		itdsk_sigfox_encrypt_t encrypt,
		bool ack,
		uint8_t * dwn
);

itdsk_sigfox_txrx_t itsdk_sigfox_sendBit(
		bool bitValue,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		bool ack,
		uint8_t * dwn
);

itdsk_sigfox_txrx_t itsdk_sigfox_sendOob(
		itdsk_sigfox_oob_t oobType,
		itdsk_sigfox_speed_t speed,
		int8_t power
);

itsdk_sigfox_init_t itsdk_sigfox_continuousModeStart(
		uint32_t				frequency,
		itdsk_sigfox_speed_t 	speed,
		int8_t 					power
);
itsdk_sigfox_init_t itsdk_sigfox_continuousModeStop();

// --------------------------------------------------------------------
// Function to be overloaded in the main program
// --------------------------------------------------------------------
itsdk_sigfox_init_t itsdk_sigfox_eas_getNonce(uint8_t * nonce);
itsdk_sigfox_init_t itsdk_sigfox_eas_getSharedKey(uint32_t * sharedKey);
itsdk_sigfox_init_t itsdk_sigfox_eas_getMasterKey(uint8_t * masterKey);
itsdk_sigfox_init_t itsdk_sigfox_speck_getMasterKey(uint64_t * masterKey);
// --------------------------------------------------------------------
// Logging
// --------------------------------------------------------------------

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_SIGFOX) > 0
#define LOG_INFO_SIGFOX(x)		log_info x
#define LOG_WARN_SIGFOX(x) 		log_warn x
#define LOG_ERROR_SIGFOX(x)		log_error x
#define LOG_DEBUG_SIGFOX(x)		log_debug x
#else
#define LOG_INFO_SIGFOX(x)
#define LOG_WARN_SIGFOX(x)
#define LOG_ERROR_SIGFOX(x)
#define LOG_DEBUG_SIGFOX(x)
#endif

#endif //ITSDK_WITH_SIGFOX_LIB

#endif // IT_SDK_SIGFOX_H_

