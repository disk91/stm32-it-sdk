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
#include <it_sdk/encrypt/encrypt.h>

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


#define SIGFOX_UNSUPPORTED	0
#define SIGFOX_RCZ1		1
#define SIGFOX_RCZ2 	2
#define SIGFOX_RCZ3C 	3
#define SIGFOX_RCZ4		4
#define SIGFOX_RCZ5		5
#define SIGFOX_RCZ6		6

#define SIGFOX_KEY_PRIVATE	0
#define SIGFOX_KEY_PUBLIC	1

#define SIGFOX_DEFAULT_POWER	-127			// Use these values as config for setting the default value for state
#define SIGFOX_DEFAULT_SPEED	0xFFFF

typedef uint32_t itsdk_sigfox_device_is_t;

#define SIGFOX_POWER_DEFAULT	-1

// --------------------------------------------------------------------
// Header for NVM storage
// --------------------------------------------------------------------
#define ITSDK_SIGFOX_NVM_MAGIC	0x5178

typedef struct {
	uint16_t	magic;
	uint8_t		size;
	uint8_t		reserved;
} itsdk_sigfox_nvm_header_t;


// --------------------------------------------------------------------
// Public Functions
// --------------------------------------------------------------------

itsdk_sigfox_init_t itsdk_sigfox_setup();
itsdk_sigfox_init_t itsdk_sigfox_loop();
itsdk_sigfox_init_t itsdk_sigfox_deinit();
itsdk_sigfox_init_t itsdk_sigfox_getCurrentRcz(uint8_t * rcz);
itsdk_sigfox_init_t itsdk_sigfox_getTxPower(int8_t * power);
itsdk_sigfox_init_t itsdk_sigfox_setTxPower(int8_t power);
itsdk_sigfox_init_t itsdk_sigfox_setTxPower_ext(int8_t power, bool force);
itsdk_sigfox_init_t itsdk_sigfox_setTxSpeed(itdsk_sigfox_speed_t speed);
itsdk_sigfox_init_t itsdk_sigfox_getTxSpeed(itdsk_sigfox_speed_t * speed);
itsdk_sigfox_init_t itsdk_sigfox_getDeviceId(itsdk_sigfox_device_is_t * devId);
itsdk_sigfox_init_t itsdk_sigfox_getInitialPac(uint8_t * pac);
itsdk_sigfox_init_t itsdk_sigfox_getLastRssi(int16_t * rssi);
itsdk_sigfox_init_t itsdk_sigfox_switchPublicKey();
itsdk_sigfox_init_t itsdk_sigfox_switchPrivateKey();
itsdk_sigfox_init_t itsdk_sigfox_setRcSyncPeriod(uint16_t numOfFrame);
itsdk_sigfox_init_t itsdk_sigfox_getLastSeqId(uint16_t * seqId);
itsdk_sigfox_init_t itsdk_sigfox_getNextSeqId(uint16_t * seqId);
itsdk_sigfox_init_t itsdk_sigfox_getSigfoxLibVersion(uint8_t ** version);
itsdk_sigfox_init_t itsdk_sigfox_getRczFromRegion(uint32_t region, uint8_t * rcz);

itdsk_sigfox_txrx_t itsdk_sigfox_sendFrame(
		uint8_t * buf,
		uint8_t len,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		itdsk_payload_encrypt_t encrypt,
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
// Public function for internal use
// --------------------------------------------------------------------
itsdk_sigfox_init_t itsdk_sigfox_getNvmSize(uint32_t * sz);
itsdk_sigfox_init_t itsdk_sigfox_getNvmOffset(uint32_t * offset);
itsdk_sigfox_init_t itsdk_sigfox_getSeNvmOffset(uint32_t * offset);
itsdk_sigfox_init_t itsdk_sigfox_getSigfoxNvmOffset(uint32_t * offset);
itsdk_sigfox_init_t __itsdk_sigfox_resetNvmToFactory(bool force);

// --------------------------------------------------------------------
// Function to be overloaded in the main program
// --------------------------------------------------------------------
itsdk_sigfox_init_t itsdk_sigfox_resetFactoryDefaults(bool force);
itsdk_sigfox_init_t itsdk_sigfox_getKEY(uint8_t * key);

// --------------------------------------------------------------------
// Logging
// --------------------------------------------------------------------


#if (ITSDK_LOGGER_MODULE & __LOG_MOD_STKSIGFOX) > 0
#define LOG_INFO_SIGFOXSTK(x)		log_info x
#define LOG_WARN_SIGFOXSTK(x) 		log_warn x
#define LOG_ERROR_SIGFOXSTK(x)		log_error x
#define LOG_DEBUG_SIGFOXSTK(x)		log_debug x
#else
#define LOG_INFO_SIGFOXSTK(x)
#define LOG_WARN_SIGFOXSTK(x)
#define LOG_ERROR_SIGFOXSTK(x)
#define LOG_DEBUG_SIGFOXSTK(x)
#endif


#endif //ITSDK_WITH_SIGFOX_LIB

#endif // IT_SDK_SIGFOX_H_

