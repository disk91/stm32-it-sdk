/* ==========================================================
 * lorawan.h - Abstraction layer for lorawan libraries
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 22 jan. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2019 Disk91
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
#ifndef IT_SDK_LORAWAN_H_
#define IT_SDK_LORAWAN_H_

#include <it_sdk/encrypt/encrypt.h>

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_STKLORA) > 0
#define LOG_INFO_LORAWANSTK(x)		log_info x
#define LOG_WARN_LORAWANSTK(x) 		log_warn x
#define LOG_ERROR_LORAWANSTK(x)		log_error x
#define LOG_DEBUG_LORAWANSTK(x)		log_debug x
#else
#define LOG_INFO_LORAWANSTK(x)
#define LOG_WARN_LORAWANSTK(x)
#define LOG_ERROR_LORAWANSTK(x)
#define LOG_DEBUG_LORAWANSTK(x)
#endif


typedef enum {
	LORAWAN_RETURN_SUCESS = 0,
	LORAWAN_RETURN_FAILED
} itsdk_lorawan_return_t;


typedef enum {
	LORAWAN_INIT_SUCESS = 0,
	LORAWAN_INIT_CHANNEL_FAILED,
	LORAWAN_INIT_FAILED
} itsdk_lorawan_init_t;

typedef enum {
	LORAWAN_JOIN_PENDING = 0,
	LORAWAN_JOIN_SUCCESS,
	LORAWAN_JOIN_FAILED
} itsdk_lorawan_join_t;

typedef enum {
	LORAWAN_SEND_QUEUED = 0,				  // Message put in the sending queue (SUCCESS)
	LORAWAN_SEND_SENT,						  // Message sent (SUCCESS)
	LORAWAN_SEND_ACKED,						  // Message sent and Acked (SUCCESS)
	LORAWAN_SEND_ACKED_WITH_DOWNLINK,		  // Message sent and Acked with downlink (SUCCESS)
	LORAWAN_SEND_ACKED_WITH_DOWNLINK_PENDING, // Message sent and Acked with downlink & Other downlink is pending (SUCCESS)
	LORAWAN_SEND_RUNNING,					  // Message is in progress (async mode) (SUCCESS)
	LORAWAN_SEND_NOT_JOINED,				  // The device has not joined message can be sent (ERROR)
	LORAWAN_SEND_DUTYCYCLE,					  // Not sent - duty cycle constraints
	LORAWAN_SEND_ALREADYRUNNING,			  // A transmission is already in progress (ERROR)
	LORAWAN_SEND_FAILED						  // Various other failure (ERROR)
} itsdk_lorawan_send_t;


typedef enum {
	LORAWAN_DEVICE_CLASS_A = 0,
	LORAWAN_DEVICE_CLASS_B,
	LORAWAN_DEVICE_CLASS_C
} itsdk_lorawan_dev_class;

typedef enum {
	LORAWAN_RUN_SYNC = 0,
	LORAWAN_RUN_ASYNC
} itsdk_lorawan_run_t;

typedef enum {
	LORAWAN_SEND_CONFIRMED = 0,
	LORAWAN_SEND_UNCONFIRMED
} itsdk_lorawan_sendconf_t;

typedef enum {
	LORAWAN_RSSISNR_VALID = 0,
	LORAWAN_RSSISNR_INVALID
} itsdk_lorawan_rssisnr_t;

typedef enum {
	LORAWAN_CHANNEL_SUCCESS = 0,
	LORAWAN_CHANNEL_INVALID_PARAMS,
	LORAWAN_CHANNEL_FAILED
} itsdk_lorawan_channel_t;

typedef enum {
	LORAWAN_TXPOWER_0 = 0,
	LORAWAN_TXPOWER_1 = 1,
	LORAWAN_TXPOWER_2 = 2,
	LORAWAN_TXPOWER_3 = 3,
	LORAWAN_TXPOWER_4 = 4,
	LORAWAN_TXPOWER_5 = 5,
	LORAWAN_TXPOWER_6 = 6,
	LORAWAN_TXPOWER_7 = 7,
	LORAWAN_TXPOWER_8 = 8,
	LORAWAN_TXPOWER_9 = 9,
	LORAWAN_TXPOWER_10 = 10,
	LORAWAN_TXPOWER_11 = 11,
	LORAWAN_TXPOWER_12 = 12,
	LORAWAN_TXPOWER_13 = 13,
	LORAWAN_TXPOWER_14 = 14,
	LORAWAN_TXPOWER_15 = 15,
	LORAWAN_TXPOWER_16 = 16,
	LORAWAN_TXPOWER_17 = 17,
	LORAWAN_TXPOWER_18 = 18,
	LORAWAN_TXPOWER_19 = 19,
	LORAWAN_TXPOWER_20 = 20,
	LORAWAN_TXPOWER_21 = 21,
	LORAWAN_TXPOWER_22 = 22
} itsdk_lorawan_txpower;

typedef struct {
	uint8_t num;
	struct {
		uint8_t  id;
		uint32_t frequency;
		uint32_t frequencyRx;
		uint8_t  minDr;
		uint8_t  maxDr;
		uint8_t  band;
	} channels[];
} itsdk_lorawan_channelInit_t;

// ===============================================================
// PUBLIC API
// ===============================================================


itsdk_lorawan_init_t itsdk_lorawan_setup(									// Init LoRaWan lib
		uint16_t region, 													//   Region
		itsdk_lorawan_channelInit_t * channelConfig							//   Static Channel configuration
);

itsdk_lorawan_join_t itsdk_lorawan_join_sync();								// Join according to configuration (synchrnous)
itsdk_lorawan_join_t itsdk_lorawan_join_async(								// Join asynchronously
		void (*callback_func)(itsdk_lorawan_join_t status)					//  function to callback on join success/failed
);
bool itsdk_lorawan_hasjoined();												// Ensure we have joined
itsdk_lorawan_join_t itsdk_lorawan_getJoinState();							// Join state details

itsdk_lorawan_send_t itsdk_lorawan_send_sync(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate,
		itsdk_lorawan_sendconf_t confirm,
		uint8_t	  retry,
		uint8_t	* rPort,													// In case of reception - Port (uint8_t)
		uint8_t	* rSize,													// In case of reception - Size (uint8_t) - init with buffer max size
		uint8_t * rData,													// In case of recpetion - Data (uint8_t[] bcopied)
		itdsk_payload_encrypt_t encrypt										// End to End encryption mode
);
itsdk_lorawan_send_t itsdk_lorawan_send_async(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate,
		itsdk_lorawan_sendconf_t confirm,
		uint8_t	  retry,
		void (*callback_func)(itsdk_lorawan_send_t status, uint8_t port, uint8_t size, uint8_t * rxData),
		itdsk_payload_encrypt_t encrypt										// End to End encryption mode
);
itsdk_lorawan_send_t itsdk_lorawan_getSendState();						// Send state for polling

itsdk_lorawan_return_t itsdk_lorawan_changeDefaultRate(uint8_t newRate);
itsdk_lorawan_rssisnr_t itsdk_lorawan_getLastRssiSnr(int16_t *rssi, uint8_t *snr);
itsdk_lorawan_return_t itsdk_lorawan_setTxPower(itsdk_lorawan_txpower txPwr);
itsdk_lorawan_return_t itsdk_lorawan_getTxPower(itsdk_lorawan_txpower * power);
itsdk_lorawan_return_t itsdk_lorawan_getDownlinkFrameCounter(uint16_t * counter);
itsdk_lorawan_return_t itsdk_lorawan_getUplinkFrameCounter(uint16_t * counter);
itsdk_lorawan_return_t itsdk_lorawan_getNextUplinkFrameCounter(uint16_t * counter);
itsdk_lorawan_return_t itsdk_lorawan_resetFactoryDefaults(bool force);
void itsdk_lorawan_loop();													// LoRaWan stack processing loop - MUST be in project_loop()


// ===============================================================
// CAN BE OVERRIDDED
// ===============================================================

// Static key access... override to work differently.
itsdk_lorawan_return_t itsdk_lorawan_aes_getNonce(uint8_t * nonce);
itsdk_lorawan_return_t itsdk_lorawan_aes_getSharedKey(uint32_t * sharedKey);
itsdk_lorawan_return_t itsdk_lorawan_aes_getMasterKey(uint8_t * masterKey);
itsdk_lorawan_return_t itsdk_lorawan_speck_getMasterKey(uint64_t * masterKey);

// Get the DevEUI
itsdk_lorawan_return_t itsdk_lorawan_getDeviceId(uint64_t * devId);
itsdk_lorawan_return_t itsdk_lorawan_getDeviceEUI(uint8_t * devEui);
itsdk_lorawan_return_t itsdk_lorawan_getAppEUI(uint8_t * appEui);
itsdk_lorawan_return_t itsdk_lorawan_getAppKEY(uint8_t * appKey);

// ===============================================================
// TO BE OVERRIDDED
// ===============================================================

// Function automatically fired on data reception
void itsdk_lorawan_onDataReception(uint8_t port, uint8_t * data, uint8_t size);
// Function automatically fired on join success
void itsdk_lorawan_onJoinSuccess();
// Function automatically fired on device class change confirmation
void itsdk_lorawan_onConfirmClass(itsdk_lorawan_dev_class class);
// Function automatically fired when the network is requesting a Uplink transmission
void itsdk_lorawan_onTxNeeded();
// Function automatically fired when the network server has confirmed ack reception
void itsdk_lorawan_uplinkAckConfirmed();

#endif
