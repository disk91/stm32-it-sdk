/* ==========================================================
 * lorawan.c - Abstraction layer for lorawan libraries
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
#include <stdbool.h>
#include <string.h>

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>

#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
#include <drivers/lorawan/core/lorawan.h>
#include <drivers/lorawan/core/lora.h>
#include <drivers/lorawan/phy/radio.h>
#include <it_sdk/lorawan/lorawan.h>

// =================================================================================
// INIT
// =================================================================================

// Unsecured storage... lets modify this part later
// @TODO
static uint8_t devEui[8] = ITSDK_LORAWAN_DEVEUI;
static uint8_t appEui[8] = ITSDK_LORAWAN_APPEUI;
static uint8_t appKey[16] = ITSDK_LORAWAN_APPKEY;

/**
 * Init the LoRaWan Stack
 * Actually static
 */
itsdk_lorawan_init_t itsdk_lorawan_setup(uint16_t region, itsdk_lorawan_channelInit_t * channelConfig) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_setup\r\n"));
	lorawan_driver_config_t __config;

	Radio.IoInit();

	#if (ITSDK_LORAWAN_DEVEUI_SRC == __LORAWAN_DEVEUI_GENERATED)
	  itsdk_getUniqId(DevEui, 8);
	#endif


	#if ITSDK_LORAWAN_ADR == __LORAWAN_ADR_ON
	__config.adrEnable =LORAWAN_ADR_ON;
	#elif ITSDK_LORAWAN_ADR == __LORAWAN_ADR_OFF
	__config.adrEnable =LORAWAN_ADR_OFF;
	#else
   	  #error Invalid ITSDK_LORAWAN_ADR configuration
	#endif

	__config.JoinType = ITSDK_LORAWAN_ACTIVATION;
	__config.devEui = devEui;
	#if ITSDK_LORAWAN_NETWORKTYPE == __LORAWAN_NWK_PUBLIC
	__config.enablePublicNetwork = true;
	#else
	__config.enablePublicNetwork = false;
	#endif
	__config.region = region;
	__config.txDatarate = ITSDK_LORAWAN_DEFAULT_DR;
	#if ITSDK_LORAWAN_ACTIVATION ==  __LORAWAN_OTAA
	__config.config.otaa.appEui = appEui;
	__config.config.otaa.appKey = appKey;
	__config.config.otaa.nwkKey = appKey;
	#else
		#error "ABP not yest supported"
	#endif

	lorawan_driver_LORA_Init(&__config);
	bzero(&__config,sizeof(__config));

	if ( channelConfig != NULL ) {
		for ( int i=0 ; i < channelConfig->num ; i++ ) {
			if ( lorawan_driver_LORA_AddChannel(
					channelConfig->channels[i].id,
					channelConfig->channels[i].frequency,
					channelConfig->channels[i].frequencyRx,
					channelConfig->channels[i].minDr,
					channelConfig->channels[i].maxDr,
					channelConfig->channels[i].band
				) != LORAWAN_CHANNEL_SUCCESS ) return LORAWAN_INIT_CHANNEL_FAILED;
		}
	}
	return LORAWAN_INIT_SUCESS;
}

// =================================================================================
// JOIN
// =================================================================================

/**
 * Join Process
 * The process can be
 *  - synchronous : will return after connection success or failed ( return - LORAWAN_JOIN_PENDING )
 *  - asynchronous : will return immediately after transmission and reception will be managed over timer & interrupt
 *                   it allows to switch low power but is a higher risk in term of timing respect.
 *                   returns (LORAWAN_JOIN_SUCCESS/LORAWAN_JOIN_FAILED)
 *                   in async mode the given function will be called with the join status.
 *                   When no function is proposed, the status can be polled.
 */
itsdk_lorawan_join_t itsdk_lorawan_join_sync() {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_join_sync\r\n"));
	return lorawan_driver_LORA_Join(LORAWAN_RUN_SYNC);
}


// Override the underlaying callbacks
static void (*__itsdk_lorawan_join_cb)(itsdk_lorawan_join_t status)  = NULL;
void lorawan_driver_onJoinSuccess() {
	LOG_INFO_LORAWANSTK(("** onJoinSuccess\r\n"));
	if (__itsdk_lorawan_join_cb != NULL) {
		__itsdk_lorawan_join_cb(LORAWAN_JOIN_SUCCESS);
	}
}
void lorawan_driver_onJoinFailed() {
	LOG_INFO_LORAWANSTK(("** onJoinFailed\r\n"));
	if (__itsdk_lorawan_join_cb != NULL) {
		__itsdk_lorawan_join_cb(LORAWAN_JOIN_FAILED);
	}
}

itsdk_lorawan_join_t itsdk_lorawan_join_async(
		void (*callback_func)(itsdk_lorawan_join_t status)
) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_join_async\r\n"));
	if ( callback_func != NULL ) {
		__itsdk_lorawan_join_cb = callback_func;
	} else {
		__itsdk_lorawan_join_cb = NULL;
	}
	return lorawan_driver_LORA_Join(LORAWAN_RUN_ASYNC);
}

/**
 * Return true once the device has joined the the network
 */
bool itsdk_lorawan_hasjoined() {
	LOG_DEBUG_LORAWANSTK(("itsdk_lorawan_hasjoined\r\n"));
	return ( lorawan_driver_LORA_getJoinState() == LORAWAN_STATE_JOIN_SUCCESS);
}

itsdk_lorawan_join_t itsdk_lorawan_getJoinState() {
	LOG_DEBUG_LORAWANSTK(("itsdk_lorawan_getJoinState\r\n"));
#warning tobe fixed
	return lorawan_driver_LORA_getSendState();
}

// =================================================================================
// SEND
// =================================================================================


/**
 * Send a LoRaWAN frame containing the Payload of the given payloadSize bytes on given port.
 * The dataRate can be set.
 * Confirmation mode (downlink) can be specified. In this case a number of retry can be precised.
 * The send can be synchronous or asynchronous.
 * In synchronous mode the status will be
 *   - LORAWAN_SEND_SENT/LORAWAN_SEND_ACKED on success
 *   - LORAWAN_SEND_NOT_JOINED / LORAWAN_SEND_DUTYCYCLE / LORAWAN_SEND_FAILED on error
 * In asynchronous mode the status will be
 *   - LORAWAN_SEND_RUNNING
 */
itsdk_lorawan_send_t itsdk_lorawan_send_sync(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate,
		itsdk_lorawan_sendconf_t confirm,
		uint8_t	  retry
) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_send_sync\r\n"));
	return lorawan_driver_LORA_Send(payload,payloadSize,port,dataRate,confirm,retry,LORAWAN_RUN_SYNC);
}

static void (*__itsdk_lorawan_send_cb)(itsdk_lorawan_send_t status)  = NULL;
void lorawan_driver_onSendSuccessAckFailed() {
	LOG_INFO_LORAWANSTK(("**onSendSuccessAckFailed\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_STATE_NOTACKED);
	}
}
void lorawan_driver_onSendAckSuccess() {
	LOG_INFO_LORAWANSTK(("**onSendSuccessAckSuccess\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_STATE_ACKED);
	}
}
void lorawan_driver_onSendSuccess() {
	LOG_INFO_LORAWANSTK(("**onSendSuccess\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_STATE_SENT);
	}
}
void lorawan_driver_onDataReception(uint8_t port, uint8_t * data, uint8_t size) {
	LOG_INFO_LORAWANSTK(("**onDataReception\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK);
	}
}

void lorawan_driver_onPendingDownlink() {
	LOG_INFO_LORAWANSTK(("**onPendingDownlink\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_STATE_ACKED_DOWNLINK_PENDING);
	}
}

itsdk_lorawan_send_t itsdk_lorawan_send_async(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate,
		itsdk_lorawan_sendconf_t confirm,
		uint8_t	  retry,
		void (*callback_func)(itsdk_lorawan_send_t status)
) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_send_async\r\n"));
	if ( callback_func != NULL ) {
		__itsdk_lorawan_send_cb = callback_func;
	} else {
		__itsdk_lorawan_send_cb = NULL;
	}
	return lorawan_driver_LORA_Send(payload,payloadSize,port,dataRate,confirm,retry,LORAWAN_RUN_ASYNC);
}

/**
 * Return sent current state
 */
itsdk_lorawan_send_t itsdk_lorawan_getSendState() {
# warning to be fixed
	LOG_DEBUG_LORAWANSTK(("itsdk_lorawan_getSendState\r\n"));
	return lorawan_driver_LORA_getSendState();
}

// =================================================================================
// MISC
// =================================================================================

void itsdk_lorawan_changeDefaultRate(uint8_t newRate) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_changeDefaultRate\r\n"));
	lorawan_driver_LORA_ChangeDefaultRate(newRate);
}

itsdk_lorawan_rssisnr_t itsdk_lorawan_getLastRssiSnr(int16_t *rssi, uint8_t *snr){
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getLastRssiSnr\r\n"));
	return lorawan_driver_LORA_GetLastRssiSnr(rssi,snr);
}

bool itsdk_lorawan_setTxPower(itsdk_lorawan_txpower txPwr) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_setTxPower\r\n"));
	return lorawan_driver_LORA_SetTxPower(txPwr);
}

itsdk_lorawan_txpower itsdk_lorawan_getTxPower() {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getTxPower\r\n"));
	return lorawan_driver_LORA_GetTxPower();
}


uint16_t itsdk_lorawan_getDownlinkFrameCounter() {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getDownlinkFrameCounter\r\n"));
	return lorawan_driver_LORA_GetDownlinkFrameCounter();
}


uint16_t itsdk_lorawan_getUplinkFrameCounter() {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getUplinkFrameCounter\r\n"));
	return lorawan_driver_LORA_GetUplinkFrameCounter();
}

/**
 * This function need to be called in the project_loop function
 * to manage the lorawan stack ( mandatory for async mode )
 */
void itsdk_lorawan_loop() {
	LOG_DEBUG_LORAWANSTK(("itsdk_lorawan_loop\r\n"));
	lorawan_driver_loop();
}



#endif
