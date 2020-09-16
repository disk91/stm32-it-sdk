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
#include <it_sdk/encrypt/encrypt.h>

#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
#include <drivers/lorawan/core/lorawan.h>
#include <drivers/lorawan/core/lora.h>
#include <drivers/lorawan/phy/radio.h>
#include <it_sdk/lorawan/lorawan.h>

#if ITSDK_WITH_SECURESTORE == __ENABLE
#include <it_sdk/eeprom/securestore.h>
#endif

#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
#include <it_sdk/eeprom/sdk_config.h>
#endif

#if ITSDK_WITH_ERROR_RPT == __ENABLE
#include <it_sdk/logger/error.h>
#endif


// =================================================================================
// INIT
// =================================================================================


/**
 * Init the LoRaWan Stack
 * Actually static
 */
itsdk_lorawan_init_t itsdk_lorawan_setup(uint16_t region, itsdk_lorawan_channelInit_t * channelConfig) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_setup\r\n"));
	lorawan_driver_config_t __config;
	static uint8_t devEui[8];
	static uint8_t appEui[8];
	static uint8_t appKey[16];

	// On first run we store the configuration into the SecureStore
	itsdk_lorawan_resetFactoryDefaults(false);

	itsdk_lorawan_getDeviceEUI(devEui);
	itsdk_lorawan_getAppEUI(appEui);
	itsdk_lorawan_getAppKEY(appKey);

//	log_info_array("DEV :",devEui,8);
//	log_info_array("APP :",appEui,8);
//	log_info_array("KEY :",appKey,16);

	Radio.IoInit();

	#if ITSDK_CONFIGURATION_MODE == __CONFIG_STATIC
		#if (ITSDK_LORAWAN_DEVEUI_SRC == __LORAWAN_DEVEUI_GENERATED)
		  itsdk_getUniqId(devEui, 8);
		#endif
	#else
	   if ( itsdk_config.sdk.lorawan.devEuiType == __LORAWAN_DEVEUI_GENERATED ) {
		  itsdk_getUniqId(devEui, 8);
	   }
	#endif

    #if ITSDK_CONFIGURATION_MODE == __CONFIG_STATIC
		#if ITSDK_LORAWAN_ADR == __LORAWAN_ADR_ON
		__config.adrEnable =LORAWAN_ADR_ON;
		#elif ITSDK_LORAWAN_ADR == __LORAWAN_ADR_OFF
		__config.adrEnable =LORAWAN_ADR_OFF;
		#else
		  #error Invalid ITSDK_LORAWAN_ADR configuration
		#endif
    #else
		__config.adrEnable = (itsdk_config.sdk.lorawan.adrMode == __LORAWAN_ADR_ON)?LORAWAN_ADR_ON:LORAWAN_ADR_OFF;
	#endif

	#if ITSDK_CONFIGURATION_MODE == __CONFIG_STATIC
	__config.JoinType = ITSDK_LORAWAN_ACTIVATION;
	#else
	__config.JoinType = itsdk_config.sdk.lorawan.joinMode;
	#endif
	__config.devEui = devEui;
	#if ITSDK_CONFIGURATION_MODE == __CONFIG_STATIC
		#if ITSDK_LORAWAN_NETWORKTYPE == __LORAWAN_NWK_PUBLIC
		__config.enablePublicNetwork = true;
		#else
		__config.enablePublicNetwork = false;
		#endif
	#else
	__config.enablePublicNetwork = (itsdk_config.sdk.lorawan.networkType == __LORAWAN_NWK_PUBLIC);
	#endif


	__config.region = region;
	__config.txDatarate = ITSDK_LORAWAN_DEFAULT_DR;
	#if ( ITSDK_LORAWAN_ACTIVATION &  __LORAWAN_OTAA )> 0
	__config.config.otaa.appEui = appEui;
	__config.config.otaa.appKey = appKey;
	__config.config.otaa.nwkKey = appKey;
	#else
		#error "ABP not yest supported"
	#endif

	lorawan_driver_LORA_Init(&__config);
	bzero(&__config,sizeof(__config));

	if ( channelConfig != NULL ) {
		switch (region) {
		case __LORAWAN_REGION_US915:
		{
			// US915 does not allow to addChannel ; all the possible channel are already defined
			// and activated. Here we basically unactivate the one we do not need
			uint16_t channels[6];
			bzero(channels,6*sizeof(uint16_t));
			for ( int i=0 ; i < channelConfig->num ; i++ ) {
				int channel = (channelConfig->channels[i].frequency - 902300000) / 200000; // get chan ID on the 72
				int index = channel >> 4; // associated word
				int shift = channel & 0x0F;
				channels[index] |= (1 << shift);
				LOG_DEBUG_LORAWANSTK(("Add channel %d at freq %d on idx %d with shift %d\r\n",channel,channelConfig->channels[i].frequency,index,shift));
			}
			lorawan_driver_LORA_SelectChannels(__LORAWAN_REGION_US915,channels);
		}
		break;

		default:
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
			break;
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
	lorawan_driver_joinState r = lorawan_driver_LORA_getSendState();
	switch(r) {
	case LORAWAN_STATE_JOIN_SUCCESS:
		return LORAWAN_JOIN_SUCCESS;
	case LORAWAN_STATE_JOIN_FAILED:
		return LORAWAN_JOIN_FAILED;

	default:
	case LORAWAN_STATE_NONE:
	case LORAWAN_STATE_INITIALIZED:
	case LORAWAN_STATE_JOINING:
	case LORAWAN_STATE_END:
		return LORAWAN_JOIN_PENDING;
	}
}

// =================================================================================
// SEND
// =================================================================================

/**
 * Send a LoRaWAN frame containing the Payload of the given payloadSize bytes on given port.
 * This first simple uplink is reducing the number of options...
 * Synchronous, no ack.
 */
itsdk_lorawan_send_t itsdk_lorawan_send_simple_uplink_sync(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate
) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_send_simple_uplink_sync\r\n"));
	return lorawan_driver_LORA_Send(payload,payloadSize,port,dataRate,LORAWAN_SEND_UNCONFIRMED,0,LORAWAN_RUN_SYNC, NULL,NULL,NULL);
}



/**
 * Internal function to encrypt the payload.
 * This is E2E encryption not LoRaWan encryption included in the LoRaWan Stack.
 * This encryption layer is a second layer of encryption on top of the payload before applying network security.
 * This encryption layer will need to be decrypted on the receiver side.
 */
static void __itsdk_lorawan_encrypt_payload(
	uint8_t * payload,
	uint8_t   payloadSize,
	itdsk_payload_encrypt_t encrypt
){
#if ( ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_SPECK ) > 0
	if ( (encrypt & PAYLOAD_ENCRYPT_SPECK) > 0 ) {
		uint64_t masterKey;
		itsdk_encrypt_speck_getMasterKey(&masterKey);
		itsdk_speck_encrypt(
				payload,
				payload,
				payloadSize,
				masterKey
		);
	}
#endif
#if (ITSDK_LORAWAN_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR) > 0
	if ( (encrypt & PAYLOAD_ENCRYPT_AESCTR) > 0 ) {
		uint64_t devId64;
		itsdk_lorawan_getDeviceId(&devId64);
		uint16_t seqId;
		itsdk_lorawan_getNextUplinkFrameCounter(&seqId);
		uint8_t nonce;
		itsdk_encrypt_aes_getNonce(&nonce);
		uint32_t sharedKey;
		itsdk_encrypt_aes_getSharedKey(&sharedKey);
		uint8_t masterKey[16];
		itsdk_encrypt_aes_getMasterKey(masterKey);

		itsdk_aes_ctr_encrypt_128B(
				payload,							// Data to be encrypted
				payload,							// Can be the same as clearData
				payloadSize,						// Size of data to be encrypted
				(uint32_t)devId64,					// 32b device ID (32b low)
				seqId,								// 16b sequenceId (incremented for each of the frame)
				nonce,								// 8b  value you can update dynamically from backend
				sharedKey,							// 24b hardcoded value (hidden with ITSDK_PROTECT_KEY)
				masterKey							// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
		);
	}
#endif
}


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
		uint8_t	  retry,
		uint8_t	* rPort,													// In case of reception - Port (uint8_t)
		uint8_t	* rSize,													// In case of reception - Size (uint8_t) - init with buffer max size
		uint8_t * rData,													// In case of recpetion - Data (uint8_t[] bcopied)
		itdsk_payload_encrypt_t encrypt										// End to End encryption mode
) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_send_sync\r\n"));
	__itsdk_lorawan_encrypt_payload(payload,payloadSize,encrypt);
	return lorawan_driver_LORA_Send(payload,payloadSize,port,dataRate,confirm,retry,LORAWAN_RUN_SYNC,rPort,rSize,rData);
}

static void (*__itsdk_lorawan_send_cb)(itsdk_lorawan_send_t status, uint8_t port, uint8_t size, uint8_t * rxData) = NULL;
void lorawan_driver_onSendSuccessAckFailed() {
	LOG_INFO_LORAWANSTK(("**onSendSuccessAckFailed\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_SENT,0,0,NULL);
	}
}
void lorawan_driver_onSendAckSuccess() {
	LOG_INFO_LORAWANSTK(("**onSendSuccessAckSuccess\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_ACKED,0,0,NULL);
	}
}
void lorawan_driver_onSendSuccess() {
	LOG_INFO_LORAWANSTK(("**onSendSuccess\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_SENT,0,0,NULL);
	}
}

void lorawan_driver_onDataReception(uint8_t port, uint8_t * data, uint8_t size) {
	LOG_INFO_LORAWANSTK(("**onDataReception (%d) : ",size));
	#if (ITSDK_LOGGER_MODULE & __LOG_MOD_STKLORA) > 0
		for ( int i = 0 ; i < size ; i++ ) {
			LOG_INFO_LORAWANSTK(("%02X ",data[i]));
		}
		LOG_INFO_LORAWANSTK(("\n"));
	#endif
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_ACKED_WITH_DOWNLINK,port,size,data);
	}
}

void lorawan_driver_onPendingDownlink() {
	LOG_INFO_LORAWANSTK(("**onPendingDownlink\r\n"));
	if (__itsdk_lorawan_send_cb != NULL) {
		__itsdk_lorawan_send_cb(LORAWAN_SEND_ACKED_WITH_DOWNLINK_PENDING,0,0,NULL);
	}
}

itsdk_lorawan_send_t itsdk_lorawan_send_async(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate,
		itsdk_lorawan_sendconf_t confirm,
		uint8_t	  retry,
		void (*callback_func)(itsdk_lorawan_send_t status, uint8_t port, uint8_t size, uint8_t * rxData),
		itdsk_payload_encrypt_t encrypt										// End to End encryption mode
) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_send_async\r\n"));
	__itsdk_lorawan_encrypt_payload(payload,payloadSize,encrypt);
	if ( callback_func != NULL ) {
		__itsdk_lorawan_send_cb = callback_func;
	} else {
		__itsdk_lorawan_send_cb = NULL;
	}
	return lorawan_driver_LORA_Send(payload,payloadSize,port,dataRate,confirm,retry,LORAWAN_RUN_ASYNC,NULL,NULL,NULL);
}

/**
 * Return sent current state
 */
itsdk_lorawan_send_t itsdk_lorawan_getSendState() {
	LOG_DEBUG_LORAWANSTK(("itsdk_lorawan_getSendState\r\n"));
	lorawan_driver_sendState r = lorawan_driver_LORA_getSendState();
	switch ( r ) {
	case LORAWAN_SEND_STATE_SENT:
	case LORAWAN_SEND_STATE_NOTACKED:
		return LORAWAN_SEND_SENT;
	case LORAWAN_SEND_STATE_ACKED:
	case LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK:
	case LORAWAN_SEND_STATE_ACKED_DOWNLINK_PENDING:
		return LORAWAN_SEND_ACKED;
	case LORAWAN_SEND_STATE_FAILED:
		return LORAWAN_SEND_FAILED;
	case LORAWAN_SEND_STATE_DUTYCYCLE:
		return LORAWAN_SEND_DUTYCYCLE;
	case LORAWAN_SEND_STATE_NONE:
	case LORAWAN_SEND_STATE_RUNNING:
	case LORAWAN_SEND_STATE_END:
	default:
		return LORAWAN_SEND_RUNNING;
	}
}

// =================================================================================
// MISC
// =================================================================================

/**
 * Get the device EUI as a uint64_t value
 */
__weak itsdk_lorawan_return_t itsdk_lorawan_getDeviceId(uint64_t * devId) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getDeviceId\r\n"));
	itsdk_lorawan_return_t r = LORAWAN_RETURN_SUCESS;
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[8];
	r = itsdk_lorawan_getDeviceEUI(d);
#else
	uint8_t d[8] = ITSDK_LORAWAN_DEVEUI;
#endif
	uint64_t di = 0;
	for ( int i = 0 ; i < 8 ; i++ ){
		di = (di << 8) | d[i];
	}
	*devId = di;
	return r;
}

/**
 * Get the device EUI as a uint8_t[]
 */
__weak itsdk_lorawan_return_t itsdk_lorawan_getDeviceEUI(uint8_t * devEui){
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getDeviceEUI\r\n"));
	#if ITSDK_WITH_SECURESTORE == __ENABLE
		uint8_t d[8];
		uint8_t buffer[16];
		if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, buffer) != SS_SUCCESS ) {
			#if ITSDK_WITH_ERROR_RPT == __ENABLE
				ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_SS_INVALID,0);
			#endif
			bzero(devEui,8);
			return LORAWAN_RETURN_FAILED;
		} else {
			memcpy(d,buffer,8);
		}
	#else
		uint8_t d[8] = ITSDK_LORAWAN_DEVEUI;
	#endif
	memcpy(devEui,d,8);
	return LORAWAN_RETURN_SUCESS;
}

/**
 * Get the appEUI as a uint8_t[]
 */
__weak itsdk_lorawan_return_t itsdk_lorawan_getAppEUI(uint8_t * appEui){
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getAppEUI\r\n"));
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[8];
	uint8_t buffer[16];
	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, buffer) != SS_SUCCESS ) {
		#if ITSDK_WITH_ERROR_RPT == __ENABLE
			ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_SS_INVALID,1);
		#endif
		bzero(appEui,8);
		return LORAWAN_RETURN_FAILED;
	} else {
		memcpy(d,buffer+8,8);
	}
#else
	uint8_t d[8] = ITSDK_LORAWAN_APPEUI;
#endif
	memcpy(appEui,d,8);
	return LORAWAN_RETURN_SUCESS;
}

/**
 * Get the appKEY as a uint8_t[]
 */
__weak itsdk_lorawan_return_t itsdk_lorawan_getAppKEY(uint8_t * appKey){
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getAppKEY\r\n"));
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[16];
	if ( itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_APPKEY, d) != SS_SUCCESS ) {
		#if ITSDK_WITH_ERROR_RPT == __ENABLE
			ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_SS_INVALID,2);
		#endif
		bzero(appKey,16);
		return LORAWAN_RETURN_FAILED;
	}
#else
	uint8_t d[16] = ITSDK_LORAWAN_APPKEY;
#endif
	memcpy(appKey,d,16);
	return LORAWAN_RETURN_SUCESS;
}


/**
 * Configure the SecureStore with the Static values obtained from configLoRaWan.h
 * When force is false, the secure store will be refreshed only if there is no
 * configuration already setup.
 */
#if ITSDK_WITH_SECURESTORE == __ENABLE
itsdk_lorawan_return_t itsdk_lorawan_resetFactoryDefaults(bool force) {
	uint8_t buffer[16];
	if ( force || itsdk_secstore_readBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, buffer) != SS_SUCCESS ) {
		uint8_t de[8] = ITSDK_LORAWAN_DEVEUI;
		uint8_t ap[8] = ITSDK_LORAWAN_APPEUI;
		for ( int i = 0 ; i< 8 ; i++) {
			buffer[ITSDK_SECSTORE_OTAA_DEV_ID+i] = de[i];
			buffer[ITSDK_SECSTORE_OTAA_APP_ID+i] = ap[i];
		}
		itsdk_secstore_writeBlock(ITSDK_SS_LORA_OTAA_DEVEUIAPPEUI, buffer);

		uint8_t appkey[16] = ITSDK_LORAWAN_APPKEY;
		itsdk_secstore_writeBlock(ITSDK_SS_LORA_OTAA_APPKEY, appkey);
	}
	return LORAWAN_RETURN_SUCESS;
}
#else
itsdk_lorawan_return_t itsdk_lorawan_resetFactoryDefaults(bool force) {
	return LORAWAN_RETURN_SUCESS;
}
#endif




// @TODO add the ABP configuration extraction ...
#warning "The ABP Configuration and setup is missing"


itsdk_lorawan_return_t itsdk_lorawan_changeDefaultRate(uint8_t newRate) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_changeDefaultRate\r\n"));
	lorawan_driver_LORA_ChangeDefaultRate(newRate);
	return LORAWAN_RETURN_SUCESS;
}

itsdk_lorawan_rssisnr_t itsdk_lorawan_getLastRssiSnr(int16_t *rssi, uint8_t *snr){
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getLastRssiSnr\r\n"));
	return lorawan_driver_LORA_GetLastRssiSnr(rssi,snr);
}

itsdk_lorawan_return_t itsdk_lorawan_setTxPower(itsdk_lorawan_txpower txPwr) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_setTxPower\r\n"));
	return (lorawan_driver_LORA_SetTxPower(txPwr))?LORAWAN_RETURN_SUCESS:LORAWAN_RETURN_FAILED;
}

itsdk_lorawan_return_t itsdk_lorawan_getTxPower(itsdk_lorawan_txpower * power) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getTxPower\r\n"));
	*power = lorawan_driver_LORA_GetTxPower();
	return LORAWAN_RETURN_SUCESS;
}


itsdk_lorawan_return_t itsdk_lorawan_getDownlinkFrameCounter(uint16_t * counter) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getDownlinkFrameCounter\r\n"));
	*counter = lorawan_driver_LORA_GetDownlinkFrameCounter();
	return LORAWAN_RETURN_SUCESS;
}


itsdk_lorawan_return_t itsdk_lorawan_getUplinkFrameCounter(uint16_t * counter) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getUplinkFrameCounter\r\n"));
	*counter = lorawan_driver_LORA_GetUplinkFrameCounter();
	return LORAWAN_RETURN_SUCESS;
}

itsdk_lorawan_return_t itsdk_lorawan_getNextUplinkFrameCounter(uint16_t * counter) {
	LOG_INFO_LORAWANSTK(("itsdk_lorawan_getNextUplinkFrameCounter\r\n"));
	*counter = lorawan_driver_LORA_GetUplinkFrameCounter();
	*counter = *counter+1;
	return LORAWAN_RETURN_SUCESS;
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
