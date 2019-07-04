/* ==========================================================
 * lorawan.h - Driver for making semtech LoRaWan stack
 *             working in a synchronous mode for interfacing
 *             with itsdk abstraction layer
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 03 fev. 2019
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
 */

#ifndef IT_SDK_DRIVERS_LORAWAN_H_
#define IT_SDK_DRIVERS_LORAWAN_H_

#include <it_sdk/lorawan/lorawan.h>

typedef enum lorawan_driver_joinState_e {
	LORAWAN_STATE_NONE = 0,
	LORAWAN_STATE_INITIALIZED,
	LORAWAN_STATE_JOINING,
	LORAWAN_STATE_JOIN_SUCCESS,
	LORAWAN_STATE_JOIN_FAILED,

	LORAWAN_STATE_END
} lorawan_driver_joinState;

typedef enum lorawan_driver_sendState_e {
	LORAWAN_SEND_STATE_NONE = 0,
	LORAWAN_SEND_STATE_RUNNING,
	LORAWAN_SEND_STATE_SENT,
	LORAWAN_SEND_STATE_ACKED,
	LORAWAN_SEND_STATE_ACKED_NO_DOWNLINK,
	LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK,
	LORAWAN_SEND_STATE_ACKED_DOWNLINK_PENDING,
	LORAWAN_SEND_STATE_NOTACKED,
	LORAWAN_SEND_STATE_FAILED,
	LORAWAN_SEND_STATE_DUTYCYCLE,

	LORAWAN_SEND_STATE_END
} lorawan_driver_sendState;


#define	LORAWAN_DRIVER_INVALID_RSSI	0xFFFF

/**
 * This structure maintains the abstraction layer internal state
 */
typedef struct lorawan_driver_state_s {
	volatile lorawan_driver_joinState	joinState;			// current state for the stack
	volatile lorawan_driver_sendState	sendState;			// current state for running send

	uint32_t							joinTime;			// Time in S when the device has confirmed last joined
	bool								reqPending;			// Indicate a processing is pending

    uint16_t							upLinkCounter;		// Uploing Frame counter
    uint16_t							downlinkCounter;	// Downlink Frame counter
    int16_t								lastRssi;			// Last Ack / Rx Rssi value
    int8_t								lastSnr;			// Last Ack / Rx Snr value
    uint8_t								lastRetries;		// Number of retry on last acked transmision

	uint8_t 							txDatarate;			// default transmission rate
    uint8_t   							JoinType;			// OTAA / ABP
    union {
		   struct s_otaa1 {
			   uint8_t 					devEui[8];			// devEui
			   uint8_t 					appEui[8];			// appEui
		   } otaa;
		   struct s_abp1 {

		   } abp;
    } join;

} lorawan_driver_state_t;


/**
 * This structure contains the LoRaStack configuration to be transfered
 * at the LoRaInit. Clean it after init as it contains sensitive informations
 */
typedef struct lorawan_driver_config_s {
    bool 		adrEnable;				// ADR -> Adaptative data rate On/Off
    int8_t 		txDatarate;				// Tx DataRate
    bool 		enablePublicNetwork;	// Public or private network
    uint16_t	region;					// Region to be set

    // __LORAWAN_OTAA or __LORAWAN_ABP
    uint8_t   JoinType;
    uint8_t * devEui;						// Dev EUI pointer (8B)
    union {
	   struct s_otaa {
		   uint8_t * appEui;				// OTAA App EUI pointer (8B)
		   uint8_t * appKey;		   		// OTAA App Key pointer (16B)
		   uint8_t * nwkKey;		   		// OTAA Nwk Key pointer (16B)
	   } otaa;
	   struct s_abp {
		   uint8_t * FNwkSIntKey;			// Nwk Internal Session Key
		   uint8_t * SNwkSIntKey;			//
		   uint8_t * nwkSEncKey;			// Nwk Session Key
		   uint8_t * appSKey;				// App Session Key
		   uint32_t  devAddr;				// Dev addr - When 0, the device address will be generated randomly
	   } abp;
    } config;
} lorawan_driver_config_t;


/**
 * API
 */

void lorawan_driver_LORA_Init(
		lorawan_driver_config_t * config
);

itsdk_lorawan_join_t lorawan_driver_LORA_Join(
		itsdk_lorawan_run_t 	  runMode
);

itsdk_lorawan_send_t lorawan_driver_LORA_Send(
		uint8_t					* payload,
		uint8_t					  size,
		uint8_t					  port,
		uint8_t					  dataRate,
		itsdk_lorawan_sendconf_t  isTxConfirmed,
		uint8_t					  retry,
		itsdk_lorawan_run_t 	  runMode,
		uint8_t					* rPort,				// for sync mode only - on reception - Port
		uint8_t					* rSize,				// for sync mode only - on reception - DataSize - contains maxSize on input
		uint8_t					* rData					// for sync mode only - on reception - Data (bcopied)
);


lorawan_driver_sendState lorawan_driver_LORA_getSendState();
lorawan_driver_joinState lorawan_driver_LORA_getJoinState();
void lorawan_driver_LORA_ChangeDefaultRate(uint8_t newRate);
itsdk_lorawan_rssisnr_t lorawan_driver_LORA_GetLastRssiSnr(int16_t *rssi, uint8_t *snr);
bool lorawan_driver_LORA_SetTxPower(itsdk_lorawan_txpower txPwr );
itsdk_lorawan_txpower lorawan_driver_LORA_GetTxPower();
uint16_t lorawan_driver_LORA_GetDownlinkFrameCounter();
uint16_t lorawan_driver_LORA_GetUplinkFrameCounter();
itsdk_lorawan_channel_t lorawan_driver_LORA_RemoveChannel(uint8_t channelId);
itsdk_lorawan_channel_t lorawan_driver_LORA_AddChannel(
		uint8_t		channelId,
		uint32_t 	frequency,
		uint32_t	rx1Frequency,
		uint8_t		minDataRate,
		uint8_t		maxDataRate,
		uint8_t		band
);
itsdk_lorawan_channel_t lorawan_driver_LORA_SelectChannels(uint16_t region, uint16_t * channels );
void lorawan_driver_loop();

// ===========================================================================
// Callback API -> can be override
// ===========================================================================
void lorawan_driver_onTxNeeded();
void lorawan_driver_onSendSuccessAckFailed();
void lorawan_driver_onSendAckSuccess();
void lorawan_driver_onSendSuccess();
void lorawan_driver_onJoinSuccess();
void lorawan_driver_onJoinFailed();
void lorawan_driver_onDataReception(uint8_t port, uint8_t * data, uint8_t size);
void lorawan_driver_onPendingDownlink();


// ===========================================================================
// Particular function with correct default setting, can be override when the
// default way the sdk works is modified.
// ===========================================================================
uint16_t lorawan_driver_temperature();
uint8_t lorawan_driver_battery_level();
void lorawan_driver_waitUntilEndOfExecution();

#endif // IT_SDK_DRIVERS_LORAWAN_H_
