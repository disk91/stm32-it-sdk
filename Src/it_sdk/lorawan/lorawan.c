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


#define HEX16(X)  X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7],X[8],X[9], X[10],X[11], X[12],X[13], X[14],X[15]
#define HEX8(X)   X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7]


/**
 * Return a 8 bytes uniq Id for the device
 */
void itsdk_lorawan_getUniqId(uint8_t *id ) {
	itsdk_getUniqId(id, 8);
	return;
}


/**
 * Callback function on JOIN Success
 */
void itsdk_lorawan_onConfirmClass_internal(DeviceClass_t class) {
	itsdk_lorawan_onConfirmClass((itsdk_lorawan_dev_class)class);
}

__weak void itsdk_lorawan_onConfirmClass(itsdk_lorawan_dev_class class) {
   log_info("[LoRaWAN] Class switch to %d confirmed\r\n","ABC"[class]);
}


/**
 * Callback on Uplink Ack from the Network server
 */
__weak void lorawan_driver_onUplinkAckConfirmed() {
    log_info("[LoRaWAN] Network Server \"ack\" an uplink data confirmed message transmission\r\n");
}


/**
 * Init the LoRaWan Stack
 */
static uint8_t devEui[8] = ITSDK_LORAWAN_DEVEUI;
static uint8_t appEui[8] = ITSDK_LORAWAN_APPEUI;
static uint8_t appKey[16] = ITSDK_LORAWAN_APPKEY;

itsdk_lorawan_init_t itsdk_lorawan_setup(uint16_t region) {
	log_info("itsdk_lorawan_setup\r\n");
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

	if ( __config.JoinType == __LORAWAN_OTAA ) {
		log_debug( "OTAA\n\r");
		log_debug( "DevEui= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n\r", HEX8(__config.devEui));
		log_debug( "AppEui= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n\r", HEX8(__config.config.otaa.appEui));
		log_debug( "AppKey= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n\r", HEX16(__config.config.otaa.appKey));
	} else if (__config.JoinType == __LORAWAN_ABP) {
		log_debug( "ABP\n\r");
		log_debug( "DevEui= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n\r", HEX8(__config.devEui));
		log_debug( "DevAdd=  %08X\n\r", __config.config.abp.devAddr) ;
		log_debug( "NwkSKey= %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX16(__config.config.abp.nwkSEncKey));
		log_debug( "AppSKey= %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX16(__config.config.abp.appSKey));
	}

	lorawan_driver_LORA_Init(&__config);
	bzero(&__config,sizeof(__config));

	return LORAWAN_INIT_SUCESS;
}

/**
 * Join Process
 * The process can be
 *  - synchronous : will return after connection success or failed ( return - LORAWAN_JOIN_PENDING )
 *  - asynchronous : will return immediately after transmission and reception will be managed over timer & interrupt
 *                   it allows to switch low power but is a higher risk in term of timing respect.
 *                   returns (LORAWAN_JOIN_SUCCESS/LORAWAN_JOIN_FAILED)
 */
itsdk_lorawan_join_t itsdk_lorawan_join(itsdk_lorawan_run_t runMode) {
	log_info("itsdk_lorawan_join\r\n");
	return lorawan_driver_LORA_Join(runMode);
}


/**
 * Send a LoRaWAN frame containing the Payload of the given payloadSize bytes on given port.
 * The dataRate can be set.
 * Confirmation mode (downlink) can be specified.
 * The send can be synchronous or asynchronous.
 * In synchronous mode the status will be
 *   - LORAWAN_SEND_SENT/LORAWAN_SEND_ACKED on success
 *   - LORAWAN_SEND_NOT_JOINED / LORAWAN_SEND_DUTYCYCLE / LORAWAN_SEND_FAILED on error
 * In asynchronous mode the status will be
 *   - LORAWAN_SEND_RUNNING
 */
itsdk_lorawan_send_t itsdk_lorawan_send(
		uint8_t * payload,
		uint8_t   payloadSize,
		uint8_t   port,
		uint8_t	  dataRate,
		itsdk_lorawan_sendconf_t confirm,
		itsdk_lorawan_run_t runMode
) {

	return lorawan_driver_LORA_Send(payload,payloadSize,port,dataRate,confirm,runMode);

}


/**
 * Return true once the device has joined the the network
 */
bool itsdk_lorawan_hasjoined() {
	return true; //(__loraWanState.hasJoined);
}






#endif
