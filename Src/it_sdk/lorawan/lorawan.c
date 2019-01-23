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
#include <drivers/lorawan/core/lora.h>
#include <it_sdk/lorawan/lorawan.h>


/**
 * Return a batteryLevel from 1 to 254
 * 1 = VBAT_MIN
 * 254 = VBAT_MAX
 */
__weak uint8_t itsdk_lorawan_battery_level() {
	 uint16_t mv = adc_getVBat();
	 if ( mv <= ITSDK_VBAT_MIN ) return 1;
	 if ( mv >= ITSDK_VBAT_MAX ) return 254;
	 return (( (uint32_t) (mv - ITSDK_VBAT_MIN)*ITSDK_VBAT_MAX) /(ITSDK_VBAT_MAX-ITSDK_VBAT_MIN) );
}

/**
 * Return the temperature
 * temperature in fixed decimal : 8b integer + 8b decimal
 */
uint16_t itsdk_lorawan_temerature() {
	int16_t t = adc_getTemperature();
	t = (int16_t)(((int32_t)t << 8)/100);
	return (uint16_t)t;
}

/**
 * Return a 8 bytes uniq Id for the device
 */
void itsdk_lorawan_getUniqId(uint8_t *id ) {
	itsdk_getUniqId(id, 8);
	return;
}

/**
 * Callback function on data reception
 * this function is based on a lorawan stack internal structure
 */
void itsdk_lorawan_ReceiveData(lora_AppData_t *AppData)
{
	itsdk_lorawan_onDataReception(
			AppData->Port,
			AppData->Buff,
			AppData->BuffSize
	);
}


/**
 * Callback function on Data Reception
 * this function is based on standard element and can be overrided.
 */
__weak void itsdk_lorawan_onDataReception(uint8_t port, uint8_t * data, uint8_t size) {
   log_info("[LoRaWAN] data received %d bytes\r\n",size);
}

/**
 * Callback function on JOIN Success
 */
__weak void itsdk_lorawan_onJoinSuccess() {
   log_info("[LoRaWAN] Join success\r\n");
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
 * Callback function requesting transmission
 */
__weak void itsdk_lorawan_onTxNeeded() {
   log_info("[LoRaWAN] Network Server is asking for an uplink transmission\r\n");
}

/**
 * ??? to be clarified
 */
void itsdk_lorawan_macProcessNotify(void) {
  log_info("[LoRaWAN] Mac Process Notify\r\n");

  //LoraMacProcessRequest=LORA_SET;
}

/**
 * Callback on Uplink Ack from the Network server
 */
__weak void itsdk_lorawan_uplinkAckConfirmed() {
    log_info("[LoRaWAN] Network Server \"ack\" an uplink data confirmed message transmission\r\n");
}

itsdk_lorawan_init_t itsdk_lorawan_setup() {

	static LoRaMainCallback_t LoRaMainCallbacks = { itsdk_lorawan_battery_level,
													itsdk_lorawan_temerature,
													itsdk_lorawan_getUniqId,
													itsdk_getRandomSeed,
													itsdk_lorawan_ReceiveData,
													itsdk_lorawan_onJoinSuccess,
													itsdk_lorawan_onConfirmClass_internal,
													itsdk_lorawan_onTxNeeded,
													itsdk_lorawan_macProcessNotify,
													/*itsdk_lorawan_uplinkAckConfirmed*/};
	static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_ON,
	                                    DR_0,
										#if ITSDK_LORAWAN_NETWORKTYPE == __LORAWAN_NWK_PUBLIC
										   true
										#else
										   false
										#endif
	                                    };
	LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

	return LORAWAN_INIT_SUCESS;
}



#endif
