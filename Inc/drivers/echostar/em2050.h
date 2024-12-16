/* ==========================================================
 * em2050.h - implementation echostar EM2050 module
 * ----------------------------------------------------------
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
 * ---------------------------------------------------------
 *
 *  Created on: 29 November 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#ifndef IT_SDK_DRIVERS_EM2050_H_
#define IT_SDK_DRIVERS_EM2050_H_

#include <it_sdk/config.h>
#include <it_sdk/logger/logger.h>

#if ITSDK_DRIVERS_EM2050 == __ENABLE

#define _EM2050_INITTMOUT_DEFAULT	1000
#define _EM2050_TMOUT_DEFAULT		500


typedef enum {
	JM_ABP = 0,		// doc says Manual
	JM_OTAA = 1,	// doc says Auto OTA

	JM_UNKNOWN = 0xFF
} es_joinMode_t;

typedef enum {
	JS_DISCONNECTED=0,
	JS_JOINED = 1,

	JS_UNKNOWN = 0xFF
} es_joinStatus_t;

typedef enum {
	REGION_MSS_S = 0,
	REGION_EU868 = 1,
	REGION_US915 = 2,

	REGION_UNKNOWN = 0xFF
} es_region_t;

typedef enum {
	ADR_OFF = 0,
	ADR_ON = 1,

	ADR_UNKNOWN = 0xFF
} es_adr_t;

typedef enum {
	EM_CLASS_A = 0,
	EM_CLASS_B = 1,
	EM_CLASS_C = 2,

	EM_CLASS_UNKNOWN = 0xFF
} es_class_t;

typedef enum {
	URG_NONE = 0,		// Normal
	URG_LVL1 = 1,		// Requires a specific contract
	URG_LVL2 = 2,
	URG_LVL3 = 3,
	URG_LVL4 = 4,
	URG_LVL5 = 5,
	URG_LVL6 = 6,
	URG_LVL7 = 7,
	URG_LVL8 = 8,

	URG_UNKNOWN = 0xFF
} es_urgent_t;

#define TX_POWER_ERROR 	0xFF
#define ANT_GAIN_ERROR  -128


typedef enum {
	SF_LRFHSS_1_3 = 0,
	SF_LRFHSS_2_3 = 1,
	SF_12 = 3,
	SF_11 = 4,
	SF_10 = 5,

	SF_UNKNOWN = 0xFF
} es_sf_t;


#if (ITSDK_LOGGER_MODULE & __LOG_MOD_ECHOSTAR) > 0
	#define _LOG_ECHOSTAR_DEBUG(x)	log_debug x
	#define _LOG_ECHOSTAR_INFO(x)	log_info x
	#define _LOG_ECHOSTAR_WARN(x)	log_warn x
	#define _LOG_ECHOSTAR_ERROR(x)	log_error x
#else
	#define _LOG_ECHOSTAR_DEBUG(x)
	#define _LOG_ECHOSTAR_INFO(x)
	#define _LOG_ECHOSTAR_WARN(x)
	#define _LOG_ECHOSTAR_ERROR(x)
#endif


// Init / reinit the GPIO & dediacted Uart
void echoStarInit();
void echoStarInitOff();
itsdk_bool_e echoStarSoftReset();

// wake modem up
itsdk_bool_e echoStarWakeUpModem();

// just display what the modem send (for debugging)
void echoStarshowMessages();

// return true when the modem is responding on the serial line
itsdk_bool_e isEchoStarModemresponding();
uint16_t echoStarGetVersion();
itsdk_bool_e echoStarGetDeviceId(uint8_t * devEui);
itsdk_bool_e echoStarGetNetworkKey(uint8_t * key);
itsdk_bool_e echoStarSetNetworkKey(uint8_t * ntwKey);
itsdk_bool_e echoStarGetJoinEui(uint8_t * joinEui);
itsdk_bool_e echoStarSetJoinEui(uint8_t * joinEui);
itsdk_bool_e echoStarGetPinCode(uint8_t * pinCode);
es_joinMode_t echoStarGetJoinMode();
itsdk_bool_e echoStarSetJoinMode(es_joinMode_t joinMode);
es_joinStatus_t echoStarGetJoinStatus();
es_region_t echoStarGetRegion();
itsdk_bool_e echoStarSetRegion(es_region_t region);
uint8_t echoStarGetMssTxPower();
itsdk_bool_e echoStarSetMssTxPower(uint8_t power);
int8_t echoStarGetMssAntGain();
itsdk_bool_e echoStarSetMssAntGain(int8_t gain);
uint8_t echoStarGetSGhTxPower();
itsdk_bool_e echoStarSetSGhTxPower(uint8_t power);
es_adr_t echoStarGetAdrState();
itsdk_bool_e echoStarSetAdrState(es_adr_t adrState);
itsdk_bool_e echoStarGetUserKey(uint8_t * key);
itsdk_bool_e echoStarSetUserKey(uint8_t * ntwKey);
itsdk_bool_e echoStarEncrypt(uint8_t * payload, uint8_t size);
itsdk_bool_e echoStarDecrypt(uint8_t * payload, uint8_t size);
es_class_t echoStarGetDeviceClass();
itsdk_bool_e echoStarSetDeviceClass(es_class_t devClass);
uint32_t echoStarGetKeepAliveTimeMs();
itsdk_bool_e echoStarSetKeepAliveTimeMs(uint32_t kaTimeMs);
itsdk_bool_e echoStarGetSatPowerAndSf(uint8_t * power, es_sf_t * sf);
itsdk_bool_e echoStarFactoryReset();
void echoStarSwitchActive();
void echoStarSwitchSleep();

itsdk_bool_e echoStarJoin();
itsdk_bool_e echoStarSendBytes(uint8_t port, uint8_t * msg, uint8_t sz, itsdk_bool_e ack, es_urgent_t urg, itsdk_bool_e blocking);


void em2050_customSerialInit();
void em2050_customSerialConnect();
void em2050_customSerial_write(uint8_t * bytes,uint16_t len);
void em2050_customSerial_println(char * msg);
serial_read_response_e em2050_customSerial_read(char * ch);

#endif // ITSDK_DRIVERS_EM2050

#endif // IT_SDK_DRIVERS_EM2050_H_

