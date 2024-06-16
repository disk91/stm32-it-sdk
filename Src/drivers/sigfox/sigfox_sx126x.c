/* ==========================================================
 * sigfox_sx126x.c - sigfox / itsdk integration
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
 *  Created on: 14 june 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_SIGFOX_LIB == __ENABLE && ITSDK_SIGFOX_LIB == __SIGFOX_SX126X
#include <string.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/eeprom/sdk_state.h>
#include <drivers/sx126x/sigfox_sx126x.h>
#include "sigfox_ep_api.h"
#include "sigfox_rc.h"


// ---------------------------------------------------------
// Init library
//
// ---------------------------------------------------------
itsdk_sigfox_init_t sx126x_sigfox_init( void ) {

	LOG_ERROR_SFXSX126X(("sx126x_sigfox_init\r\n"));

	SIGFOX_EP_API_config_t SIGFOX_EP_API_config;
	switch (itsdk_state.sigfox.rcz) {
	  #ifdef RC1_ZONE
		case SIGFOX_RCZ1:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC1;
			break;
	  #endif

	  #ifdef RC2_ZONE
		case SIGFOX_RCZ2:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC2;
			break;
	  #endif

	  #ifdef RC3C_ZONE
		case SIGFOX_RCZ3C:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC3C;
			break;
	  #endif

	  #ifdef RC3D_ZONE
		case SIGFOX_RCZ3D:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC3D;
			break;
	  #endif

	  #ifdef RC4_ZONE
		case SIGFOX_RCZ4:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC4;
			break;
	  #endif

	  #ifdef RC5_ZONE
		case SIGFOX_RCZ5:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC5;
			break;
	  #endif

	  #ifdef RC6_ZONE
		case SIGFOX_RCZ6:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC6;
			break;
	  #endif

	  #ifdef RC7_ZONE
		case SIGFOX_RCZ7:
			SIGFOX_EP_API_config.rc = &SIGFOX_RC7;
			break;
	  #endif
		default:
			LOG_ERROR_SFXSX126X(("[ERROR] Unsupported Sigfox RCZ\r\n"));
			return SIGFOX_INIT_PARAMSERR;
	}

	SIGFOX_EP_API_status_t r = SIGFOX_EP_API_open(&SIGFOX_EP_API_config);
	if ( r != SIGFOX_EP_API_SUCCESS ) {
		LOG_ERROR_SFXSX126X(("[ERROR] Unable to open Sigfox %d\r\n",r));
		return SIGFOX_INIT_FAILED;
	}

	return SIGFOX_INIT_SUCESS;
}

// ---------------------------------------------------------
// Get the last Rssi from downlink, as this is not stored in
// the sigfox library, let's store it from the last downlink
// we had.
//
// ---------------------------------------------------------
#ifdef BIDIRECTIONAL
 static int16_t __sx126x_last_rssi = __SX126X_RSSI_NONE;
#endif
itsdk_sigfox_init_t sx126x_sigfox_getRssi(int16_t * rssi) {
	#ifdef BIDIRECTIONAL
	 if ( __sx126x_last_rssi == __SX126X_RSSI_NONE ) return SIGFOX_INIT_FAILED;
	 else *rssi = __sx126x_last_rssi;
	 return SIGFOX_INIT_SUCESS;
	#else
	 return SIGFOX_INIT_FAILED;
	#endif
}

// ----------------------------------------------------------
// Configure the messages for sigfox lib
// common parameters, used by all type of frames
// ----------------------------------------------------------
itsdk_sigfox_init_t sx126x_sigfox_tx_common_config(
		SIGFOX_EP_API_common_t * common_parameters,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		bool ack,
		uint8_t rcz,
		uint8_t sfxKey
) {
  #if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	#ifndef UL_BIT_RATE_BPS
	  switch (speed) {
		case SIGFOX_SPEED_100: common_parameters->ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS; break;
		default: // now 600 works everywhere
		case SIGFOX_SPEED_600: common_parameters->ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS; break;
	  }
	#endif
	#ifndef TX_POWER_DBM_EIRP
	 common_parameters->tx_power_dbm_eirp = power;
	#endif
	#ifndef SINGLE_FRAME
	 common_parameters->number_of_frames = repeat;
	 #ifndef T_IFU_MS
	  switch ( rcz ) {
	  	  default:
	  		common_parameters->t_ifu_ms = (ack)?ITSDK_SIGFOX_IF_TXRX_RCZ1:ITSDK_SIGFOX_IF_TX_RCZ1;
	  		break;
	  	  case SIGFOX_RCZ3C:
	  	  case SIGFOX_RCZ3D:
		  	common_parameters->t_ifu_ms = (ack)?ITSDK_SIGFOX_IF_TXRX_RCZ3:ITSDK_SIGFOX_IF_TX_RCZ3;
		  	break;
	  }
	 #endif
	#endif
	#ifdef PUBLIC_KEY_CAPABLE
	  switch ( sfxKey ) {
		  default:
		  case SIGFOX_KEY_PRIVATE: common_parameters->ep_key_type = SIGFOX_EP_KEY_PRIVATE; break;
		  case SIGFOX_KEY_PUBLIC: common_parameters->ep_key_type = SIGFOX_EP_KEY_PUBLIC; break;
	  }
	#endif
 #endif
 return SIGFOX_INIT_SUCESS;
}

// ----------------------------------------------------------
// Configure the messages for sigfox lib
// send parameters, used by all type of uplink data frames
// ----------------------------------------------------------

itsdk_sigfox_init_t sx126x_sigfox_tx_config(
		SIGFOX_EP_API_application_message_t * m,
		uint8_t * buf, 		// null for bit frame
		bool value,			// for bit frame
		uint8_t len,		// 0 / 1 for bit frame / 2 for OOB frame with buf == NULL
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		bool ack,
		uint8_t rcz,
		uint8_t sfxKey
) {
	sx126x_sigfox_tx_common_config(&m->common_parameters,repeat,speed,power,ack,rcz,sfxKey);

#warning "TODO - ASYNC MANAGEMENT"
  #ifdef ASYNCHRONOUS		// we don't really do Asynchronous here... let see it later
	m->uplink_cplt_cb = NULL;
	#ifdef BIDIRECTIONAL
	m->downlink_cplt_cb = NULL;
	#endif
	m->message_cplt_cb = NULL;
  #endif
  #ifndef T_CONF_MS
	m->t_conf_ms = ITSDK_SIGFOX_DWNCNF_DELAY;	// Delay between downlink frame reception and uplink confirmation (OOB message)
  #endif

  if ( buf == NULL) {
	  switch (len) {
	  default:
	  case 0:
		  m->type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
	  	  break;
	  case 1:
		  m->type = (value)?SIGFOX_APPLICATION_MESSAGE_TYPE_BIT1:SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0;
		  break;
	  }
  } else m->type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;

  #if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
  	  m->ul_payload = buf;
  #endif
  #ifndef UL_PAYLOAD_SIZE
	  m->ul_payload_size_bytes = len;
  #endif
  #ifdef BIDIRECTIONAL
	  m->bidirectional_flag = (ack)?SFX_TRUE:SFX_FALSE;
  #endif

  return SIGFOX_INIT_SUCESS;
}

#endif // ITSDK_WITH_SIGFOX_LIB == __ENABLE && ITSDK_SIGFOX_LIB == __SIGFOX_SX126X
