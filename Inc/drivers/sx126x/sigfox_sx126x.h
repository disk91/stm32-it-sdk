/* ==========================================================
 * sigfox_sx126x.h - implementation for sigfox library, general headers
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
 *  Created on: 11 june 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#ifndef IT_SDK_DRIVERS_SX126X_SIGFOX_H_
#define IT_SDK_DRIVERS_SX126X_SIGFOX_H_

#include <it_sdk/config.h>
#include <it_sdk/logger/logger.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX126X)
#include "sigfox_types.h"
#include "sigfox_ep_api.h"

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
#define LOG_INFO_SFXSX126X(x)		log_info x
#define LOG_WARN_SFXSX126X(x) 		log_warn x
#define LOG_ERROR_SFXSX126X(x)		log_error x
#define LOG_DEBUG_SFXSX126X(x)		log_debug x
#else
#define LOG_INFO_SFXSX126X(x)
#define LOG_WARN_SFXSX126X(x)
#define LOG_ERROR_SFXSX126X(x)
#define LOG_DEBUG_SFXSX126X(x)
#endif

#define __SX126X_RXTX_OFF		0x00
#define __SX126X_RX 			0x01
#define __SX126X_TX 			0x02

#define __SX126X_SMPS_DRV_20  	((uint8_t) ((0x0)<<1))
#define __SX126X_SMPS_DRV_40  	((uint8_t) ((0x1)<<1))
#define __SX126X_SMPS_DRV_60  	((uint8_t) ((0x2)<<1))
#define __SX126X_SMPS_DRV_100 	((uint8_t) ((0x3)<<1))
#define __SX126X_SMPS_DRV_MASK 	((uint8_t) ((0x3)<<1))

#define __SX126X_REG_SMPSC2R	0x0923
#define __SX126X_REG_OCP 		0x08E7
#define __SX126X_REG_TX_CLAMP 	0x08D8

#define __SX126X_RSSI_NONE		-16000


void sx126x_onTxComplete_cb(void);
void sx126x_onRxComplete_cb(void);
void sx126x_onMessageComplete_cb(void);
void sx126x_onProcess_cb(void);

void _sx126x_rfSwitchSet(uint8_t paSelected, uint8_t rxTx);
itsdk_sigfox_init_t sx126x_sigfox_init( void );
itsdk_sigfox_init_t sx126x_sigfox_getRssi(int16_t * rssi);
itsdk_sigfox_init_t sx126x_sigfox_tx_common_config(
		SIGFOX_EP_API_common_t * common_parameters,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		bool ack,
		uint8_t rcz,
		uint8_t sfxKey
);
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
);
// conditional presence ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0 && defined BIDIRECTIONAL && !defined ASYNCHRONOUS
itsdk_bool_e sx126x_hasDataReceived();
void sx126x_resetDataReceived();
itsdk_sigfox_init_t sx126x_sigfox_getSeqId(uint16_t * seq);
SIGFOX_EP_API_status_t sx126x_sigfox_process_async();

#endif

#endif // IT_SDK_DRIVERS_SX126X_SIGFOX_H_
