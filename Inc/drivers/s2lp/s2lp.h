/* ==========================================================
 * s2lp.h - Prototypes for S2LP drivers
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 5 nov. 2018
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

#ifndef IT_SDK_DRIVERS_S2LP_H_
#define IT_SDK_DRIVERS_S2LP_H_

#include <it_sdk/config.h>
#include <it_sdk/sigfox/sigfox.h>

void s2lp_hwInit();
itsdk_sigfox_init_t s2lp_sigfox_init();
itsdk_sigfox_init_t s2lp_sigfox_deinit();
void s2lp_shutdown();
void s2lp_wakeup();

typedef enum {
	S2LP_SIGFOX_ERR_NONE = 0,
	S2LP_SIGFOX_ERR_BREAK,			// Force to break a wait loop
	S2LP_SIGFOX_ERR_LIBINIT,		// Impossible to open Sigfox Lib
	S2LP_SIGFOX_ERR_CONFIG,			// Error on Set Std Config
	S2LP_SIGFOX_ERROR				// Error, no precision


} s2lp_sigfox_ret_t;


#if ITSDK_SIGFOX_NVM_SOURCE	== __SFX_NVM_M95640

typedef struct s2lp_eprom_config_s {
	uint8_t		magic;				// should not be 0 or 0xFF
	uint8_t		xtalFreq;			// 0 - 24MHz, 1 - 25Mhz, 2 - 26Mhz, 3 - 48Mhz, 4 - 50 Mhz, 5 - 52 Mhz, 0xff - custom
	uint8_t		unknown1;
	uint8_t		band;				//
	uint8_t		unknown2;
	uint8_t		range;				//
	uint8_t		unknown3[21];
	uint32_t	customFreq;			// freq int 32
	uint8_t		tcxo;				// 1 when TCXO is present
}  __attribute__ ((__packed__)) s2lp_eprom_config_t;

typedef struct s2lp_eprom_offset_s {
	uint8_t		offset[4];
}  __attribute__ ((__packed__)) s2lp_eprom_offset_t;

#endif

#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
  #define __S2LP__ITSDK_S2LP_CNF_FREQ 		ITSDK_S2LP_CNF_FREQ
  #define __S2LP__ITSDK_S2LP_CNF_OFFSET 	ITSDK_S2LP_CNF_OFFSET
  #define __S2LP__ITSDK_SIGFOX_LBTOFFSET	ITSDK_SIGFOX_LBTOFFSET
  #define __S2LP__ITSDK_S2LP_CNF_TCX0 		ITSDK_S2LP_CNF_TCX0


#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
  #define __S2LP__ITSDK_S2LP_CNF_FREQ 		s2lp_driver_config.tcxoFreq
  #define __S2LP__ITSDK_S2LP_CNF_OFFSET 	s2lp_driver_config.freqOffset
  #define __S2LP__ITSDK_SIGFOX_LBTOFFSET	ITSDK_SIGFOX_LBTOFFSET
  #define __S2LP__ITSDK_S2LP_CNF_TCX0 		ITSDK_S2LP_CNF_TCX0

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_HEADERS
  #define __S2LP__ITSDK_S2LP_CNF_FREQ 		ITSDK_S2LP_CNF_FREQ
  #define __S2LP__ITSDK_S2LP_CNF_OFFSET 	ITSDK_S2LP_CNF_OFFSET
  #define __S2LP__ITSDK_SIGFOX_LBTOFFSET	ITSDK_SIGFOX_LBTOFFSET
  #define __S2LP__ITSDK_S2LP_CNF_TCX0 		ITSDK_S2LP_CNF_TCX0

#endif


typedef struct {
	uint8_t				lastReadRssi;		// Last RSSI read from S2LP (can be an invalid message) rssi = value -146
	uint8_t				lastReceptionRssi;  // Last RSSI corresponding to a valid downlink rssi = value - 146

	#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640 || ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_HEADERS
	uint32_t			deviceId;						// DeviceId
	uint8_t				initialPac[8];					// Initial PAC
	uint8_t				key[16];						// Sigfox Key
	#endif
	#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	int32_t  			freqOffset;						// Offset
	uint32_t			tcxoFreq;						// Tcxo frequency
 	#endif

} s2lp_drivers_config_t;

extern s2lp_drivers_config_t s2lp_driver_config;

/*
typedef struct s2lp_config_s {
	uint32_t	xtalFreq;			// freq int 32
	uint8_t		tcxo;				// 1 when TCXO is present
	uint8_t		range;
	uint8_t		band;
	uint8_t 	rcz;				// sigfox RCZ
	int32_t		offset;
	int32_t 	rssiOffset;

	uint32_t 	id;					// sigfox ID
	uint8_t  	pac[8];				// sigfox initial Pac
	uint8_t  	key[16];			// sigfox Key (this must be aligned on 32b block)
	uint8_t  	aux[16];			// sigfox Aux
	uint16_t	seqId;				// last sent seqId

	uint8_t		low_power_flag;		// switch to low power during S2LP processing
	uint8_t		payload_encryption; // encrypt the payload
	uint8_t		lastReadRssi;		// Last RSSI read from S2LP (can be an invalid message) rssi = value -146
	uint8_t		lastReceptionRssi;  // Last RSSI corresponding to a valid downlink rssi = value - 146

}  s2lp_config_t;
*/




// Config standardization
void s2lp_loadConfiguration();
s2lp_sigfox_ret_t s2lp_sigfox_getSeqId( uint16_t * seqId );

#endif /* IT_SDK_DRIVERS_S2LP_H_ */
