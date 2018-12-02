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


void s2lp_hwInit();
void s2lp_init();
void s2lp_shutdown();
void s2lp_wakeup();



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


#define S2LP_RANGE_EXT_NONE 		0
#define S2LP_RANGE_SKYWORKS_868		1



// Config standardization
void s2lp_loadConfiguration(
		s2lp_config_t * s2lpConf
);

void s2lp_applyConfig(
	s2lp_config_t * cnf
);

void s2lp_printConfig(
		s2lp_config_t * s2lpConf
);

#endif /* IT_SDK_DRIVERS_S2LP_H_ */
