/* ==========================================================
 * sdk_state.h - structure used for the SDK dynamic state
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 04 May 2019
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
#ifndef IT_SDK_EEPROM_SDK_STATE_H_
#define IT_SDK_EEPROM_SDK_STATE_H_

#include <it_sdk/config.h>
#include <it_sdk/wrappers.h>
#include <stdint.h>
#include <stdbool.h>


typedef struct {
	itsdk_reset_cause_t lastResetCause;			// Reason of the last reset cause
	uint8_t				activeNetwork;			// Currently active network see __ACTIV_NETWORK_*
	uint64_t			lastWakeUpTimeUs;		// Store time from last reset in uS (use by adc to ensure reference are stabel before read)
	#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	struct {
		bool					initialized;
		uint8_t					rcz;
		int8_t					current_power;
		uint16_t				current_speed;
	} sigfox;
	#endif
} itsdk_state_t;

extern itsdk_state_t itsdk_state;


void itsdk_state_init();
void itsdk_print_state();

#endif // IT_SDK_EEPROM_SDK_STATE_H_
