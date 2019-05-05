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
#include <stdint.h>


typedef struct {
	uint8_t		activeNetwork;			// Currently active network see __ACTIV_NETWORK_*

} itsdk_state_t;

extern itsdk_state_t itsdk_state;


void itsdk_state_init();

#endif // IT_SDK_EEPROM_SDK_STATE_H_
