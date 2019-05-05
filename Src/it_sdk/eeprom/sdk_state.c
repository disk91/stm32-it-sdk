/* ==========================================================
 * sdk_state.c - structure used for the SDK dynamic state
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
#include <it_sdk/config.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/eeprom/sdk_state.h>
itsdk_state_t itsdk_state;

void itsdk_state_init() {
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
	itsdk_state.activeNetwork = (uint8_t)itsdk_config.sdk.activeNetwork;
#else
	itsdk_state.activeNetwork = ITSDK_DEFAULT_NETWORK;
#endif
	return;
}
