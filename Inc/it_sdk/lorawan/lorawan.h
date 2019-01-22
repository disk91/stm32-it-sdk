/* ==========================================================
 * lorawan.h - Abstraction layer for lorawan libraries
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
#ifndef IT_SDK_LORAWAN_H_
#define IT_SDK_LORAWAN_H_

typedef enum {
	LORAWAN_INIT_SUCESS = 0,
	LORAWAN_INIT_FAILED
} itsdk_lorawan_init_t;


// ===============================================================
// TO BE OVERRIDDED
// ===============================================================

/**
 * Return a batteryLevel from 1 to 254
 * 1 = VBAT_MIN
 * 254 = VBAT_MAX
 */
uint8_t itsdk_lorawan_battery_level();

// Function automatically fired on data reception
void itsdk_lorawan_onDataReception(uint8_t port, uint8_t * data, uint8_t size);
// Function automatically fired on join success
void itsdk_lorawan_onJoinSuccess();
// Function automatically fired on device class change confirmation
void itsdk_lorawan_onConfirmClass(DeviceClass_t class);
// Function automatically fired when the network is requesting a Uplink transmission
void itsdk_lorawan_onTxNeeded();
// Function automatically fired when the network server has confirmed ack reception
void itsdk_lorawan_uplinkAckConfirmed();

#endif
