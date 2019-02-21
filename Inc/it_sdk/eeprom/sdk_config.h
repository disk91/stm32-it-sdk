/* ==========================================================
 * sdk_config.h - structure used for the SDK configuration
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 21 févr. 2019
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
#ifndef IT_SDK_EEPROM_SDK_CONFIG_H_
#define IT_SDK_EEPROM_SDK_CONFIG_H_

#include <it_sdk/config.h>
#include <stdint.h>

#if ITSDK_WITH_CONFIGURATION_NVM == __ENABLE
	#include <it_sdk/configNvm.h>
#endif


/**
 * SDK specific structure
 * size aligned on 32b
 */
typedef struct {
	uint8_t				version;
	uint16_t			size;

	// ----------- LoRaWan settings --------------------------------------------------
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
	struct {
		uint8_t			joinMode:2;			// 1 OTAA / 2 ABP
		uint8_t			networkType:2;		// 1 Public / 2 Private
		uint8_t			devEuiType:2;		// 1 static / 2 generated
		uint8_t			adrMode:2;			// 1 OFF / 2 ON
		uint8_t			retries;			// Number of Tx retry when no ack received

		uint16_t		align32b;			// 32b alignement
	} lorawan;
#endif


} itsdk_configuration_internal_t;


typedef struct {
	itsdk_configuration_internal_t	sdk;
#if ITSDK_WITH_CONFIGURATION_NVM == __ENABLE
	itsdk_configuration_app_t		app;
#endif
} itsdk_configuration_nvm_t;

// The configuration version
#define ITSDK_CONFIGURATION_SDK_VERSION		ITSDK_VERSION_BYTE

// The configuration management version
#define ITSDK_CONFIGURATION_MNG_VERSION		0x01

// ===========================================================================
// API
// ===========================================================================

extern itsdk_configuration_nvm_t itsdk_config;

typedef enum {
	CONFIG_NORMAL_LOAD = 0,
	CONFIG_FORCE_TO_FACTORY
} itsdk_config_load_mode_e;

typedef enum {
	CONFIG_SUCCESS = 0,
	CONFIG_LOADED,
	CONFIG_RESTORED_FROM_FACTORY,


	CONFIG_FAILED
} itsdk_config_ret_e;


// This function need to be override when the app mode is enabled
itsdk_config_ret_e itsdk_config_app_resetToFactory();

itsdk_config_ret_e itsdk_config_loadConfiguration(itsdk_config_load_mode_e mode);

#endif /* IT_SDK_EEPROM_SDK_CONFIG_H_ */
