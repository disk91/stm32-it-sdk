/* ==========================================================
 * sdk_config.h - structure used for the SDK configuration
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 21 f�vr. 2019
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

#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
	#include <it_sdk/configNvm.h>
#endif

/**
 * SDK specific structure
 * size aligned on 32b
 */
typedef struct {
	uint8_t				version;
	uint16_t			size;
	uint8_t				align32b_0;					   // 32b alignment

#if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
	uint16_t			activeNetwork;				   // active network Sigfox vs LoRaWan 32b for alignement
	uint16_t		    activeRegion;				   // current region to activate on startup
#endif
	// ----------- Sigfox settings --------------------------------------------------
#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	struct {
		int16_t			rssiCal;						// Rssi calibration ( offset )
		int8_t      	txPower;						// Default Tx power - use SIGFOX_DEFAULT_POWER for default regional setting
		uint16_t		speed;							// Default Speed - use SIGFOX_DEFAULT_SPEED for default regional setting
		uint8_t			sgfxKey;						// Sigfox key type Public / Private
		uint8_t			rcz;							// Default RCZ
		uint32_t		macroch_config_words_rc2[3];    // Macro channel config
		uint32_t		macroch_config_words_rc3[3];	// Macro channel config
		uint32_t		macroch_config_words_rc4[3];	// Macro channel config
	#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
		uint32_t		deviceId;						// DeviceId
		uint8_t			initialPac[8];					// Initial PAC
	#endif
		uint8_t         align32b[1];		// 32b alignement
	} sigfox;
#endif

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
	uint32_t			reserved[2];		// reserve space for later use

} itsdk_configuration_internal_t;


typedef struct {
	itsdk_configuration_internal_t	sdk;
#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
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
extern itsdk_configuration_nvm_t itsdk_config_shadow;

typedef enum {
	CONFIG_NORMAL_LOAD = 0,
	CONFIG_FORCE_TO_FACTORY
} itsdk_config_load_mode_e;

typedef enum {
	CONFIG_COMMIT_ONLY = 0,			// copy shadow to config, not more
	CONFIG_COMMIT_SAVE,				// copy shadow to config, save the config
	CONFIG_COMMIT_SAVE_REBOOT		// copy, save, reboot device
} itsdk_config_commit_mode_e;

typedef enum {
	CONFIG_SUCCESS = 0,
	CONFIG_LOADED,
	CONFIG_RESTORED_FROM_FACTORY,
	CONFIG_UPGRADED,

	CONFIG_FAILED
} itsdk_config_ret_e;


// These functions need to be override when the app mode is enabled
itsdk_config_ret_e itsdk_config_app_resetToFactory();
itsdk_config_ret_e itsdk_config_app_upgradeConfiguration();
#if ITSDK_WITH_CONSOLE == __ENABLE
void itsdk_config_app_printConfig(itsdk_configuration_nvm_t * c);
#endif
itsdk_config_ret_e itsdk_config_app_commitConfiguration();

// Init Config
itsdk_config_ret_e itsdk_config_loadConfiguration(itsdk_config_load_mode_e mode);

// Commit the shadow configuration into configuration
itsdk_config_ret_e itsdk_config_commitConfiguration(itsdk_config_commit_mode_e mode);

// Store the current config into the eeprom & update shadow
itsdk_config_ret_e itsdk_config_flushConfig();

// Factory default
itsdk_config_ret_e itsdk_config_resetToFactory();

// ===========================================================================
// MISC INTERNAL
// ===========================================================================


#endif /* IT_SDK_EEPROM_SDK_CONFIG_H_ */
