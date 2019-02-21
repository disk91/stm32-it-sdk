/* ==========================================================
 * sdk_config.c - sdk NVM configuration
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

#include <it_sdk/config.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/logger/error.h>

/**
 * In Memory configuration image
 */
itsdk_configuration_nvm_t itsdk_config;


/**
 * The SDK config initialization function
 */
static itsdk_config_ret_e itsdk_config_sdk_resetToFactory() {
	itsdk_config.sdk.version = ITSDK_CONFIGURATION_SDK_VERSION;
	itsdk_config.sdk.size = sizeof(itsdk_configuration_internal_t);
	// ----------- LoRaWan settings --------------------------------------------------
	#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
	itsdk_config.sdk.lorawan.adrMode = ITSDK_LORAWAN_ADR;
	itsdk_config.sdk.lorawan.devEuiType = ITSDK_LORAWAN_DEVEUI_SRC;
	itsdk_config.sdk.lorawan.joinMode = ITSDK_LORAWAN_ACTIVATION;
	itsdk_config.sdk.lorawan.networkType = ITSDK_LORAWAN_NETWORKTYPE;
	itsdk_config.sdk.lorawan.retries = ITSDK_LORAWAN_CNF_RETRY;
	#endif
	return CONFIG_RESTORED_FROM_FACTORY;
}

/**
 * This function need to be overrided
 */
__weak itsdk_config_ret_e itsdk_config_app_resetToFactory() {
	ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_OVERRIDE_MISS,0);
	return CONFIG_FAILED;
}


/**
 * Load the configuration from the NVM, eventually from factory default when the NVM is disabled
 * The Configuration will be restored from default when the version have changed.
 * The factory init functions will be called.
 * When a Application defined configuration exists the initialization function needs to be override.
 */
itsdk_config_ret_e itsdk_config_loadConfiguration(itsdk_config_load_mode_e mode) {

#if ITSDK_WITH_NO_CONFIG_AT_ALL == __DISABLE
  uint8_t v= ITSDK_CONFIGURATION_MNG_VERSION;
  uint8_t hasChanged = 0;
  if ( mode != CONFIG_FORCE_TO_FACTORY && eeprom_read(&itsdk_config, sizeof(itsdk_configuration_nvm_t), 1,&v) ) {
	  uint8_t force = 0;
	  // The data are correctly loaded
	  if ( itsdk_config.sdk.version != ITSDK_CONFIGURATION_SDK_VERSION ) {
		  // SDK version has changed
		  if ( itsdk_config.sdk.size != sizeof(itsdk_configuration_internal_t) ) {
			  force = 1;
		  }
		  itsdk_config_sdk_resetToFactory();
		  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,1);
		  hasChanged=1;
	  }
	  if ( force || itsdk_config.app.version != ITSDK_CONFIGURATION_APP_VERSION ) {
		  itsdk_config_app_resetToFactory();
		  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,2);
		  hasChanged=1;
	  }
  } else {
	  hasChanged=1;
#endif
  // The data are not valid - reset to factory default
  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,3);
  itsdk_config_sdk_resetToFactory();
  itsdk_config_app_resetToFactory();
#if ITSDK_WITH_NO_CONFIG_AT_ALL == __DISABLE
  }
  if ( hasChanged != 0 ) {
     eeprom_write(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
  } else {
	  return CONFIG_LOADED;
  }
#endif
  return CONFIG_RESTORED_FROM_FACTORY;
}



