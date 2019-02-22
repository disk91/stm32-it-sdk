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
#include <string.h>
#include <it_sdk/config.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/logger/error.h>
#if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
  #include <it_sdk/eeprom/eeprom.h>
  #include <it_sdk/wrappers.h>
#endif

#if ITSDK_WITH_CONSOLE == __ENABLE
  #include <it_sdk/console/console.h>
  static itsdk_console_chain_t __console_configMng;
  static itsdk_console_return_e _itsdk_config_consolePriv(char * buffer, uint8_t sz);
  static itsdk_console_return_e _itsdk_config_consolePublic(char * buffer, uint8_t sz);
#endif


/**
 * In Memory configuration image
 */
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
  itsdk_configuration_nvm_t itsdk_config;
  itsdk_configuration_nvm_t itsdk_config_shadow;
#endif


/**
 * The SDK config initialization function
 */
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
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

	#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
		/**
		 * This function need to be overrided
		 */
		__weak itsdk_config_ret_e itsdk_config_app_resetToFactory() {
			ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_OVERRIDE_MISS,0);
			return CONFIG_FAILED;
		}
	#endif

#endif

/**
 * Load the configuration from the NVM, eventually from factory default when the NVM is disabled
 * The Configuration will be restored from default when the version have changed.
 * The factory init functions will be called.
 * When a Application defined configuration exists the initialization function needs to be override.
 */
itsdk_config_ret_e itsdk_config_loadConfiguration(itsdk_config_load_mode_e mode) {

#if ITSDK_WITH_CONSOLE == __ENABLE
	__console_configMng.console_private = _itsdk_config_consolePriv;
	__console_configMng.console_public = _itsdk_config_consolePublic;
	__console_configMng.next = NULL;
	itsdk_console_registerCommand(&__console_configMng);
#endif

#if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
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
	  #if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
	  if ( force || itsdk_config.app.version != ITSDK_CONFIGURATION_APP_VERSION ) {
		  itsdk_config_app_resetToFactory();
		  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,2);
		  hasChanged=1;
	  }
	  #endif
  } else {
	  hasChanged=1;
#endif
	#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
	  // The data are not valid - reset to factory default
	  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,3);
	  itsdk_config_sdk_resetToFactory();
	 #if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
	  itsdk_config_app_resetToFactory();
	 #endif
    #endif
#if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
  }
  if ( hasChanged != 0 ) {
     eeprom_write(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
  } else {
	  bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));
	  return CONFIG_LOADED;
  }
#endif
  bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));
  return CONFIG_RESTORED_FROM_FACTORY;
}

// ====================================================================================================
// CONFIG SHADOW
// ====================================================================================================

#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC

/**
 * Commit a shadow configuration into the standard config.
 * Save it
 */
itsdk_config_ret_e itsdk_config_commitConfiguration(itsdk_config_commit_mode_e mode) {

 bcopy(&itsdk_config_shadow,&itsdk_config,sizeof(itsdk_configuration_nvm_t));
#if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
 if ( mode == CONFIG_COMMIT_SAVE || mode == CONFIG_COMMIT_SAVE_REBOOT ) {
     eeprom_write(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
 }
 if ( mode == CONFIG_COMMIT_SAVE_REBOOT ) {
	 itsdk_delayMs(200);
	 itsdk_reset();
 }
#endif
 return CONFIG_SUCCESS;
}

#endif

// ====================================================================================================
// CONSOLE PART
// ====================================================================================================


#if ITSDK_WITH_CONSOLE == __ENABLE

	#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
		/**
		 * This function need to be overrided
		 */
		__weak void itsdk_config_app_printConfig() {
			ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_OVERRIDE_MISS,0);
			return;
		}

	#endif // ITSDK_WITH_CONFIGURATION_APP

	static itsdk_console_return_e _itsdk_config_consolePublic(char * buffer, uint8_t sz) {
	   if ( sz == 1 ) {
		  switch(buffer[0]){
			case '?':
				// help
				_itsdk_console_printf("--- ConfigMng\r\n");
				_itsdk_console_printf("c          : print config\r\n");
				#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
				_itsdk_console_printf("C          : print shadow config\r\n");
				#endif
			  return ITSDK_CONSOLE_SUCCES;
			  break;
			case 'c':
			case 'C':
				{
				  #if ITSDK_CONFIGURATION_MODE == __CONFIG_STATIC
					_itsdk_console_printf("sdk.version : %02X\r\n",ITSDK_CONFIGURATION_SDK_VERSION);
					#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
					_itsdk_console_printf("sdk.lora.adrmode : %d\r\n",ITSDK_LORAWAN_ADR);
					_itsdk_console_printf("sdk.lora.devEuiType : %d\r\n",ITSDK_LORAWAN_DEVEUI_SRC);
					_itsdk_console_printf("sdk.lora.joinMode : %d\r\n",ITSDK_LORAWAN_ACTIVATION);
					_itsdk_console_printf("sdk.lora.networkType : %d\r\n",ITSDK_LORAWAN_NETWORKTYPE);
					_itsdk_console_printf("sdk.lora.retries : %d\r\n",ITSDK_LORAWAN_CNF_RETRY);
					#endif
				  #else
					itsdk_configuration_nvm_t * _c = &itsdk_config;
					if (buffer[0]=='C') _c = &itsdk_config_shadow;
					_itsdk_console_printf("sdk.version : %02X\r\n",_c->sdk.version);
					#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
					_itsdk_console_printf("sdk.lora.adrmode : %d\r\n",_c->sdk.lorawan.adrMode);
					_itsdk_console_printf("sdk.lora.devEuiType : %d\r\n",_c->sdk.lorawan.devEuiType);
					_itsdk_console_printf("sdk.lora.joinMode : %d\r\n",_c->sdk.lorawan.joinMode);
					_itsdk_console_printf("sdk.lora.networkType : %d\r\n",_c->sdk.lorawan.networkType);
					_itsdk_console_printf("sdk.lora.retries : %d\r\n",_c->sdk.lorawan.retries);
				    #endif
				  #endif
				  _itsdk_console_printf("OK\r\n");
				}
				return ITSDK_CONSOLE_SUCCES;
				break;
			default:
				break;
		  }
	  } //Sz == 1
	  return ITSDK_CONSOLE_NOTFOUND;
	}

static itsdk_console_return_e _itsdk_config_consolePriv(char * buffer, uint8_t sz) {
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
	if ( sz == 1 ) {
	  switch(buffer[0]){
		case '?':
			// help
 			#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
			  _itsdk_console_printf("S          : commit configuration\r\n");
			  _itsdk_console_printf("F          : restore factory defaults\r\n");
			#endif
			#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
			  _itsdk_console_printf("SC:0:x     : lora.adrmode 1:OFF/2:ON\r\n");
			  _itsdk_console_printf("SC:1:x     : lora.devEuiType 1:STATIC/2:GENERATED\r\n");
			  _itsdk_console_printf("SC:2:x     : lora.joinMode 1:OTAA/2:ABP\r\n");
			  _itsdk_console_printf("SC:3:x     : lora.networkType 1:PUBLIC/2:PRIVATE\r\n");
			  _itsdk_console_printf("SC:4:nn    : lora.retries 00..99\r\n");
			#endif
		  return ITSDK_CONSOLE_SUCCES;
		  break;
  	    #if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
		case 'S':
			// Commit the new configuration
			ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_COMMIT_NEW_CONF,0);
			itsdk_config_commitConfiguration(CONFIG_COMMIT_SAVE);
			_itsdk_console_printf("OK\r\n");
			return ITSDK_CONSOLE_SUCCES;
			break;
		case'F':
			  // Reset to factory default
			  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,3);
			  itsdk_config_sdk_resetToFactory();
			 #if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
			  itsdk_config_app_resetToFactory();
			 #endif
			 #if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
		     eeprom_write(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
			 #endif
		     bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));
 			 _itsdk_console_printf("OK\r\n");
			 return ITSDK_CONSOLE_SUCCES;
		#endif
		default:
			break;
	  }
	} else if ( sz >= 6 ) {
		if ( buffer[0] == 'S' && buffer[1] == 'C' && buffer[2] == ':' && buffer[4] == ':' ) {
			switch(buffer[3]) {
 	 	 	#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
			case '0':
				// lora.adrmode
				if ( buffer[5] == '1' ) {
					itsdk_config_shadow.sdk.lorawan.adrMode = __LORAWAN_ADR_OFF;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
				}
			    if ( buffer[5] == '2' ) {
			    	itsdk_config_shadow.sdk.lorawan.adrMode = __LORAWAN_ADR_ON;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
			    }
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case '1':
				// lora.devEuiType
				if ( buffer[5] == '1' ) {
					itsdk_config_shadow.sdk.lorawan.devEuiType = __LORAWAN_DEVEUI_STATIC;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
				}
			    if ( buffer[5] == '2' ) {
			    	itsdk_config_shadow.sdk.lorawan.devEuiType = __LORAWAN_DEVEUI_GENERATED;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
			    }
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case '2':
				// lora.joinMode
				if ( buffer[5] == '1' ) {
					itsdk_config_shadow.sdk.lorawan.joinMode = __LORAWAN_OTAA;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
				}
			    if ( buffer[5] == '2' ) {
			    	itsdk_config_shadow.sdk.lorawan.joinMode = __LORAWAN_ABP;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
			    }
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case '3':
				// lora.networkType
				if ( buffer[5] == '1' ) {
					itsdk_config_shadow.sdk.lorawan.networkType = __LORAWAN_NWK_PUBLIC;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
				}
			    if ( buffer[5] == '2' ) {
			    	itsdk_config_shadow.sdk.lorawan.networkType = __LORAWAN_NWK_PRIVATE;
					_itsdk_console_printf("OK\r\n");
					return ITSDK_CONSOLE_SUCCES;
			    }
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case '4':
				// lora.retries
				if ( sz >= 7 ) {
					int v = 0;
					char c = buffer[5];
					if ( c >= '0' && c <= '9' ) {
						v = 10*(c - '0');
						c = buffer[6];
						if ( c >= '0' && c <= '9' ) {
							v = v + (c - '0');
						} else 	v = -1;
					} else v = -1;
					if ( v >= 0 ) {
						itsdk_config_shadow.sdk.lorawan.retries = v;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			#endif
			default:
				break;
			}
		}
	}
#endif
  return ITSDK_CONSOLE_NOTFOUND;
}

#endif // ITSDK_WITH_CONSOLE


