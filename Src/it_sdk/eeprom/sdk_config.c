/* ==========================================================
 * sdk_config.c - sdk NVM configuration
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
#include <string.h>
#include <it_sdk/config.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>
#if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
  #include <it_sdk/eeprom/eeprom.h>
  #include <it_sdk/wrappers.h>
#endif

#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	#include <it_sdk/configSigfox.h>
	#include <it_sdk/sigfox/sigfox.h>
    #include <drivers/sigfox/se_nvm.h>
#endif

#if ITSDK_WITH_CONSOLE == __ENABLE
  #include <it_sdk/console/console.h>
  static itsdk_console_chain_t __console_configMng;
  static itsdk_console_return_e _itsdk_config_consolePriv(char * buffer, uint8_t sz);
  static itsdk_console_return_e _itsdk_config_consolePublic(char * buffer, uint8_t sz);
#endif

#if ITSDK_WITH_SECURESTORE == __ENABLE
  #include <it_sdk/eeprom/securestore.h>
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
		ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_SDKFACT_DEFAULT,0);

		itsdk_config.sdk.version = ITSDK_CONFIGURATION_SDK_VERSION;
		itsdk_config.sdk.size = sizeof(itsdk_configuration_internal_t);
		// ----------- Network choice ----------------------------------------------------
		#if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
		itsdk_config.sdk.activeNetwork = ITSDK_DEFAULT_NETWORK;
		#endif
		// ----------- LoRaWan settings --------------------------------------------------
		#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
		itsdk_config.sdk.lorawan.adrMode = ITSDK_LORAWAN_ADR;
		itsdk_config.sdk.lorawan.devEuiType = ITSDK_LORAWAN_DEVEUI_SRC;
		itsdk_config.sdk.lorawan.joinMode = ITSDK_LORAWAN_ACTIVATION;
		itsdk_config.sdk.lorawan.networkType = ITSDK_LORAWAN_NETWORKTYPE;
		itsdk_config.sdk.lorawan.retries = ITSDK_LORAWAN_CNF_RETRY;
		itsdk_config.sdk.activeRegion = ITSDK_DEFAULT_REGION;
		#endif
		// ----------- Sigfox settings --------------------------------------------------
		#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
		itsdk_config.sdk.sigfox.rssiCal = ITSDK_SIGFOX_RSSICAL;
		itsdk_config.sdk.sigfox.txPower = (ITSDK_SIGFOX_TXPOWER < ITSDK_SIGFOX_MAXPOWER || ITSDK_SIGFOX_TXPOWER == SIGFOX_DEFAULT_POWER)?ITSDK_SIGFOX_TXPOWER:ITSDK_SIGFOX_MAXPOWER;
		itsdk_config.sdk.sigfox.speed = ITSDK_SIGFOX_SPEED;
		uint8_t __rcz = 0;

		if ( itsdk_sigfox_getRczFromRegion(ITSDK_DEFAULT_REGION, &__rcz) != SIGFOX_INIT_SUCESS ) {
			// the Region is not supported
			itsdk_config.sdk.activeNetwork = __ACTIV_NETWORK_NONE;
			log_error("Sigfox Region not supported\r\n");
		}

		itsdk_config.sdk.sigfox.rcz = __rcz;
		itsdk_config.sdk.sigfox.sgfxKey = ITSDK_SIGFOX_KEY_TYPE;
		sfx_u32 config_words_2[3] = RC2_SM_CONFIG;
		bcopy(config_words_2,itsdk_config.sdk.sigfox.macroch_config_words_rc2,3*sizeof(sfx_u32));
		sfx_u32 config_words_3[3] = RC3C_CONFIG;
		bcopy(config_words_3,itsdk_config.sdk.sigfox.macroch_config_words_rc3,3*sizeof(sfx_u32));
		sfx_u32 config_words_4[3] = RC4_SM_CONFIG;
		bcopy(config_words_4,itsdk_config.sdk.sigfox.macroch_config_words_rc4,3*sizeof(sfx_u32));
		// reset the Sigfox NVM if not yet already created
		__itsdk_sigfox_resetNvmToFactory(false);
		#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
		  uint8_t pac[8] = ITSDK_SIGFOX_PAC;
		  bcopy(pac,itsdk_config.sdk.sigfox.initialPac,8);
		  itsdk_config.sdk.sigfox.deviceId = ITSDK_SIGFOX_ID;
		#endif

		#endif
		return CONFIG_RESTORED_FROM_FACTORY;
	}

	#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
		/**
		 * This function need to be overide - reset the app to factory default
		 */
		__weak itsdk_config_ret_e itsdk_config_app_resetToFactory() {
			itsdk_config.app.version = ITSDK_CONFIGURATION_APP_VERSION;
		    //ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_SDKFACT_DEFAULT,1);
			ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_OVERRIDE_MISS,0);
			return CONFIG_FAILED;
		}
		/**
		 * This function need to be overide - upgrade the app configuration
		 */
		__weak itsdk_config_ret_e itsdk_config_app_upgradeConfiguration() {
			itsdk_config.app.version = ITSDK_CONFIGURATION_APP_VERSION;
		    //ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_APPCNF_UPGRADED,1);
			ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_OVERRIDE_MISS,0);
			return CONFIG_FAILED;
		}
	#endif

	/**
	 * Reset to factory default update eeprom & shadow
	 */
	itsdk_config_ret_e itsdk_config_resetToFactory() {
		 itsdk_config_sdk_resetToFactory();
		 #if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
		 itsdk_config_app_resetToFactory();
		 #endif
		 #if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
		 eeprom_write_config(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
		 #endif
	     bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));
		 ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,3);
	     return CONFIG_SUCCESS;
	}

	/**
	 * Flush the current config into eeprom & shadow
	 * The shadow config is lost
	 */
	itsdk_config_ret_e itsdk_config_flushConfig() {
		 #if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
	     eeprom_write_config(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
		 #endif
	     bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));
	     ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_COMMIT_NEW_CONF,0);
	     return CONFIG_SUCCESS;
	}

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
  uint8_t v;
  uint8_t requestFactoryReset = 0;
  uint8_t configUpdated = 0;

  if ( mode != CONFIG_FORCE_TO_FACTORY ) {
     if ( ! eeprom_read_config(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION,&v, true) ) {
		 // failed to read configuration
		 if ( v == 0 ) {
  		    ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_CONFIG_BADMAGIC,0);
			requestFactoryReset=1;
		 } else if ( v!= ITSDK_CONFIGURATION_MNG_VERSION ) {
  		    // magic is invalid or MNG version has changed... impossible to migrate
  		    ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_CONFIG_BADMNGV,0);
			requestFactoryReset=1;
		 }
	  }
   	  // we have a config loaded but it can be a wrong one
	  // Process SDK
      //   management remains the same but the version & size may have changed.
	  //   migration is possible
	  if ( requestFactoryReset == 0 && (itsdk_config.sdk.version != ITSDK_CONFIGURATION_SDK_VERSION || itsdk_config.sdk.size != sizeof(itsdk_configuration_internal_t) ) ) {
	     // SDK version has changed or compilation option has changed
	     if ( itsdk_config.sdk.size != sizeof(itsdk_configuration_internal_t) ) {
	 	    // and the size has changed so we need to force a reset
		    requestFactoryReset=1;
		    ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_MIGRATE_FAILED,0);
		 } else {
		    // migration is possible ...
			//  if ( itsdk_config.sdk.version < 0x17 ) {
			//  }
		    itsdk_config.sdk.version = ITSDK_CONFIGURATION_SDK_VERSION;

		    configUpdated = 1;
		    ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_SDKCNF_UPGRADED,ITSDK_CONFIGURATION_SDK_VERSION);
		 }
	  }
	  #if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
	  if ( requestFactoryReset == 0 && itsdk_config.app.version != ITSDK_CONFIGURATION_APP_VERSION ) {
		 // version has changed an upgrade can be possible, lets application level to decide.
		 if ( itsdk_config_app_upgradeConfiguration() == CONFIG_UPGRADED ) {
			 // at least the version has been upgraded
			 configUpdated = 1;
			 ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_APPCNF_UPGRADED,ITSDK_CONFIGURATION_APP_VERSION);
		 } else {
			 requestFactoryReset = 1;
  		     ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_MIGRATE_FAILED,1);
		 }
      }
	  #endif

   } else requestFactoryReset = 1;


   if ( requestFactoryReset == 1 ) {
	  itsdk_config_sdk_resetToFactory();
	  itsdk_config_app_resetToFactory();
	  configUpdated = 1;
	  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,3);
   }

   if (configUpdated == 1) {
	  // need to store the configuration
      eeprom_write_config(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
   }
   bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));

 #endif
 #if ITSDK_CONFIGURATION_MODE == __CONFIG_MEMORY
      requestFactoryReset = 1;
	  itsdk_config_sdk_resetToFactory();
    #if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
	  itsdk_config_app_resetToFactory();
    #endif
      bcopy(&itsdk_config,&itsdk_config_shadow,sizeof(itsdk_configuration_nvm_t));
 #endif

   return ( requestFactoryReset == 1 )?CONFIG_RESTORED_FROM_FACTORY:CONFIG_LOADED;

}

// ====================================================================================================
// CONFIG SHADOW
// ====================================================================================================

#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC

__weak itsdk_config_ret_e itsdk_config_app_commitConfiguration() {
	return CONFIG_SUCCESS;
}


/**
 * Commit a shadow configuration into the standard config.
 * Save it
 */
itsdk_config_ret_e itsdk_config_commitConfiguration(itsdk_config_commit_mode_e mode) {

 itsdk_config_ret_e r = itsdk_config_app_commitConfiguration();
 if ( r != CONFIG_SUCCESS ) return r;

 bcopy(&itsdk_config_shadow,&itsdk_config,sizeof(itsdk_configuration_nvm_t));
 ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_COMMIT_NEW_CONF,0);

#if ITSDK_CONFIGURATION_MODE == __CONFIG_EEPROM
 if ( mode == CONFIG_COMMIT_SAVE || mode == CONFIG_COMMIT_SAVE_REBOOT ) {
     eeprom_write_config(&itsdk_config, sizeof(itsdk_configuration_nvm_t), ITSDK_CONFIGURATION_MNG_VERSION);
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
#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
static bool __checkAndConvert(char * str,uint8_t start,uint8_t stop,uint8_t sz,uint8_t * buf) {
	if ( (stop - start) < 2*sz ) return false;
	int k = 0;
	for ( int i = start ; i < stop ; i+=2 ) {
		if ( itdt_isHexChar(str[i],false) && itdt_isHexChar(str[i+1],false) ) {
			buf[k] = itdt_convertHexChar2Int(&str[i]);
			k++;
		} else return false;
	}
	return true;
}
#endif

#if ITSDK_WITH_CONSOLE == __ENABLE

	#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
		/**
		 * This function need to be overrided
		 */
		__weak void itsdk_config_app_printConfig(itsdk_configuration_nvm_t * c) {
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
					#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
					itsdk_config_app_printConfig(NULL);
					#endif
				  #else
					itsdk_configuration_nvm_t * _c = &itsdk_config;
					if (buffer[0]=='C') _c = &itsdk_config_shadow;
					_itsdk_console_printf("sdk.version : %02X\r\n",_c->sdk.version);
					#if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
					_itsdk_console_printf("sdk.activeNetwork : %d\r\n",_c->sdk.activeNetwork);
					_itsdk_console_printf("sdk.activeRegion : %04X\r\n",_c->sdk.activeRegion);
					#endif
					#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
					_itsdk_console_printf("sdk.lora.adrmode : %d\r\n",_c->sdk.lorawan.adrMode);
					_itsdk_console_printf("sdk.lora.devEuiType : %d\r\n",_c->sdk.lorawan.devEuiType);
					_itsdk_console_printf("sdk.lora.joinMode : %d\r\n",_c->sdk.lorawan.joinMode);
					_itsdk_console_printf("sdk.lora.networkType : %d\r\n",_c->sdk.lorawan.networkType);
					_itsdk_console_printf("sdk.lora.retries : %d\r\n",_c->sdk.lorawan.retries);
				    #endif

					#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
					_itsdk_console_printf("sdk.sigfox.rcz : %d\r\n",_c->sdk.sigfox.rcz);
					if ( _c->sdk.sigfox.txPower == SIGFOX_DEFAULT_POWER ) {
						  _itsdk_console_printf("sdk.sigfox.txPower : rcz default\r\n");
					} else {
					  _itsdk_console_printf("sdk.sigfox.txPower : %ddB\r\n",_c->sdk.sigfox.txPower);
					}
					if ( _c->sdk.sigfox.speed == SIGFOX_DEFAULT_SPEED ) {
						_itsdk_console_printf("sdk.sigfox.speed : rcz default\r\n");
					} else {
						_itsdk_console_printf("sdk.sigfox.speed : %dbps\r\n",_c->sdk.sigfox.speed);
					}
					_itsdk_console_printf("sdk.sigfox.sgfxKey : %d\r\n",_c->sdk.sigfox.sgfxKey);
					 #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
					 _itsdk_console_printf("sdk.sigfox.initialPac : [%02X%02X%02X%02X%02X%02X%02X%02X]\r\n",
							_c->sdk.sigfox.initialPac[0],
							_c->sdk.sigfox.initialPac[1],
							_c->sdk.sigfox.initialPac[2],
							_c->sdk.sigfox.initialPac[3],
							_c->sdk.sigfox.initialPac[4],
							_c->sdk.sigfox.initialPac[5],
							_c->sdk.sigfox.initialPac[6],
							_c->sdk.sigfox.initialPac[7]
						);
					 _itsdk_console_printf("sdk.sigfox.deviceId : %08X \r\n",_c->sdk.sigfox.deviceId);
					 _itsdk_console_printf("sdk.sigfox.rssiCal : %d\r\n",_c->sdk.sigfox.rssiCal);
					 #endif
					#endif

					#if ITSDK_WITH_CONFIGURATION_APP == __ENABLE
					   itsdk_config_app_printConfig(_c);
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
			  _itsdk_console_printf("F          : restore factory default config\r\n");
			  _itsdk_console_printf("m          : see eeprom configuration\r\n");
			#endif
			#if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
			  _itsdk_console_printf("SC:N:x     : sdk.activeNetwork 1:SFX 2:LoRa\r\n");
			  _itsdk_console_printf("SC:R:xxxx  : sdk.activeRegion __PLWAN_REGION_xx\r\n");
			#endif
			#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
			  _itsdk_console_printf("SC:0:x     : lora.adrmode 1:OFF/2:ON\r\n");
			  _itsdk_console_printf("SC:1:x     : lora.devEuiType 1:STATIC/2:GENERATED\r\n");
			  _itsdk_console_printf("SC:2:x     : lora.joinMode 1:OTAA/2:ABP\r\n");
			  _itsdk_console_printf("SC:3:x     : lora.networkType 1:PUBLIC/2:PRIVATE\r\n");
			  _itsdk_console_printf("SC:4:nn    : lora.retries 00..99\r\n");
			#endif
			#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
			  _itsdk_console_printf("SC:8:xx    : sigfox.txPower 00-22dB (decimal) 99 (default)\r\n");
			  _itsdk_console_printf("SC:9:xx    : sigfox.speed 0(default)/100/600bps (decimal)\r\n");
			  _itsdk_console_printf("SC:A:xx    : sigfox.rcz [01,02,3c,04,05]\r\n");
			  _itsdk_console_printf("SC:B:x     : sigfox.sgfxKey 0:PRIVATE 1:PUBLIC\r\n");
			  _itsdk_console_printf("SC:C:8hex  : sigfox.initialPac 8B hex string\r\n");
			  _itsdk_console_printf("SC:D:4hex  : sigfox.deviceId 4B hex string\r\n");
			  _itsdk_console_printf("sc:d       : get sigfox.deviceId\r\n");
			#endif
		  return ITSDK_CONSOLE_SUCCES;
		  break;
  	    #if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
		case 'S':
			// Commit the new configuration
			itsdk_config_commitConfiguration(CONFIG_COMMIT_SAVE);
			_itsdk_console_printf("OK\r\n");
			return ITSDK_CONSOLE_SUCCES;
			break;
		case'F':
			  // Reset to factory default
			  ITSDK_ERROR_REPORT(ITSDK_ERROR_CONFIG_FACTORY_DEFAULT,3);
			  itsdk_config_resetToFactory();
			  _itsdk_console_printf("OK\r\n");
			 return ITSDK_CONSOLE_SUCCES;
		case 'm': {
			  // print the eeprom memory mapping configuration
			  uint32_t offset = 0;
			  uint32_t size = 0;
			  uint32_t totSize = 0;
			  #if ITSDK_WITH_SECURESTORE == __ENABLE
			  	itsdk_secstore_getStoreSize(&size);
			  	_itsdk_console_printf("SecureStore: 0x%08X->0x%08X (%dB)\r\n",offset,offset+size,size);
			  	offset += size;
			  	totSize += size;
			  #endif
			  #if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
			  	itsdk_error_getSize(&size);
			  	_itsdk_console_printf("ErrorLog: 0x%08X->0x%08X (%dB)\r\n",offset,offset+size,size);
			  	offset += size;
			  	totSize += size;
			  #endif
			  #if (ITSDK_WITH_SIGFOX_LIB == __ENABLE)
			  	itsdk_sigfox_getNvmSize(&size);
			  	_itsdk_console_printf("SigfoxConfig: 0x%08X->0x%08X (%dB)\r\n",offset,offset+size,size);
			  	offset += size;
			  	totSize += size;
			  #endif
			  eeprom_getConfigSize(&size);
  		  	  totSize += size;
			  _itsdk_console_printf("ApplicationConfig: 0x%08X->0x%08X (%dB)\r\n",offset,offset+size,size);
			  _itsdk_console_printf("UsedMemory: %dB on %dB\r\n",totSize,ITSDK_EPROM_SIZE);
			  _itsdk_console_printf("OK\r\n");
			 return ITSDK_CONSOLE_SUCCES;
			}
		#endif
		default:
			break;
	  }
	} else if ( sz == 4 ) {
		if ( buffer[0] == 's' && buffer[1] == 'c' && buffer[2] == ':' ) {
			switch(buffer[3]) {
				#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
				case 'd': {
						itsdk_sigfox_device_is_t deviceId;
						itsdk_sigfox_getDeviceId(&deviceId);
						_itsdk_console_printf("%08X\r\n",deviceId);
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					break;
				#endif
				default:
					_itsdk_console_printf("KO\r\n");
					return ITSDK_CONSOLE_FAILED;
			}
		}
	} else if ( sz >= 6 ) {
		if ( buffer[0] == 'S' && buffer[1] == 'C' && buffer[2] == ':' && buffer[4] == ':' ) {
			switch(buffer[3]) {
			#if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
			case 'N': {
				// sdk.activeNetwork
				switch ( buffer[5] ) {
					case '0':
					   itsdk_config_shadow.sdk.activeNetwork = __ACTIV_NETWORK_NONE;
					   break;
					case '1':
					   itsdk_config_shadow.sdk.activeNetwork = __ACTIV_NETWORK_SIGFOX;
					   break;
					case '2':
					   itsdk_config_shadow.sdk.activeNetwork = __ACTIV_NETWORK_LORAWAN;
					   break;
					default:
						_itsdk_console_printf("KO\r\n");
						return ITSDK_CONSOLE_FAILED;
				}
				_itsdk_console_printf("OK\r\n");
				return ITSDK_CONSOLE_SUCCES;
			}
			case 'R': {
				if ( itdt_isHexString( &buffer[5],4,false) ) {
					uint16_t v = itdt_convertHexChar4Int(&buffer[5]);
					if ( itdt_count_bits_1(v) <= 1 ) {
						itsdk_config_shadow.sdk.activeRegion = v;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
			}
			#endif

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
					int v = itdt_convertDecCharNInt(&buffer[5],2);
					if ( v != ITSDK_INVALID_VALUE_32B && v <= 5 ) {
						itsdk_config_shadow.sdk.lorawan.retries = v;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			#endif
			#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
			case '8':
				// sigfox.txPower
				if ( sz >= 7 ) {
					int v = itdt_convertDecCharNInt(&buffer[5],2);
					if ( v != ITSDK_INVALID_VALUE_32B && v >= -1 && v < 24 ) {
						itsdk_config_shadow.sdk.sigfox.txPower = v;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					if ( v == 99 ) {
						itsdk_config_shadow.sdk.sigfox.txPower = SIGFOX_DEFAULT_POWER;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case '9':
				// sigfox.speed
				if ( sz >= 8 ) {
					if ( buffer[6] == '0' && buffer[7] == '0' ) {
						if ( buffer[5] == '1') {
							itsdk_config_shadow.sdk.sigfox.speed = SIGFOX_SPEED_100;
							_itsdk_console_printf("OK\r\n");
							return ITSDK_CONSOLE_SUCCES;
						} else if ( buffer[5] == '6' ) {
							itsdk_config_shadow.sdk.sigfox.speed = SIGFOX_SPEED_600;
							_itsdk_console_printf("OK\r\n");
							return ITSDK_CONSOLE_SUCCES;
						} else if ( buffer[5] == '0' ) {
							itsdk_config_shadow.sdk.sigfox.speed = SIGFOX_DEFAULT_SPEED;
							_itsdk_console_printf("OK\r\n");
							return ITSDK_CONSOLE_SUCCES;
						}
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case 'A':
				if ( sz >= 7 ) {
					if ( buffer[5] == '0' && buffer[6] == '1' ) {
						itsdk_config_shadow.sdk.sigfox.rcz = SIGFOX_RCZ1;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					if ( buffer[5] == '0' && buffer[6] == '2' ) {
						itsdk_config_shadow.sdk.sigfox.rcz = SIGFOX_RCZ2;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					if ( buffer[5] == '3' && buffer[6] == 'c' ) {
						itsdk_config_shadow.sdk.sigfox.rcz = SIGFOX_RCZ3C;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					if ( buffer[5] == '0' && buffer[6] == '4' ) {
						itsdk_config_shadow.sdk.sigfox.rcz = SIGFOX_RCZ4;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					if ( buffer[5] == '0' && buffer[6] == '5' ) {
						itsdk_config_shadow.sdk.sigfox.rcz = SIGFOX_RCZ5;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case 'B':
				if ( sz >= 6) {
					if ( buffer[5] == '0' ) {
						itsdk_config_shadow.sdk.sigfox.sgfxKey = SIGFOX_KEY_PRIVATE;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					if ( buffer[5] == '1' ) {
						itsdk_config_shadow.sdk.sigfox.sgfxKey = SIGFOX_KEY_PUBLIC;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
				}
				_itsdk_console_printf("KO\r\n");
				return ITSDK_CONSOLE_FAILED;
				break;
			case 'C':
				{
					uint8_t b[8];
					if ( __checkAndConvert(buffer,5,sz,8,b) ) {
						bcopy(b,itsdk_config_shadow.sdk.sigfox.initialPac,8);
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					_itsdk_console_printf("KO\r\n");
					return ITSDK_CONSOLE_FAILED;
				}
				break;
			case 'D':
				{
					uint8_t b[4];
					if ( __checkAndConvert(buffer,5,sz,4,b) ) {
						itsdk_config_shadow.sdk.sigfox.deviceId = (b[0] << 24) & 0xFF000000;
						itsdk_config_shadow.sdk.sigfox.deviceId |= (b[1] << 16) & 0xFF0000;
						itsdk_config_shadow.sdk.sigfox.deviceId |= (b[2] << 8) & 0xFF00;
						itsdk_config_shadow.sdk.sigfox.deviceId |= (b[3]) & 0xFF;
						_itsdk_console_printf("OK\r\n");
						return ITSDK_CONSOLE_SUCCES;
					}
					_itsdk_console_printf("KO\r\n");
					return ITSDK_CONSOLE_FAILED;
				}
				break;
			#endif	// ITSDK_WITH_SIGFOX_LIB
			default:
				break;
			}
		}
	}
#endif
  return ITSDK_CONSOLE_NOTFOUND;
}

#endif // ITSDK_WITH_CONSOLE


