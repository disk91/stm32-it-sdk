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
#include <it_sdk/wrappers.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/eeprom/sdk_state.h>
#if ITSDK_WITH_CONSOLE == __ENABLE
#include <it_sdk/console/console.h>
#endif
#include <it_sdk/lorawan/lorawan.h>

itsdk_state_t itsdk_state;

void itsdk_state_init() {
	itsdk_state.lastWakeUpTimeUs = 0;
	itsdk_state.lastResetCause = itsdk_getResetCause();

#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
   #if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
	itsdk_state.activeNetwork = (uint8_t)itsdk_config.sdk.activeNetwork;
   #endif
#else
   #if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
	itsdk_state.activeNetwork = ITSDK_DEFAULT_NETWORK;
	#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	#endif
   #endif
#endif

#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	itsdk_state.sigfox.initialized = false;
  #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	itsdk_state.sigfox.rcz = itsdk_config.sdk.sigfox.rcz;
	itsdk_state.sigfox.current_power = itsdk_config.sdk.sigfox.txPower;
	itsdk_state.sigfox.current_speed = itsdk_config.sdk.sigfox.speed;
  #elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_CONFIG_STATIC
	uint8_t __rcz = 0;
	itsdk_sigfox_getRczFromRegion(ITSDK_DEFAULT_REGION, &itsdk_state.sigfox.rcz);
	itsdk_state.sigfox.current_power = (ITSDK_SIGFOX_TXPOWER < ITSDK_SIGFOX_MAXPOWER )?ITSDK_SIGFOX_TXPOWER:ITSDK_SIGFOX_MAXPOWER;;
	itsdk_state.sigfox.current_speed = ITSDK_SIGFOX_SPEED;
  #elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	// The setting will be made later during sigfox init part
  #else
    #error INVALID ITSDK_SIGFOX_NVM_SOURCE VALUE
  #endif
#endif

	return;
}

#if ITSDK_WITH_CONSOLE == __ENABLE
void itsdk_print_state() {
	_itsdk_console_printf("state.lastWakeUpTimeUs %d ms\r\n",(uint32_t)(itsdk_state.lastWakeUpTimeUs/1000));
	_itsdk_console_printf("state.lastResetCause 0x%X \r\n",(uint32_t)(itsdk_state.lastResetCause));
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
   #if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
	_itsdk_console_printf("state.activeNetwork : %d\r\n",itsdk_state.activeNetwork);
   #endif
#else
   #if ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE
	_itsdk_console_printf("state.activeNetwork %d\r\n",itsdk_state.activeNetwork);
	#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	#endif
   #endif
#endif
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
	if ( itsdk_state.activeNetwork == __ACTIV_NETWORK_LORAWAN ) {
  	  _itsdk_console_printf("state.lorawan.joined: %c\r\n",(itsdk_lorawan_hasjoined()?'Y':'N'));
	}
#endif
#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
	_itsdk_console_printf("state.sigfox.initialized : %d\r\n",itsdk_state.sigfox.initialized);
  #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM || ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_CONFIG_STATIC
	_itsdk_console_printf("state.sigfox.rcz %d\r\n",itsdk_state.sigfox.rcz);
	if (itsdk_state.sigfox.current_power == SIGFOX_DEFAULT_POWER ) {
		_itsdk_console_printf("state.sigfox.current_power : DEFAULT\r\n");
	} else {
		_itsdk_console_printf("state.sigfox.current_power : %d\r\n",itsdk_state.sigfox.current_power);
	}
	if (itsdk_state.sigfox.current_speed == SIGFOX_DEFAULT_SPEED ) {
		_itsdk_console_printf("state.sigfox.current_speed : DEFAULT\r\n");
	} else {
		_itsdk_console_printf("state.sigfox.current_speed : %d\r\n",itsdk_state.sigfox.current_speed);
	}
  #elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	// The setting will be made later during sigfox init part
  #else
    #error INVALID ITSDK_SIGFOX_NVM_SOURCE VALUE
  #endif
#endif

}
#endif
