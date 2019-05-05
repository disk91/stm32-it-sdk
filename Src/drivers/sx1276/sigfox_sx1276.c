/* ==========================================================
 * sx1276Sigfox.h - Sigfox implementation on sx1276
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 1 may 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
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
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276)
#include <string.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/time/timer.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/sx1276/sigfox_sx1276.h>

sx1276_sigfox_state_t	sx1276_sigfox_state;

sx1276_sigfox_ret_t sx1276_sigfox_init(itsdk_sigfox_state * sigfox_state) {
	sfx_error_t error = SX1276_SIGFOX_ERR_NONE;
	sfx_rc_t  prcz;
	sfx_u32   pconfig_words[3];

	// Get config from eeprom or static files
	#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
	    sx1276_sigfox_state.currentPower = (uint8_t)itsdk_config.sdk.sigfox.txPower;
	#else
	    sx1276_sigfox_state.currentPower = ITSDK_SIGFOX_TXPOWER;
	#endif
	sx1276_sigfox_state.meas_rssi_dbm = 0;
	sx1276_sigfox_state.rxPacketReceived= STLL_RESET;
	sx1276_sigfox_state.rxCarrierSenseFlag= STLL_RESET;

	switch (sigfox_state->rcz) {
	default:
	case SIGFOX_RCZ1:
		{
			sfx_rc_t rcz = RC1;
			bcopy(&rcz,&prcz,sizeof(rcz));
		}
		break;
	case SIGFOX_RCZ2:
		{
			sfx_rc_t rcz = RC2;
			bcopy(&rcz,&prcz,sizeof(rcz));
			sfx_u32 config_words[3] = RC2_SM_CONFIG;
			bcopy(config_words,pconfig_words,3*sizeof(sfx_u32));
		}
		break;
	case SIGFOX_RCZ3C:
		{
			sfx_rc_t rcz = RC3C;
			bcopy(&rcz,&prcz,sizeof(rcz));
			sfx_u32 config_words[3] = RC3C_CONFIG;
			bcopy(config_words,pconfig_words,3*sizeof(sfx_u32));
		}
		break;
	case SIGFOX_RCZ4:
		{
			sfx_rc_t rcz = RC4;
			bcopy(&rcz,&prcz,sizeof(rcz));
			sfx_u32 config_words[3] = RC4_SM_CONFIG;
			bcopy(config_words,pconfig_words,3*sizeof(sfx_u32));
		}
		break;
	}
	error = SIGFOX_API_open(&prcz);
	switch (sigfox_state->rcz) {
	case SIGFOX_RCZ2:
		error = SIGFOX_API_set_std_config(pconfig_words, RC2_SET_STD_TIMER_ENABLE);
		break;
	case SIGFOX_RCZ3C:
		error = SIGFOX_API_set_std_config(pconfig_words, NA);
		break;
	case SIGFOX_RCZ4:
		error = SIGFOX_API_set_std_config(pconfig_words, RC4_SET_STD_TIMER_ENABLE);
		break;
	default:
		break;
	}
	return error;
}

/**
 * This function is called when the mcu is waiting for the end of an action
 * It executes the needed background tasks during this period.
 * Returns SX1276_SIGFOX_ERR_BREAK when we want to force breaking the loop
 */
sx1276_sigfox_ret_t sx1276_sigfox_idle( void ) {
	itsdk_stimer_run();

	return sx1276_sigfox_idle_used();
}

/**
 * This function can be override for a custom function executed in background
 * during the sigfox transmit & receive phases
 */
__weak sx1276_sigfox_ret_t sx1276_sigfox_idle_used( void ) {
	return SX1276_SIGFOX_ERR_NONE;
}

#endif
