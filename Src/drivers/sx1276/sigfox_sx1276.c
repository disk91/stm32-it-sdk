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
#include <it_sdk/eeprom/sdk_state.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/time/time.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/sigfox/se_nvm.h>
#include <drivers/sx1276/sigfox_sx1276.h>
#include <drivers/sx1276/sx1276.h>

sx1276_sigfox_state_t	sx1276_sigfox_state;

/**
 * Configure the sigfox stack for sx1276
 */
sx1276_sigfox_ret_t sx1276_sigfox_init( void ) {
	static sfx_rc_t  __sfx1276_prcz;
	static sfx_u32   __sfx1276_pconfig_words[3];
	LOG_INFO_SFXSX1276((">> sx1276_sigfox_init\r\n"));

	sfx_error_t error = SX1276_SIGFOX_ERR_NONE;

	// Hardware Init
	SX1276IoInit();

	STLL_Radio_Init();

	// set DIO3 from 'buffer empty' to NA to save current
	SX1276Write( 0x40, 0x01 );

	sx1276_sigfox_state.lastHseSwitch_S = itsdk_time_get_ms()/1000;
	sx1276_sigfox_state.meas_rssi_dbm = 0;
	sx1276_sigfox_state.rxPacketReceived= STLL_RESET;
	sx1276_sigfox_state.rxCarrierSenseFlag= STLL_RESET;

	switch (itsdk_state.sigfox.rcz) {
	default:
	case SIGFOX_RCZ1:
		{
			sfx_rc_t rcz = RC1;
			bcopy(&rcz,&__sfx1276_prcz,sizeof(rcz));
		}
		break;
	case SIGFOX_RCZ2:
		{
			sfx_rc_t rcz = RC2;
			bcopy(&rcz,&__sfx1276_prcz,sizeof(rcz));
			sfx_u32 config_words[3] = RC2_SM_CONFIG;
			bcopy(config_words,__sfx1276_pconfig_words,3*sizeof(sfx_u32));
		}
		break;
	case SIGFOX_RCZ3C:
		{
			sfx_rc_t rcz = RC3C;
			bcopy(&rcz,&__sfx1276_prcz,sizeof(rcz));
			sfx_u32 config_words[3] = RC3C_CONFIG;
			bcopy(config_words,__sfx1276_pconfig_words,3*sizeof(sfx_u32));
		}
		break;
	case SIGFOX_RCZ4:
		{
			sfx_rc_t rcz = RC4;
			bcopy(&rcz,&__sfx1276_prcz,sizeof(rcz));
			sfx_u32 config_words[3] = RC4_SM_CONFIG;
			bcopy(config_words,__sfx1276_pconfig_words,3*sizeof(sfx_u32));
		}
		break;
	}
	LOG_INFO_SFXSX1276((">> SIGFOX_API_open\r\n"));
	sfx_error_t serror = SIGFOX_API_open(&__sfx1276_prcz);

	if ( serror != SFX_ERR_NONE ) {
		LOG_ERROR_SFXSX1276(("[ERROR] Sigfox Open(%08X)\r\n",serror));
		return SX1276_SIGFOX_ERR_LIBINIT;
	}

	switch (itsdk_state.sigfox.rcz) {
	case SIGFOX_RCZ2:
		error = SIGFOX_API_set_std_config(__sfx1276_pconfig_words, RC2_SET_STD_TIMER_ENABLE);
		break;
	case SIGFOX_RCZ3C:
		error = SIGFOX_API_set_std_config(__sfx1276_pconfig_words, NA);
		break;
	case SIGFOX_RCZ4:
		error = SIGFOX_API_set_std_config(__sfx1276_pconfig_words, RC4_SET_STD_TIMER_ENABLE);
		break;
	default:
		break;
	}

	if ( error != SFX_ERR_NONE ) {
		LOG_ERROR_SFXSX1276(("[ERROR] Sigfox set config(%08X)\r\n",error));
		error = SX1276_SIGFOX_ERR_CONFIG;
	}

	return error;
}


/**
 * DeInit Sigfox Stack
 */
sx1276_sigfox_ret_t sx1276_sigfox_deinit( void ) {
	LOG_INFO_SFXSX1276((">> sx1276_sigfox_deinit\r\n"));

	SX1276IoDeInit();
	return SX1276_SIGFOX_ERR_NONE;
}


/**
 * Change power
 */
sx1276_sigfox_ret_t sx1276_sigfox_setPower( int8_t power ) {
	LOG_INFO_SFXSX1276((">> sx1276_sigfox_setPower\r\n"));

	STLL_RadioPowerSetBoard(power);
    return SX1276_SIGFOX_ERR_NONE;
}

/**
 * Return the sequence Id
 */
sx1276_sigfox_ret_t sx1276_sigfox_getSeqId( uint16_t * seqId ) {
	LOG_INFO_SFXSX1276((">> sx1276_sigfox_getSeqId\r\n"));
	sfx_u8 read_data[SFX_SE_NVMEM_BLOCK_SIZE];
	SE_NVM_get(read_data);
    *seqId = (read_data[SE_NVMEM_SEQNUM]+(read_data[SE_NVMEM_SEQNUM+1] << 8)) & 0xFFF;
    return SX1276_SIGFOX_ERR_NONE;
}

/**
 * Return the last Rssi value
 */
sx1276_sigfox_ret_t sx1276_sigfox_getRssi(int16_t * rssi) {
	LOG_INFO_SFXSX1276((">> sx1276_sigfox_getRssi\r\n"));

  *rssi = sx1276_sigfox_state.meas_rssi_dbm;
  return SX1276_SIGFOX_ERR_NONE;
}

/**
 * This function is called when the mcu is waiting for the end of an action
 * It executes the needed background tasks during this period.
 * Returns SX1276_SIGFOX_ERR_BREAK when we want to force breaking the loop
 */
#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
static uint32_t __xx = 0;
#endif
sx1276_sigfox_ret_t sx1276_sigfox_idle( void ) {
#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
	if ( (__xx & 0x000FFFF) == 0 ) LOG_INFO_SFXSX1276(("I\r\n"));
	__xx++;
#endif
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
