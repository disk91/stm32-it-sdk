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
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/sx1276/sx1276Sigfox.h>


itsdk_sigfox_init_t sx1276_sigfox_init(itsdk_sigfox_state * sigfox_state) {
	sfx_error_t error = SFX_ERR_NONE;
	sfx_rc_t  prcz;
	sfx_u32   pconfig_words[3];
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



#endif
