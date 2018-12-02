/* ==========================================================
 * sigfox_helper.h - Prototypes for Sigfox helper functions
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 9 nov. 2018
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
 *
 * ==========================================================
 */

#ifndef IT_SDK_DRIVERS_SIGFOX_HELPER_H_
#define IT_SDK_DRIVERS_SIGFOX_HELPER_H_

#include <it_sdk/config.h>
#include <stdbool.h>
#include <drivers/s2lp/s2lp.h>

bool s2lp_sigfox_init(s2lp_config_t * conf);
void s2lp_sigfox_cifferKey(s2lp_config_t * conf);
void s2lp_sigfox_unCifferKey(s2lp_config_t * conf);

bool s2lp_sigfox_retreive_key(int32_t deviceId, uint8_t * pac, uint8_t * key);
void enc_protect_key();
void enc_unprotect_key();
void itsdk_sigfox_configInit(s2lp_config_t * cnf);
int16_t s2lp_sigfox_getLastRssiLevel();
void s2lp_sigfox_retreive_rssi();

extern s2lp_config_t *	_s2lp_sigfox_config;

#define S2LP_UNKNOWN_RSSI	0xFF;

/* ----------------------------------------------------------------
 * Logging
 */

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
#define LOG_INFO_S2LP(x)		log_info x
#define LOG_WARN_S2LP(x) 		log_warn x
#define LOG_ERROR_S2LP(x)		log_error x
#define LOG_DEBUG_S2LP(x)		log_debug x
#else
#define LOG_INFO_S2LP(x)
#define LOG_WARN_S2LP(x)
#define LOG_ERROR_S2LP(x)
#define LOG_DEBUG_S2LP(x)
#endif




#endif /* IT_SDK_DRIVERS_SIGFOX_HELPER_H_ */
