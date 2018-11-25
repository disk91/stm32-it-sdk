/* ==========================================================
 * sigfox_helper.c - Sigfox helper
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 09 nov. 2018
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
#include <it_sdk/config.h>
#if ITSDK_WITH_SIGFOX_LIB > 0

#include <it_sdk/itsdk.h>
#include <drivers/s2lp/s2lp.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/sigfox/sigfox_helper.h>
#include <it_sdk/logger/logger.h>

/**
 * Init the sigfox library according to the configuration
 * passed as parameter.
 * return true when success.
 */
bool sigfox_init(s2lp_config_t * conf) {

	switch ( conf->rcz ) {
	case 1:
		SIGFOX_API_open(&(sfx_rc_t)RC1);
		break;
	}

	return true;
}


/**
 * Protect the private key in memory
 *  - basic Xor ... better than nothing
 */
void sigfox_cifferKey(s2lp_config_t * conf) {
	 uint32_t key = ITSDK_SIGFOX_PROTECT_KEY;
	 uint32_t * pk = (uint32_t *)(conf->key);
	 for ( int i = 0  ; i < 4 ; i++,pk++ ) *pk ^= key;
}
void sigfox_unCifferKey(s2lp_config_t * conf) {
	sigfox_cifferKey(conf);
}



#endif // ITSDK_WITH_SIGFOX_LIB test

