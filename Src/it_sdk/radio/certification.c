/* ==========================================================
 * certification.c - For radio certification
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 22 nov. 2020
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

#include <it_sdk/itsdk.h>
#include <it_sdk/config.h>
#include <it_sdk/radio/certification.h>


#if ITSDK_LORAWAN_LIB == __LORAWAN_SX1276 || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
#include <drivers/sx1276/sx1276.h>
#endif

/**
 * Start a continuous wave transmission on the given frequency in Hz and given power un dBm
 * The waves stops after the given duration. 0 for infinite.
 */
itsdk_bool_e startContinousWaveTransmission(uint32_t frequency, int8_t power, uint32_t durationMs ) {
#if ITSDK_LORAWAN_LIB == __LORAWAN_SX1276 || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	SX1276IoInit();
	SX1276Init( NULL );
	if ( durationMs == 0 || durationMs > 65535000 ) durationMs = 65535000;
	SX1276SetTxContinuousWave(frequency, power, (durationMs / 1000));
	return BOOL_TRUE;
#else
	return BOOL_FALSE;
#endif
}

/**
 * Stop continuous wave when it is possible return BOOL_FALSE when not possible
 */
itsdk_bool_e stopContinousWaveTransmission( void ) {
#if ITSDK_LORAWAN_LIB == __LORAWAN_SX1276 || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	return BOOL_FALSE;
#else
	return BOOL_FALSE;
#endif
}
