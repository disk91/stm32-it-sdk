/* ==========================================================
 * time.h - 
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 12 sept. 2018
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
#include <stdint.h>
#include <it_sdk/itsdk.h>

#ifndef IT_SDK_TIME_TIME_H_
#define IT_SDK_TIME_TIME_H_

// System Time - the reference is startup
void itsdk_time_add_us(uint32_t us);
void itsdk_time_set_ms(uint64_t ms);
uint64_t itsdk_time_get_ms();
uint64_t itsdk_time_get_us();
void itsdk_time_reset();
void itsdk_time_init();

// UTC time (HH:MM:SS) when the reference has been set by an external driver
void itsdk_time_sync_UTC_s( uint32_t utc_s );				// Access time with reference MIDNIGHT UTC when set (otherwise startup)
uint32_t itsdk_time_get_UTC_s();
itsdk_bool_e itsdk_time_is_UTC_s(uint32_t * destTime);
uint8_t itsdk_time_get_UTC_sec();
uint8_t itsdk_time_get_UTC_min();
uint8_t itsdk_time_get_UTC_hour();

// UTC Time from EPOC when the reference has been set by an external driver
void itsdk_time_sync_EPOC_s( uint32_t utc_s );				// Access time with reference EPOC when set (otherwise startup)
uint32_t itsdk_time_get_EPOC_s();
itsdk_bool_e itsdk_time_is_EPOC_s(uint32_t * destTime);

#endif /* IT_SDK_TIME_TIME_H_ */
