/* ==========================================================
 * debug.h - Debug macro per library
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
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

#ifndef IT_SDK_DEBUG_H_
#define IT_SDK_DEBUG_H_

#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>

#define ITSDK_DEBUG_ERROR	1
#define ITSDK_DEBUG_SCHED	0
#define ITSDK_DEBUG_STATEM	0
#define ITSDK_DEBUG_EEPROM	0


#if ITSDK_DEBUG_SCHED > 0
#define _LOG_SCHED(x)	log_debug x
#else
#define _LOG_SCHED(x)
#endif

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_STATEMINF) > 0 || (ITSDK_LOGGER_MODULE & __LOG_MOD_STATEMDBG) > 0
#define _LOG_STATEM(x)	log_debug x
#else
#define _LOG_STATEM(x)
#endif

#if ITSDK_DEBUG_EEPROM > 0
#define _LOG_EEPROM(x)	log_debug x
#else
#define _LOG_EEPROM(x)
#endif


#endif /* IT_SDK_DEBUG_H_ */
