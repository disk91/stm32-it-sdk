/* ==========================================================
 * sigfox_sx126x.h - implementation for sigfox library, general headers
 * ----------------------------------------------------------
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
 * ---------------------------------------------------------
 *
 *  Created on: 11 june 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#ifndef IT_SDK_DRIVERS_SX126X_SIGFOX_H_
#define IT_SDK_DRIVERS_SX126X_SIGFOX_H_

#include <it_sdk/config.h>
#include <it_sdk/logger/logger.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX126X)

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
#define LOG_INFO_SFXSX126X(x)		log_info x
#define LOG_WARN_SFXSX126X(x) 		log_warn x
#define LOG_ERROR_SFXSX126X(x)		log_error x
#define LOG_DEBUG_SFXSX126X(x)		log_debug x
#else
#define LOG_INFO_SFXSX126X(x)
#define LOG_WARN_SFXSX126X(x)
#define LOG_ERROR_SFXSX126X(x)
#define LOG_DEBUG_SFXSX126X(x)
#endif

#endif

#endif // IT_SDK_DRIVERS_SX126X_SIGFOX_H_
