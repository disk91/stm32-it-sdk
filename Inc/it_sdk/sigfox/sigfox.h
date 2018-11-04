/* ==========================================================
 * sigfox.h - Sigfox communication abstraction layer
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 04 nov. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
#ifndef IT_SDK_SIGFOX_H_
#define IT_SDK_SIGFOX_H_

#include <stdbool.h>

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>

#ifdef ITSDK_WITH_SIGFOX_LIB

// --------------------------------------------------------------------
// Public Functions
// --------------------------------------------------------------------

uint8_t itsdk_sigfox_setup();
uint8_t itsdk_sigfox_sendFrame(char * buf, uint8_t len, uint8_t repeat, uint16_t speed, uint8_t power, bool ack);
uint8_t itsdk_sigfox_sendBit(char bitValue,  uint8_t repeat, uint16_t speed, uint8_t power, bool ack);


#endif //ITSDK_WITH_SIGFOX_LIB

#endif // IT_SDK_SIGFOX_H_

