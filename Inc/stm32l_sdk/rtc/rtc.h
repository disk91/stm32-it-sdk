/* ==========================================================
 * rtc.h - headers for Real Time Clock
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
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

#ifndef STM32L_SDK_RTC_RTC_H_
#define STM32L_SDK_RTC_RTC_H_

#include <stdbool.h>
#include <it_sdk/config.h>


void rtc_configure4LowPower(uint16_t ms);
void rtc_disable4LowPower();
uint64_t rtc_getTimestampMs();
void rtc_prepareSleepTime();
void rtc_updateTimeAfterSleepTime();

#if ITSDK_WITH_CLK_ADJUST > 0
uint32_t rtc_getRealRtcFrequency();
#endif

extern bool __enable_systick;

#endif /* STM32L_SDK_RTC_RTC_H_ */
