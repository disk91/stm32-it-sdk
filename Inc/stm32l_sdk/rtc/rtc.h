/* ==========================================================
 * rtc.h - headers for Real Time Clock
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
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

#ifndef STM32L_SDK_RTC_RTC_H_
#define STM32L_SDK_RTC_RTC_H_

#include <stdbool.h>
#include <stdint.h>
#include <it_sdk/config.h>
#if ITSDK_WITH_RTC != __RTC_NONE
#include <rtc.h>

typedef struct s_rtc_irq_chain {
	void (*irq_func)(RTC_HandleTypeDef *h);
	struct s_rtc_irq_chain * next;
} rtc_irq_chain_t;
void rtc_registerIrqAction(rtc_irq_chain_t * chain);
void rtc_removeIrqAction(rtc_irq_chain_t * chain);
bool rtc_existAction(rtc_irq_chain_t * chain);

void rtc_configure4LowPower(uint16_t ms);
void rtc_disable4LowPower();
uint64_t rtc_getTimestampMs();
uint64_t rtc_getTimestampMsRaw(bool adjust);
void rtc_prepareSleepTime();
void rtc_updateTimeAfterSleepTime();

void rtc_runRtcUntil(uint16_t ms);
void rtc_runRtcUntilTicks(uint32_t ticks);
void rtc_runRtcUntilMs(uint32_t ms);
uint32_t rtc_getTicksFromDuration(uint32_t ms);
int32_t rtc_getMsFromTicks(uint32_t ticks);
void rtc_disableWakeUp();

void rtc_resetTime();

uint32_t rtc_calcClockRatio();
void rtc_adjustTime();
uint32_t rtc_getClockAdjustement();

#if ITSDK_WITH_CLK_ADJUST > 0
uint32_t rtc_getRealRtcFrequency();
#endif

extern bool __enable_systick;

#endif // ITSDK_WITH_RTC

#endif /* STM32L_SDK_RTC_RTC_H_ */
