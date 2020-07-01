/* ==========================================================
 * lowpower.c - Lowpower sdk functions
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
#include <it_sdk/config.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/time/time.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/eeprom/sdk_state.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#include <stm32l_sdk/lowpower/lowpower.h>
	#include <stm32l_sdk/rtc/rtc.h>
#endif
#if ITSDK_TIMER_SLOTS > 0
	#include <it_sdk/time/timer.h>
#endif
#if ITSDK_SHEDULER_TASKS > 0
	#include <it_sdk/sched/scheduler.h>
#endif

lowPower_wu_reason_t __lowPower_wakeup_reason = LOWPWR_WAKEUP_UNDEF;
static lowPower_state_e __lowPowerState = LOWPRW_ENABLE;
/**
 * Switch to low power mode selected for the expected platform
 */
void __attribute__((optimize("O3"))) lowPower_switch() {

	if (__lowPowerState==LOWPRW_ENABLE) {
		// Ensure we will wake up at next softTimer end or Task end.
		uint32_t duration = __INFINITE_32B;
		#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_RTC ) > 0
			duration = ITSDK_LOWPOWER_RTC_MS;
		#endif
		#if ITSDK_SHEDULER_TASKS > 0
			uint32_t schedDur = itdt_sched_nextRun();
			if ( schedDur < duration ) duration = schedDur;
		#endif
		#if ITSDK_TIMER_SLOTS > 0
			uint32_t maxDur = itsdk_stimer_nextTimeoutMs();
			if ( maxDur < duration ) duration = maxDur;
		#endif
		#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_RTC ) == 0
			if ( duration != __INFINITE_32B ) {
				// We have RTC disable but we need to verify time for timer or task
				// so we can't jump into sleep mode until the timer end.
				return;
			}
		#endif
		if ( duration > ITSDK_LOWPOWER_MINDUR_MS ) {
			#if ITSDK_PLATFORM == __PLATFORM_STM32L0
			// sleeping
			if ( stm32l_lowPowerSetup(duration,STM32L_LOWPOWER_NORMAL_STOP) == STM32L_LOWPOWER_SUCCESS ) {
				// waking up
				stm32l_lowPowerResume(STM32L_LOWPOWER_NORMAL_STOP);
				itsdk_state.lastWakeUpTimeUs = itsdk_time_get_us();
			}
			#endif
		}
	}

}

/**
 * Disable LowPower mode
 */
void lowPower_enable() {
	__lowPowerState= LOWPRW_ENABLE;
}

/**
 * Enable LowPower mode
 */
void lowPower_disable() {
	__lowPowerState= LOWPRW_DISABLE;
}

/**
 * Have a delay in low power with no wake-up reason other than RTC
 * end. The duration won't be respected if a timer ends before the
 * expected duration. The pending duration is returned
 */
uint32_t lowPower_delayMs(uint32_t duration) {
	uint32_t pendingDur = 0;
	uint32_t maxDur = itsdk_stimer_nextTimeoutMs();
	if ( maxDur < duration ) {
		pendingDur = duration - maxDur;
		duration = maxDur;
	}
	if ( itsdk_stimer_isLowPowerSwitchAutorized()  && __lowPowerState == LOWPRW_ENABLE ) {
		if ( duration > ITSDK_LOWPOWER_MINDUR_MS ) {
			#if ITSDK_PLATFORM == __PLATFORM_STM32L0
			// sleeping
			if ( stm32l_lowPowerSetup(duration,STM32L_LOWPOWER_RTCONLY_STOP) == STM32L_LOWPOWER_SUCCESS ) {
				// waking up
				stm32l_lowPowerResume(STM32L_LOWPOWER_RTCONLY_STOP);
			}
			#endif
		} else {
			itsdk_delayMs(duration);
		}
	} else {
		itsdk_delayMs(duration);
	}

	return pendingDur;
}
