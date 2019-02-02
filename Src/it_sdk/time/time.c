/* ==========================================================
 * time.c - Functions about counting time
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
#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/time/time.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#include <stm32l_sdk/rtc/rtc.h>
	#include <stm32l_sdk/time/time.h>
#endif

volatile uint64_t __timeus = 0;
uint8_t  __time_has_overrun = 0;
uint8_t  __time_overrun_cnt = 0;

/**
 * Add the given number of uS to the uS global timer
 */
void itsdk_time_add_us(uint32_t us) {
	// apply correction
	#if ITSDK_CLK_CORRECTION != 0
	us = us + ( (int64_t)us * ITSDK_CLK_CORRECTION ) / 1000;
	#endif
	uint64_t n = __timeus + us;
	if ( n < __timeus  ) {
		__time_has_overrun=1;
		__time_overrun_cnt++;
	}
	__timeus = n;
}

/**
 * Set current time in ms
 */
void itsdk_time_set_ms(uint64_t ms) {
	uint64_t n = ms * 1000L;
	if ( n < __timeus  ) {
		__time_has_overrun=1;
		__time_overrun_cnt++;
	}
	__timeus = n;
}

/**
 * Get current time in ms
 */
uint64_t itsdk_time_get_ms() {
	return __timeus / 1000;
}

/**
 * Get current time in us
 */
uint64_t itsdk_time_get_us() {
	return __timeus;
}


/**
 * Reset the time to 0
 */
void itsdk_time_reset() {
	#if ITSDK_PLATFORM == __PLATFORM_STM32L0
		rtc_resetTime();
	#else
		#error "platform not supported"
	#endif
	__timeus = 0;
}

/**
 * Init time functions
 */
void itsdk_time_init() {
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	rtc_resetTime();
	rtc_adjustTime();
	systick_adjustTime();
#else
	#error "platform not supported"
#endif
}
