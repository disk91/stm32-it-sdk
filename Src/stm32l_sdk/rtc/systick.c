/* ==========================================================
 * systick.c - Manage time with systicks
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
 * The systick timer is configured by default to be updated
 * every ms.
 *
 * ==========================================================
 */
#include <it_sdk/itsdk.h>
#include <it_sdk/time/time.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <stm32l0xx_hal.h>
#if ITSDK_CLK_BEST_SOURCE == __CLK_BEST_SRC_RTC
	#include <stm32l_sdk/rtc/rtc.h>
#endif
#if ITSDK_CLK_BEST_SOURCE == __CLK_BEST_SRC_CLK
	#include <stm32l_sdk/timer/timer.h>
#endif

bool 		__enable_systick = true;
#if ITSDK_WITH_CLK_ADJUST > 0
   uint32_t	__systick_ratio = 1000;
#else
   #define 	__systick_ratio   1000
#endif

#ifdef ITSDK_CORE_CLKFREQ
	#if ITSDK_CORE_CLKFREQ > 16000000
		#define __TICK_DURATION_US ( 1000 / (ITSDK_CORE_CLKFREQ/16000000) )
	#else
		#define __TICK_DURATION_US ( 1000 * (16000000/ITSDK_CORE_CLKFREQ) )
	#endif
#else
	#define __TICK_DURATION_US 1000
#endif

/**
 * Action to be executed on Systick
 * The name vary depending on the Firwmare version ... ST, you make me crazy !
 */
void HAL_IncTick(void) {
	// add 1ms to the global counter
	if (__enable_systick) itsdk_time_add_us((__TICK_DURATION_US*__systick_ratio)/1000);
	uwTick++;
	__lowPower_wakeup_reason = LOWPWR_WAKEUP_SYSTICK;
}
void HAL_SYSTICK_Callback(void) {
	// add 1ms to the global counter
	if (__enable_systick) itsdk_time_add_us((__TICK_DURATION_US*__systick_ratio)/1000);
	__lowPower_wakeup_reason = LOWPWR_WAKEUP_SYSTICK;
}


/**
 * Update the correction tickRatio => realTicks = (calcTickRatio * seenTicks)/1000
 */
void systick_adjustTime() {
#if ITSDK_WITH_CLK_ADJUST > 0
	uint64_t start_clk = itsdk_time_get_us()/1000;
	#if ITSDK_CLK_BEST_SOURCE == __CLK_BEST_SRC_RTC
		#if ITSDK_WITH_RTC == __RTC_ENABLED
			uint64_t start_rtc = rtc_getTimestampMsRaw(false);
			while ( (rtc_getTimestampMsRaw(false)-start_rtc) < 200 );	// wait for 200ms
		#else
		   #error 'RTC IS DISABLED, CAN BE USED AS A CLK SOURCE'
		#endif
	#elif ITSDK_CLK_BEST_SOURCE == __CLK_BEST_SRC_CLK
		itsdk_hwtimer_sync_run(
				200,
				NULL,
				0
		);
	#else
		#error 'INVALID BEST CLK SRC'
	#endif
	uint64_t stop_clk = itsdk_time_get_us()/1000;
	uint64_t ratio = (1000*200)/(stop_clk-start_clk);
	//log_info("ticks : %d / 200ms ==> %d\r\n",(int)(stop_clk-start_clk),(int)ratio);

	// Protection against value too bad, sounds like a problem
	if ( ratio > 1400 || ratio < 600 ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_TICKS_INVALID_CLKRATIO,(uint16_t)ratio);
		ratio = 1000;
	}
	__systick_ratio=(uint32_t)ratio;
#endif
}
