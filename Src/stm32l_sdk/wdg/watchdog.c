/* ==========================================================
 * watchdog.c - Manage watchdog functions
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 16 sept. 2018
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
 * Use WDG is the most standard way : ensure the watchdog is
 * called regularly in the main loop until a given time.
 * WDG must be activated and configured in CubeMX
 * LSI is 37KHz
 * - The IWDG_WINDOW is disabled setting 4096 as value
 * - The IWDG_PRESCALER is set to be about 7ms => IWDG_PRESCALER_256 to cover up to 28s
 * - The Reload value is set to match 28s until being changed by the setup function (4096)
 * WDG is refreshed in the main loop so any cycle duration must be
 * lower than the wdg timer configuration, otherwise you need to manually refresh it
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
#if ITSDK_WITH_WDG != __WDG_NONE
#include <it_sdk/wrappers.h>
#include <it_sdk/debug.h>
#include <stm32l_sdk/rtc/rtc.h>
#include <it_sdk/logger/error.h>
#if ITSDK_WITH_WDG == __WDG_IWDG
#include "iwdg.h"
#endif

/**
 * Setup the WatchDog for fireing a reset after the given Ms time
 * Values from 7ms to 28s according to the possible values
 */
void wdg_setupWithMaxMs(uint32_t ms) {

	if ( ms > 28000 || ms < 10 ) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_WDG_OUTOFBOUNDS,(uint16_t)ms);
	}
  #if ITSDK_WDG_MS >0
	int32_t uwLsiFreq;
	uwLsiFreq = (ITSDK_WDG_CLKFREQ * rtc_getClockAdjustement())/1000;

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	uint32_t rel = (ms * ( (uwLsiFreq * 10) / 256 )) / 10000;
	hiwdg.Init.Reload = rel;

	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		/* Initialization Error */
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_WDG_INIT_FAILED,(uint16_t)ms);
	}
	// To enable IWDG freeze during debug session
	//__HAL_DBGMCU_FREEZE_IWDG();
  #else
	#ifdef IWDG
      #error "Watchdog disabled you need to disable it also in CubeMx"
	#endif
  #endif
}


void wdg_refresh() {
	HAL_IWDG_Refresh(&hiwdg);
}
#endif

#endif
