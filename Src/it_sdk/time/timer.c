/* ==========================================================
 * timer.c - Long software timer functions
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 25 nov. 2018
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
 * Manage timer with long duration based on the internal ms counter
 * The timer function is called during the device wakeup operations
 * the precision can't be higher than the wakeup frequency defined
 * by ITSDK_LOWPOWER_RTC_MS
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/time/time.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/logger/logger.h>

#if ITSDK_PLATFORM == __PLATFORM_STM32L0
   #include <stm32l_sdk/timer/timer.h>
#endif

/**
 * ===============================================================================
 *                         HARDWARE TIMER
 * ===============================================================================
 */

#if ITSDK_WITH_HW_TIMER > 0

/**
 * Run the timer for the given time. Sync mode, return only after timer execution
 * At end the callback_func is called with value as parameter. If NULL the function
 * just return.
 */
itsdk_timer_return_t itsdk_hwtimer_sync_run(
		uint32_t ms,
		void (*callback_func)(uint32_t value),
		uint32_t value
) {

	#if ITSDK_PLATFORM == __PLATFORM_STM32L0
		return stm32l_hwtimer_sync_run(ms,callback_func,value);
	#else
		#error "platform not supported"
	#endif

}


#endif



/**
 * ===============================================================================
 *                          SOFTWARE TIMER
 * ===============================================================================
 */


#if ITSDK_TIMER_SLOTS > 0

itsdk_stimer_slot_t	__stimer_slots[ITSDK_TIMER_SLOTS] = {0};

/**
 * Register a new timer in the timer list
 * The list size is defined by ITSDK_TIMER_SLOTS
 */
itsdk_timer_return_t itsdk_stimer_register(
		uint32_t ms,
		void (*callback_func)(uint32_t value),
		uint32_t value
) {
	if ( ms < ITSDK_LOWPOWER_RTC_MS ) {
		#if (ITSDK_LOGGER_MODULE & __LOG_MOD_STIMER) > 0
		  log_error("STimer try to register too short timer\r\n");
		#endif
		return TIMER_TOO_SHORT;
	}

	int i = 0;
	while ( i < ITSDK_TIMER_SLOTS) {
		if (__stimer_slots[i].inUse == false) {
			break;
		}
		i++;
	}
	if ( i < ITSDK_TIMER_SLOTS ) {
		__stimer_slots[i].inUse = true;
		__stimer_slots[i].customValue = value;
		__stimer_slots[i].callback_func = callback_func;
		__stimer_slots[i].timeoutMs = itsdk_time_get_ms()+ms;
		return TIMER_INIT_SUCCESS;
	}
	#if (ITSDK_LOGGER_MODULE & __LOG_MOD_STIMER) > 0
	  log_error("STimer list full\r\n");
	#endif
	return TIMER_LIST_FULL;
}

/**
 * Stop a running timer
 * identified by function pointer & value
 */
itsdk_timer_return_t itsdk_stimer_stop(
		void (*callback_func)(uint32_t value),
		uint32_t value
) {
	for (int i=0 ; i < ITSDK_TIMER_SLOTS ; i++) {
		if (
				__stimer_slots[i].inUse == true
			&&  __stimer_slots[i].customValue == value
			&&  __stimer_slots[i].callback_func == callback_func
		) {
			// found
			__stimer_slots[i].inUse = false;
			return TIMER_INIT_SUCCESS;
		}
	}
	return TIMER_NOT_FOUND;
}

/**
 * Return true is the given timer is still running
 * identified by function pointer & value
 */
bool itsdk_stimer_isRunning(
		void (*callback_func)(uint32_t value),
		uint32_t value
) {
	for (int i=0 ; i < ITSDK_TIMER_SLOTS ; i++) {
		if (
				__stimer_slots[i].inUse == true
			&&  __stimer_slots[i].customValue == value
			&&  __stimer_slots[i].callback_func == callback_func
		) {
			// found
			return true;
		}
	}
	return false;
}


/**
 * Run the software timer execution. Call this function as much as
 * possible. At least on every wake-up from sleep
 */
void itsdk_stimer_run() {
	uint32_t t = itsdk_time_get_ms();
	for ( int i = 0 ; i < ITSDK_TIMER_SLOTS ; i++ ) {
		if ( __stimer_slots[i].inUse && __stimer_slots[i].timeoutMs <= t ) {
			__stimer_slots[i].inUse = false;
			if (__stimer_slots[i].callback_func != NULL )
				__stimer_slots[i].callback_func(__stimer_slots[i].customValue);
		}
	}
}


#endif
