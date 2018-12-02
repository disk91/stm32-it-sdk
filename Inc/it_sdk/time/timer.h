/* ==========================================================
 * timer.h - Long Timer Functions
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
 *
 * ==========================================================
 */
#ifndef IT_SDK_TIME_TIMER_H_
#define IT_SDK_TIME_TIMER_H_

#include <stdbool.h>


typedef enum {
	TIMER_INIT_SUCCESS=0,
	TIMER_TOO_SHORT,				// requested duration too short for this type of timer
	TIMER_TOO_LONG,					// requested duration too long for this type of timer
	TIMER_LIST_FULL,				// None of the timer slot is available
	TIMER_NOT_FOUND					// Given timer not found when trying to remove it

} itsdk_timer_return_t;

// =====================================================================================
// HW TIMERS
// =====================================================================================

itsdk_timer_return_t itsdk_hwtimer_sync_run(
		uint32_t ms,
		void (*callback_func)(uint32_t value),
		uint32_t value
);

// =====================================================================================
// SOFT TIMERS
// =====================================================================================


typedef struct s_itsdk_stimer_slot {
	bool			inUse;								// This structure is in use
	uint32_t		timeoutMs;							// End of the timer value
	void 			(*callback_func)(uint32_t value);	// Callback function
	uint32_t		customValue;
} itsdk_stimer_slot_t;

itsdk_timer_return_t itsdk_stimer_register(
		uint32_t ms,									// timer duration
		void (*callback_func)(uint32_t value),			// callback function
		uint32_t value									// value to pass to callback function
);

itsdk_timer_return_t itsdk_stimer_stop(
		void (*callback_func)(uint32_t value),
		uint32_t value
);

bool itsdk_stimer_isRunning(
		void (*callback_func)(uint32_t value),
		uint32_t value
);

void itsdk_stimer_run();

#endif /* IT_SDK_TIME_TIME_H_ */
