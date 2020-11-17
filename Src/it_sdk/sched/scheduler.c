/* ==========================================================
 * scheduler.c - Manage recurrent task
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
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
 * Actually the task are allocated one after the previous one
 * so we can't dynamically add/remove task. An improvement could
 * be to search for empty task to allocate it.
 *
 * ==========================================================
 */

#include <it_sdk/sched/scheduler.h>
#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/debug.h>

#if ITSDK_SHEDULER_TASKS > 0
sched_t __scheds[ITSDK_SHEDULER_TASKS];
uint8_t __sNum = 0;


/**
 * Register a new task in the scheduler with the given period in Ms and the
 * associated function to call. The mode params defines the scheduler behavior
 * Returns the scedId on success or ITSDK_SCHED_ERROR on error.
 */
uint8_t itdt_sched_registerSched(uint32_t periodMs,uint16_t mode, void (*f)(void)) {

	if ( periodMs > ITSDK_SCHED_MAX_PERIOD ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_SCHED_DURATION_OVERFLOW,0);
		return ITSDK_SCHED_ERROR;
	}
	if ( __sNum < ITSDK_SHEDULER_TASKS ) {
		__scheds[__sNum].func=f;
		__scheds[__sNum].period=periodMs;
		__scheds[__sNum].nextRun=(mode & ITSDK_SCHED_CONF_IMMEDIATE)?itsdk_time_get_ms():itsdk_time_get_ms()+periodMs;
		__scheds[__sNum].halt=(mode & ITSDK_SCHED_CONF_HALT)?1:0;
		__scheds[__sNum].skip=(mode & ITSDK_SCHED_CONF_SKIP)?1:0;
		__sNum++;
		return __sNum-1;
	} else return ITSDK_SCHED_ERROR;

}

/**
 * Task executor
 */
void itdt_sched_execute() {

	uint64_t t = itsdk_time_get_ms();
	for (int i = 0 ; i < __sNum ; i++) {
		do {
			if ( __scheds[i].nextRun <= t ) {
	 		    __scheds[i].nextRun += (uint64_t)__scheds[i].period;
				if ( !__scheds[i].halt ) {
					_LOG_SCHED(("[sched] (%d) exec @%ld\r\n",i,t));
					(*__scheds[i].func)();
				}
			}
		} while (!__scheds[i].skip && __scheds[i].nextRun <= t );
		while (__scheds[i].skip &&__scheds[i].nextRun <= t) __scheds[i].nextRun += __scheds[i].period;
		_LOG_SCHED(("[sched] (%d) next @%ld\r\n",i,__scheds[i].nextRun));
	}

}

/**
 * Disable a task
 */
void itdt_sched_haltSched(uint8_t schedId) {
	__scheds[schedId].halt = 1;
	_LOG_SCHED(("[sched] (%d) halted\r\n",schedId));
}

/**
 * Change sched configuration. This is only possible on a disable sched
 * periodMs = 0 - do not change the period
 * f = NULL - do not change the callback function
 * Return ITSDK_SCHED_ERROR in case of error; sched Id otherwise
 */
uint8_t itdt_sched_changeSched(uint8_t schedId, uint32_t periodMs, void (*f)(void) ) {
	if ( schedId >= 0 && schedId < __sNum && __scheds[schedId].halt == 1 ) {
		if ( periodMs <= ITSDK_SCHED_MAX_PERIOD ) {
			if ( periodMs > 0 ) {
				__scheds[schedId].period=periodMs;
			}
			if ( f != NULL ) {
				__scheds[schedId].func=f;
			}
		} else {
			return ITSDK_SCHED_ERROR;
		}
	} else {
		return ITSDK_SCHED_ERROR;
	}
	_LOG_SCHED(("[sched] (%d) updated to %d ms\r\n",schedId,periodMs));
	return schedId;
}

/**
 * Enable a task
 */
void itdt_sched_runSched(uint8_t schedId) {
	__scheds[schedId].halt = 0;
	__scheds[schedId].nextRun = itsdk_time_get_ms()+__scheds[schedId].period;
	_LOG_SCHED(("[sched] (%d) restarted\r\n",schedId));
}

/**
 * Clean the potential storm run we can have when a sched stopped during a certain time.
 * The normal behavior is to have the scheduler running on time and we can have multiple
 * execution with no pause if the scheduler is running late.
 * This function clear the last processing date with the current processing date to avoid this
 * behavior.
 */
void itdt_sched_clearNextRun(uint8_t schedId) {
	__scheds[schedId].nextRun = itsdk_time_get_ms()+__scheds[schedId].period;
	_LOG_SCHED(("[sched] (%d) nexRun cleared\r\n",schedId));
}

/**
 * Return time in ms to the next task running
 */
uint32_t itdt_sched_nextRun() {
	uint64_t min = 0;
	for (int i = 0 ; i < __sNum ; i++) {
		if (  !__scheds[i].halt && (min == 0 || min > __scheds[i].nextRun) ) {
			min = __scheds[i].nextRun;
		}
	}
	if ( min > 0 ) {
		uint64_t t = itsdk_time_get_ms();
		if ( min >= t ){
			return (uint32_t)(min - t);
		} else {
			return 0;
		}
	} else {
		return __INFINITE_32B; // max duration
	}
}

#endif
