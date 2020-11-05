/* ==========================================================
 * scheduler.h - Manage recurrent tasks
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
 * Try to be tinny as possible ;)
 *
 * ==========================================================
 */

#ifndef IT_SDK_SCHEDULER_H_
#define IT_SDK_SCHEDULER_H_

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>

#define ITSDK_SCHED_CONF_DEFAULT	0x00
#define ITSDK_SCHED_CONF_SKIP		0x01		// SKIP mode will avoid the scheduler to run twice (or more)
												// if it starts to be late and iteration have been missed
												// with not SKIP the scheduler executer a task on every period but
												// it can be consecutive, not after period time.
#define ITSDK_SCHED_CONF_HALT		0x02		// The scheduler is halted (1) / Running (0)

#define ITSDK_SCHED_CONF_IMMEDIATE	0x100		// The task is immediately executed vs will be executed in next time period


#define ITSDK_SCHED_ERROR			0xFF

#define ITSDK_SCHED_MAX_PERIOD		0x00FFFFFF	// 24b is the max duration
/**
 * schedule configuration
 */
typedef struct s_sched {
	uint64_t nextRun;			// Next run time in ms
	uint32_t period:24;			// Period between 2 execution (24 bits)
								// Mode as a bitfield for task configuration see ITSDK_SCHED_CONF_XXX
	uint8_t  skip:1;			//    skip missed execution
	uint8_t	 halt:1;			//    stop for next executions
	void (*func)(void);			// function to call
} sched_t;


/**
 * Functions
 */
void itdt_sched_execute();
uint8_t itdt_sched_registerSched(uint32_t periodMs,uint16_t mode, void (*f)(void));
void itdt_sched_haltSched(uint8_t schedId);
void itdt_sched_runSched(uint8_t schedId);
uint32_t itdt_sched_nextRun();
void itdt_sched_clearNextRun(uint8_t schedId);		// Set nextRun timestamp to now + period
uint8_t itdt_sched_changeSched(uint8_t schedId, uint32_t periodMs, void (*f)(void) );

#endif 	// IT_SDK_SCHEDULER_H_
